/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nap/nap_common.h"

#include <math.h>
#include <string.h>

#include "axi_dma.h"
#include "board.h"
#include "ext_events/ext_events.h"
#include "main.h"
#include "manage.h"
#include "nap/track_channel.h"
#include "nap_constants.h"
#include "nap_hw.h"
#include "piksi_systime.h"
#include "sbp.h"
#include "system_monitor/system_monitor.h"
#include "track/track_common.h"
#include "track/track_sid_db.h"
#include "track/track_state.h"
#include "track/track_timer.h"

#define PROCESS_PERIOD_MS (500)

#define NAP_IRQ_THREAD_STACK (32 * 1024)
#define NAP_IRQ_THREAD_PRIORITY (HIGHPRIO - 2)

#define NAP_IRQS_EXT_EVENT_MASK \
  (NAP_IRQS_EXT_EVENT0_Msk | NAP_IRQS_EXT_EVENT1_Msk | NAP_IRQS_EXT_EVENT2_Msk)

static void nap_isr(void *context);

static BSEMAPHORE_DECL(nap_irq_sem, TRUE);

static THD_WORKING_AREA(wa_nap_irq, NAP_IRQ_THREAD_STACK);

static void nap_irq_thread(void *arg);

u8 nap_dna[NAP_DNA_LENGTH] = {0};
u8 nap_track_n_channels = 0;

void nap_setup(void) {
  nap_track_n_channels = MIN(NAP_NUM_TRACKING_CHANNELS, MAX_CHANNELS);

  axi_dma_init();
  axi_dma_start(&AXIDMADriver1);

  /* Enable NAP interrupt */
  chThdCreateStatic(wa_nap_irq,
                    sizeof(wa_nap_irq),
                    NAP_IRQ_THREAD_PRIORITY,
                    nap_irq_thread,
                    NULL);
  gic_handler_register(IRQ_ID_NAP, nap_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_NAP, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_NAP, NAP_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_NAP);
}

u64 nap_timing_count(void) {
  static MUTEX_DECL(timing_count_mutex);
  static volatile u32 rollover_count = 0;
  static volatile u32 prev_count = 0;

  chMtxLock(&timing_count_mutex);

  u32 count = NAP->TIMING_COUNT;

  if (count < prev_count) {
    log_info("NAP rollover: new %" PRIu32 " old: %" PRIu32, count, prev_count);
    rollover_count++;
  }

  prev_count = count;

  u64 total_count = (u64)count | ((u64)rollover_count << 32);

  chMtxUnlock(&timing_count_mutex);
  return total_count;
}

/**
 * Utility to compute NAP time from a sample counter
 *
 * The function assumes the time after reading sample counter is short. The
 * result is formed as a high half of the NAP counter combined with a sample
 * counter. If sample counter is greater, than the low half of NAP counter, the
 * result is adjusted.
 *
 * \param[in] sample_count Readings of sample counter register
 *
 * \return Resulting time in ticks.
 */
u64 nap_sample_time_to_count(u32 sample_count) {
  /* Converts sample time into NAP time using NAP rollover value */
  u64 time_now = nap_timing_count();
  u32 time_high = (u32)(time_now >> 32);
  u32 time_low = (u32)time_now;
  if (sample_count > time_low) {
    /* NAP counter rollover has occurred before sample_count is read. */
    time_high--;
  }
  return ((u64)time_high << 32) | sample_count;
}

/**
 * Convert time in samples into time in milliseconds.
 *
 * \param[in] delta_time Time interval to convert [samples]
 *
 * \return Time interval in milliseconds.
 */
double nap_count_to_ms(u64 delta_time) {
  double time_delta =
      (double)delta_time * (1000. / NAP_FRONTEND_SAMPLE_RATE_Hz);
  return time_delta;
}

/**
 * Convert time in samples into time in nanoseconds.
 *
 * \param[in] delta_time Time interval to convert [samples]
 *
 * \return Time interval in nanoseconds.
 */
double nap_count_to_ns(u64 delta_time) {
  double time_delta =
      (double)delta_time * (SECS_NS / NAP_FRONTEND_SAMPLE_RATE_Hz);
  return time_delta;
}

static void nap_isr(void *context) {
  (void)context;
  chSysLockFromISR();

  /* Wake up processing thread */
  chBSemSignalI(&nap_irq_sem);

  chSysUnlockFromISR();
}

static void handle_nap_irq(void) {
  u32 irq = NAP->IRQS;

  while (irq) {
    if (irq & NAP_IRQS_EXT_EVENT_MASK) {
      ext_event_service(irq & NAP_IRQS_EXT_EVENT_MASK);
    }

    NAP->IRQS = irq;

    asm("dsb");
    irq = NAP->IRQS;
  }

  u32 err = NAP->IRQ_ERRORS;
  if (err) {
    NAP->IRQ_ERRORS = err;
    log_warn("Too many NAP interrupts: 0x%08X", (unsigned int)err);
  }
}

void handle_nap_track_irq(void) {
  u32 irq0 = NAP->TRK_IRQS[0];
  trackers_update(irq0, 0);
  NAP->TRK_IRQS[0] = irq0;

  u32 irq1 = NAP->TRK_IRQS[1];
  trackers_update(irq1, 32);
  NAP->TRK_IRQS[1] = irq1;

  u32 irq2 = NAP->TRK_IRQS[2];
  trackers_update(irq2, 64);
  NAP->TRK_IRQS[2] = irq2;

  asm("dsb");

  u32 err[3];
  memcpy(err, (u8 *)NAP->TRK_IRQ_ERRORS, sizeof(err));
  if (err[0] || err[1] || err[2]) {
    memcpy((u8 *)NAP->TRK_IRQ_ERRORS, err, sizeof(err));
    log_warn("Too many NAP tracking interrupts: 0x%08" PRIX32 "%08" PRIX32
             "%08" PRIX32,
             err[2],
             err[1],
             err[0]);
    trackers_missed(err[0], 0);
    trackers_missed(err[1], 32);
    trackers_missed(err[2], 64);
  }

  DO_EVERY(4096, watchdog_notify(WD_NOTIFY_NAP_ISR));
}

static void __attribute__((noreturn)) nap_irq_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("NAP");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWaitTimeout(&nap_irq_sem, MS2ST(PROCESS_PERIOD_MS));

    handle_nap_irq();
  }
}

void __attribute__((noreturn)) nap_track_irq_thread(void *arg) {
  piksi_systime_t sys_time;
  (void)arg;
  chRegSetThreadName("NAP Tracking");

  while (TRUE) {
    piksi_systime_get(&sys_time);
    u64 ms = piksi_systime_to_ms(&sys_time);
    tracker_time_set(ms);

    handle_nap_track_irq();

    DO_EACH_MS(60 * SECS_MS, check_clear_unhealthy(););

    DO_EACH_MS(PROCESS_PERIOD_MS, tracking_send_state();
               stale_trackers_cleanup(););

    /* Sleep until 250 microseconds is full. */
    piksi_systime_sleep_until_windowed_us(&sys_time, 250);
  }
}

static void nap_rd_dna_callback(u16 sender_id,
                                u8 len,
                                u8 msg[],
                                void *context) {
  (void)sender_id;
  (void)len;
  (void)msg;
  (void)context;
  sbp_send_msg(SBP_MSG_NAP_DEVICE_DNA_RESP, NAP_DNA_LENGTH, nap_dna);
}

void nap_dna_callback_register(void) {
  nap_rd_dna(nap_dna);

  static sbp_msg_callbacks_node_t nap_dna_node;

  sbp_register_cbk(
      SBP_MSG_NAP_DEVICE_DNA_REQ, &nap_rd_dna_callback, &nap_dna_node);
}

void nap_pps(u32 count) {
  NAP->PPS_TIMING_COMPARE = count + NAP_PPS_TIMING_COUNT_OFFSET;
}

void nap_pps_config(u32 microseconds, u8 active) {
  u32 width =
      ceil((double)microseconds / ((1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz) * 1e6)) -
      1;
  NAP->PPS_CONTROL =
      (width << NAP_PPS_CONTROL_PULSE_WIDTH_Pos) | (active & 0x01);
}

bool nap_pps_armed(void) {
  return GET_NAP_STATUS_PPS_TIMING_ARMED(NAP->STATUS);
}

u32 nap_get_ext_event(u8 pin, ext_event_trigger_t *trig) {
  switch (pin) {
    case 0:
      *trig = GET_NAP_STATUS_EXT_EVENT_EDGE0(NAP->STATUS);
      return NAP->EVENT_TIMING_SNAPSHOT[0] + NAP_EXT_TIMING_COUNT_OFFSET;

    case 1:
      *trig = GET_NAP_STATUS_EXT_EVENT_EDGE1(NAP->STATUS);
      return NAP->EVENT_TIMING_SNAPSHOT[1] + NAP_EXT_TIMING_COUNT_OFFSET;

    case 2:
      *trig = GET_NAP_STATUS_EXT_EVENT_EDGE2(NAP->STATUS);
      return NAP->EVENT_TIMING_SNAPSHOT[2] + NAP_EXT_TIMING_COUNT_OFFSET;

    default:
      return 0;
  }
}

void nap_set_ext_event(u8 pin, ext_event_trigger_t trig, u32 timeout) {
  u32 gap = ceil((double)timeout / ((1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz) * 1e6));

  switch (pin) {
    case 0:
      if (timeout > 0) {
        NAP->EVENT_TIMEOUT[0] = gap;
        u32 ctrl = NAP->CONTROL;
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE0(ctrl, trig) |
                       SET_NAP_CONTROL_EXT_EVENT_TIMEOUT0(ctrl, 1);
      } else {
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE0(NAP->CONTROL, trig);
      }
      return;

    case 1:
      if (timeout > 0) {
        NAP->EVENT_TIMEOUT[1] = gap;
        u32 ctrl = NAP->CONTROL;
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE1(ctrl, trig) |
                       SET_NAP_CONTROL_EXT_EVENT_TIMEOUT1(ctrl, 1);
      } else {
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE1(NAP->CONTROL, trig);
      }
      return;

    case 2:
      if (timeout > 0) {
        NAP->EVENT_TIMEOUT[2] = gap;
        u32 ctrl = NAP->CONTROL;
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE2(ctrl, trig) |
                       SET_NAP_CONTROL_EXT_EVENT_TIMEOUT2(ctrl, 1);
      } else {
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE2(NAP->CONTROL, trig);
      }
      return;

    default:
      return;
  }
}

void nap_set_can_termination() {
  u32 ctrl = NAP->CONTROL;
  NAP->CONTROL = SET_NAP_CONTROL_CAN_TERM_ENABLE(ctrl, 1);
}

void nap_unset_can_termination() {
  u32 ctrl = NAP->CONTROL;
  NAP->CONTROL = SET_NAP_CONTROL_CAN_TERM_ENABLE(ctrl, 0);
}

void nap_set_hardware_is_l5() {
  u32 ctrl = NAP->CONTROL;
  NAP->CONTROL = SET_NAP_CONTROL_HARDWARE_IS_BASE(ctrl, 1);
}
