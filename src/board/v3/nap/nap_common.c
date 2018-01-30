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

#include "alias_detector/alias_detector.h"

#include "board.h"
#include "ext_events/ext_events.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "piksi_systime.h"
#include "sbp.h"

#include "axi_dma.h"
#include "nap_constants.h"
#include "nap_hw.h"

#include "main.h"
#include "manage.h"
#include "timing/timing.h"
#include "system_monitor/system_monitor.h"
#include "track/track_state.h"

#include <math.h>
#include <string.h>

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
  static u32 rollover_count = 0;
  static u32 prev_count = 0;

  chMtxLock(&timing_count_mutex);

  u32 count = NAP->TIMING_COUNT;

  if (count < prev_count) rollover_count++;

  prev_count = count;

  u64 total_count = (u64)count | ((u64)rollover_count << 32);

  chMtxUnlock(&timing_count_mutex);
  return total_count;
}

u32 adel_time_us(void) {
  u32 count = NAP->TIMING_COUNT;
  return (u32)(count * (RX_DT_NOMINAL * 1e6));
}

/**
 * Utility to compute NAP time from a sample counter
 *
 * The function assumes the time after reading sample counter is short. The
 * result is formed as a high half of the NAP counter combined with a sample
 * counter. If sample counter is greater, than the low half of NAP counter, the
 * result is adjusted.
 *
 * \param[ın] sample_count Readings of sample counter register
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
  double time_delta = (double)delta_time * (1e9 / NAP_FRONTEND_SAMPLE_RATE_Hz);
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

struct adel_profile_last {
  u64 update_last[8];
  u64 rest_last[8];
};

struct adel_profile {
  u64 allnum[6];
  u64 update[8];
  u64 rest[8];

  struct adel_profile_last last;

  u64 start_us;
};

enum chunk_profile {
  CHUNK_START,
  CHUNK_UPDATE,
  CHUNK_REST
};

static struct adel_profile adel_profile = {0};

void nap_profile(int chunk)
{
  if (CHUNK_START == chunk) {
    adel_profile.start_us = adel_time_us();
    return;
  }
  u64 now_us = adel_time_us();
  u32 elapsed_us = (u32)(now_us - adel_profile.start_us);

  adel_profile.start_us = adel_time_us();

  u64 *hist;
  u64 *last;
  if (CHUNK_UPDATE == chunk) {
    hist = adel_profile.update;
    last = adel_profile.last.update_last;
  } else if (CHUNK_REST == chunk) {
    hist = adel_profile.rest;
    last = adel_profile.last.rest_last;
  } else {
    return;
  }
  if (elapsed_us <= 5) {
    hist[0]++;
    last[0]++;
  } else if (elapsed_us <= 10) {
    hist[1]++;
    last[1]++;
  } else if (elapsed_us <= 15) {
    hist[2]++;
    last[2]++;
  } else if (elapsed_us <= 20) {
    hist[3]++;
    last[3]++;
  } else if (elapsed_us <= 25) {
    hist[4]++;
    last[4]++;
  } else if (elapsed_us <= 30) {
    hist[5]++;
    last[5]++;
  } else {
    hist[6]++;
    last[6]++;
  }
}

static void handle_nap_track_irq(void) {
  u32 irq0 = NAP->TRK_IRQS0;
  u32 irq1 = NAP->TRK_IRQS1;
  u64 irq = ((u64)irq1 << 32) | irq0;

  memset(&adel_profile.last, 0, sizeof(adel_profile.last));

  u32 start_us = adel_time_us();

  trackers_update(irq);

  u64 now_us = adel_time_us();

  u32 elapsed_us = now_us - start_us;

  u64 *allnum = adel_profile.allnum;

  if (elapsed_us <= 100) {
    allnum[0]++;
  } else if (elapsed_us <= 200) {
    allnum[1]++;
  } else if (elapsed_us <= 300) {
    allnum[2]++;
  } else if (elapsed_us <= 400) {
    allnum[3]++;
  } else if (elapsed_us <= 500) {
    allnum[4]++;
  } else if (elapsed_us <= 600) {
    allnum[5]++;
  }
  if (elapsed_us > 450) {
    log_warn("adel hist:"
             " %" PRIu32 " %" PRIu32,
             start_us, elapsed_us);
    log_warn("adel allnum:"
             " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64,
             allnum[0], allnum[1], allnum[2], allnum[3], allnum[4], allnum[5]);

    u64 *update = adel_profile.update;
    log_warn("adel update:"
             " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64
             " %" PRIu64 " %" PRIu64 " %" PRIu64,
             update[0], update[1], update[2], update[3],
             update[4], update[5], update[6]);

    u64 *rest = adel_profile.rest;
    log_warn("adel rest:"
             " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64
             " %" PRIu64 " %" PRIu64 " %" PRIu64,
             rest[0], rest[1], rest[2], rest[3],
             rest[4], rest[5], rest[6]);

    u64 *update_last = adel_profile.last.update_last;
    log_warn("adel update_last:"
             " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64
             " %" PRIu64 " %" PRIu64 " %" PRIu64,
             update_last[0], update_last[1], update_last[2], update_last[3],
             update_last[4], update_last[5], update_last[6]);

    u64 *rest_last = adel_profile.last.rest_last;
    log_warn("adel rest_last:"
             " %" PRIu64 " %" PRIu64 " %" PRIu64 " %" PRIu64
             " %" PRIu64 " %" PRIu64 " %" PRIu64,
             rest_last[0], rest_last[1], rest_last[2], rest_last[3],
             rest_last[4], rest_last[5], rest_last[6]);
  }

  NAP->TRK_IRQS0 = irq0;
  NAP->TRK_IRQS1 = irq1;

  asm("dsb");

  u32 err0 = NAP->TRK_IRQ_ERRORS0;
  u32 err1 = NAP->TRK_IRQ_ERRORS1;
  u64 err = ((u64)err1 << 32) | err0;
  if (err) {
    NAP->TRK_IRQ_ERRORS0 = err0;
    NAP->TRK_IRQ_ERRORS1 = err1;
    log_warn("Too many NAP tracking interrupts: 0x%08" PRIX32 "%08" PRIX32,
             err1,
             err0);
    trackers_missed(err);
  }

  watchdog_notify(WD_NOTIFY_NAP_ISR);
}

static void nap_irq_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("NAP");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWaitTimeout(&nap_irq_sem, MS2ST(PROCESS_PERIOD_MS));

    handle_nap_irq();
  }
}

void nap_track_irq_thread(void *arg) {
  piksi_systime_t sys_time;
  (void)arg;
  chRegSetThreadName("NAP Tracking");

  while (TRUE) {
    piksi_systime_get(&sys_time);

    handle_nap_track_irq();

    sanitize_trackers();

    DO_EACH_MS(1 * SECS_MS, check_clear_glo_unhealthy(););

    DO_EACH_MS(DAY_SECS * SECS_MS, check_clear_unhealthy(););

    DO_EACH_MS(PROCESS_PERIOD_MS, tracking_send_state();
               tracking_send_detailed_state(););

    /* Sleep until 500 microseconds is full. */
    piksi_systime_sleep_until_windowed_us(&sys_time, 500);
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
      return NAP->EVENT0_TIMING_SNAPSHOT + NAP_EXT_TIMING_COUNT_OFFSET;

    case 1:
      *trig = GET_NAP_STATUS_EXT_EVENT_EDGE1(NAP->STATUS);
      return NAP->EVENT1_TIMING_SNAPSHOT + NAP_EXT_TIMING_COUNT_OFFSET;

    case 2:
      *trig = GET_NAP_STATUS_EXT_EVENT_EDGE2(NAP->STATUS);
      return NAP->EVENT2_TIMING_SNAPSHOT + NAP_EXT_TIMING_COUNT_OFFSET;

    default:
      return 0;
  }
}

void nap_set_ext_event(u8 pin, ext_event_trigger_t trig, u32 timeout) {
  u32 gap = ceil((double)timeout / ((1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz) * 1e6));

  switch (pin) {
    case 0:
      if (timeout > 0) {
        NAP->EVENT0_TIMEOUT = gap;
        u32 ctrl = NAP->CONTROL;
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE0(ctrl, trig) |
                       SET_NAP_CONTROL_EXT_EVENT_TIMEOUT0(ctrl, 1);
      } else {
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE0(NAP->CONTROL, trig);
      }
      return;

    case 1:
      if (timeout > 0) {
        NAP->EVENT1_TIMEOUT = gap;
        u32 ctrl = NAP->CONTROL;
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE1(ctrl, trig) |
                       SET_NAP_CONTROL_EXT_EVENT_TIMEOUT1(ctrl, 1);
      } else {
        NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE1(NAP->CONTROL, trig);
      }
      return;

    case 2:
      if (timeout > 0) {
        NAP->EVENT2_TIMEOUT = gap;
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
