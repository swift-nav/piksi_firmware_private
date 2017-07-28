/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "board.h"
#include "nap/nap_common.h"
#include "nap/track_channel.h"
#include "../../sbp.h"
#include "../../ext_events.h"

#include "nap_hw.h"
#include "nap_constants.h"
#include "axi_dma.h"

#include "main.h"
#include "track.h"
#include "manage.h"
#include "system_monitor.h"

#include <math.h>
#include <string.h>

#define PROCESS_PERIOD_MS (500)

static void nap_isr(void *context);

static BSEMAPHORE_DECL(nap_irq_sem, TRUE);

static WORKING_AREA_CCM(wa_nap_irq, 32000);

static void nap_irq_thread(void *arg);

u8 nap_dna[NAP_DNA_LENGTH] = {0};
u8 nap_track_n_channels = 0;

void nap_setup(void)
{
  nap_track_n_channels = GET_NAP_STATUS_NUM_TRACKING_CH(NAP->STATUS);
  nap_track_n_channels = MIN(nap_track_n_channels, MAX_CHANNELS);

  nap_scan_channels();

  axi_dma_init();
  axi_dma_start(&AXIDMADriver1);

  /* Phase increment initialization for GPS L1C/A processing */
  NAP_FE->RF1A_PINC = NAP_FE_GPS_L1CA_BASEBAND_MIXER_PINC;

  /* Phase increment initialization for Beidou B1 processing */
  NAP_FE->RF1B_PINC = NAP_FE_BDS_B1_BASEBAND_MIXER_PINC;

  /* Phase increment initialization for GLO L1C/A processing */
  NAP_FE->RF2A_PINC = NAP_FE_GLO_L1CA_BASEBAND_MIXER_PINC;

  /* Phase increment initialization for GLO L2C/A processing */
  NAP_FE->RF3A_PINC = NAP_FE_GLO_L2CA_BASEBAND_MIXER_PINC;

  /* Phase increment initialization for GPS L2C processing */
  NAP_FE->RF4A_PINC = NAP_FE_GPS_L2C_BASEBAND_MIXER_PINC;

  /* Phase increment initialization for Beidou B2 processing */
  NAP_FE->RF4B_PINC = NAP_FE_BDS_B2_BASEBAND_MIXER_PINC;

  /* Reset frontend NCOs after number of samples */
  NAP_FE->RF1_NCO_RESET =
      ((NAP_FE_RF1A_NCO_RESET-1) << FE_RF1_NCO_RESET_RF1A_Pos) |
      ((NAP_FE_RF1B_NCO_RESET-1) << FE_RF1_NCO_RESET_RF1B_Pos);

  NAP_FE->RF2_NCO_RESET =
      ((NAP_FE_RF2A_NCO_RESET-1) << FE_RF2_NCO_RESET_RF2A_Pos);

  NAP_FE->RF3_NCO_RESET =
      ((NAP_FE_RF3A_NCO_RESET-1) << FE_RF3_NCO_RESET_RF3A_Pos);

  NAP_FE->RF4_NCO_RESET =
      ((NAP_FE_RF4A_NCO_RESET-1) << FE_RF4_NCO_RESET_RF4A_Pos) |
      ((NAP_FE_RF4B_NCO_RESET-1) << FE_RF4_NCO_RESET_RF4B_Pos);

  /* Enable frontend channels and their respective NCO resets */
  NAP_FE->CONTROL = (1 << FE_CONTROL_ENABLE_RF1A_Pos) |
                    (0 << FE_CONTROL_ENABLE_RF1B_Pos) |
                    (1 << FE_CONTROL_ENABLE_RF2A_Pos) |
                    (1 << FE_CONTROL_ENABLE_RF3A_Pos) |
                    (1 << FE_CONTROL_ENABLE_RF4A_Pos) |
                    (0 << FE_CONTROL_ENABLE_RF4B_Pos) |
                    (1 << FE_CONTROL_RESET_RF1A_NCO_Pos) |
                    (1 << FE_CONTROL_RESET_RF1B_NCO_Pos) |
                    (1 << FE_CONTROL_RESET_RF2A_NCO_Pos) |
                    (1 << FE_CONTROL_RESET_RF3A_NCO_Pos) |
                    (1 << FE_CONTROL_RESET_RF4A_NCO_Pos) |
                    (1 << FE_CONTROL_RESET_RF4B_NCO_Pos);

  /* Enable NAP interrupt */
  chThdCreateStatic(wa_nap_irq, sizeof(wa_nap_irq), HIGHPRIO-2, nap_irq_thread, NULL);
  gic_handler_register(IRQ_ID_NAP, nap_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_NAP, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_NAP, NAP_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_NAP);

}

u64 nap_timing_count(void)
{
  static MUTEX_DECL(timing_count_mutex);
  static u32 rollover_count = 0;
  static u32 prev_count = 0;

  chMtxLock(&timing_count_mutex);

  u32 count = NAP->TIMING_COUNT;

  if (count < prev_count)
    rollover_count++;

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
 * \param[ın] sample_count Readings of sample counter register
 *
 * \return Resulting time in ticks.
 */
u64 nap_sample_time_to_count(u32 sample_count)
{
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
double nap_count_to_ms(u64 delta_time)
{
  double time_delta = (double)delta_time * (1000. / NAP_FRONTEND_SAMPLE_RATE_Hz);
  return time_delta;
}

/**
 * Convert time in samples into time in nanoseconds.
 *
 * \param[in] delta_time Time interval to convert [samples]
 *
 * \return Time interval in nanoseconds.
 */
double nap_count_to_ns(u64 delta_time)
{
  double time_delta = (double)delta_time * (1e9 / NAP_FRONTEND_SAMPLE_RATE_Hz);
  return time_delta;
}

static void nap_isr(void *context)
{
  (void)context;
  chSysLockFromISR();

  /* Wake up processing thread */
  chBSemSignalI(&nap_irq_sem);

  chSysUnlockFromISR();
}

static void handle_nap_irq(void)
{
  u32 irq = NAP->IRQS;

  while (irq) {
    if (irq & NAP_IRQS_EXT_EVENT_Msk) {
      ext_event_service();
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

static void handle_nap_track_irq(void)
{
  u32 irq0 = NAP->TRK_IRQS0;
  u32 irq1 = NAP->TRK_IRQS1;
  u64 irq = ((u64)irq1 << 32) | irq0;

  tracking_channels_update(irq);
  NAP->TRK_IRQS0 = irq0;
  NAP->TRK_IRQS1 = irq1;

  u32 err0 = NAP->TRK_IRQ_ERRORS0;
  u32 err1 = NAP->TRK_IRQ_ERRORS1;
  u64 err = ((u64)err1 << 32) | err0;
  if (err) {
    NAP->TRK_IRQ_ERRORS0 = err0;
    NAP->TRK_IRQ_ERRORS1 = err1;
    log_warn("Too many NAP tracking interrupts: 0x%08X", (unsigned int)err);
    tracking_channels_missed_update_error(err);
  }

  watchdog_notify(WD_NOTIFY_NAP_ISR);
}

static void nap_irq_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("NAP");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWaitTimeout(&nap_irq_sem, MS2ST(PROCESS_PERIOD_MS));

    handle_nap_irq();
  }
}

void nap_track_irq_thread(void *arg)
{
  systime_t sys_time;
  (void)arg;
  chRegSetThreadName("NAP Tracking");

  while (TRUE) {
    sys_time = chVTGetSystemTime();

    handle_nap_track_irq();
    tracking_channels_process();

    sanitize_trackers();

    DO_EACH_TICKS( S2ST(1),
      check_clear_glo_unhealthy();
    );

    DO_EACH_TICKS( S2ST(DAY_SECS),
      check_clear_unhealthy();
    );

    DO_EACH_TICKS( MS2ST(PROCESS_PERIOD_MS),
      tracking_send_state();
      tracking_send_detailed_state();
    );

    DO_EACH_TICKS( S2ST(100),
      log_info("Max configured PLL integration time: %" PRIu16 " ms",
               max_pll_integration_time_ms);
    );

    /* Sleep for 500 microseconds.
     * The ChibiOS function below should be capable of handling short deadline misses.
     */
    chThdSleepUntilWindowed(sys_time, sys_time+US2ST(500));
  }
}

static void nap_rd_dna_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)msg; (void) context;
  sbp_send_msg(SBP_MSG_NAP_DEVICE_DNA_RESP, NAP_DNA_LENGTH, nap_dna);
}

void nap_dna_callback_register(void)
{
  nap_rd_dna(nap_dna);

  static sbp_msg_callbacks_node_t nap_dna_node;

  sbp_register_cbk(SBP_MSG_NAP_DEVICE_DNA_REQ, &nap_rd_dna_callback,
      &nap_dna_node);
}

void nap_pps(u32 count)
{
  NAP->PPS_TIMING_COMPARE = count + NAP_PPS_TIMING_COUNT_OFFSET;
}

void nap_pps_config(u32 microseconds, u8 active)
{
  u32 width = ceil((double)microseconds / ((1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz) * 1e6)) - 1;
  NAP->PPS_CONTROL = (width << NAP_PPS_CONTROL_PULSE_WIDTH_Pos) | (active & 0x01);
}

bool nap_pps_armed(void)
{
  return GET_NAP_STATUS_PPS_TIMING_ARMED(NAP->STATUS);
}

u32 nap_rw_ext_event(u8 *event_pin, ext_event_trigger_t *event_trig,
    ext_event_trigger_t next_trig, u32 timeout)
{
  if (event_pin) {
    *event_pin = 0;
  }

  if (event_trig) {
    *event_trig = GET_NAP_STATUS_EXT_EVENT_EDGE(NAP->STATUS);
  }

  if (timeout > 0) {
    NAP->EVENT_TIMEOUT = ceil((double)timeout /
        ((1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz) * 1e6));

    u32 ctrl = NAP-> CONTROL;
    NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE(ctrl, next_trig) |
                   SET_NAP_CONTROL_EXT_EVENT_TIMEOUT(ctrl, 1);
  } else {
    NAP->CONTROL = SET_NAP_CONTROL_EXT_EVENT_EDGE(NAP->CONTROL, next_trig);
  }

  return NAP->EVENT_TIMING_SNAPSHOT + NAP_EXT_TIMING_COUNT_OFFSET;
}
