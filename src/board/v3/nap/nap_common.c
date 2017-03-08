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
#include "../../sbp.h"
#include "../../ext_events.h"

#include "nap_hw.h"
#include "nap_fe_hw.h"
#include "nap_constants.h"
#include "axi_dma.h"

#include "track.h"
#include "system_monitor.h"

#include <math.h>
#include <string.h>

#define PROCESS_PERIOD_ms (1000)

static void nap_isr(void *context);
static void nap_track_isr(void *context);

static BSEMAPHORE_DECL(nap_irq_sem, TRUE);
static BSEMAPHORE_DECL(nap_track_irq_sem, TRUE);

static WORKING_AREA_CCM(wa_nap_irq, 32000);
static WORKING_AREA_CCM(wa_nap_track_irq, 32000);

static void nap_irq_thread(void *arg);
static void nap_track_irq_thread(void *arg);

u8 nap_dna[NAP_DNA_LENGTH] = {0};
u8 nap_track_n_channels = 0;

void nap_setup(void)
{
  nap_track_n_channels = (NAP->STATUS & NAP_STATUS_TRACKING_CH_Msk) >>
                          NAP_STATUS_TRACKING_CH_Pos;
  nap_track_n_channels = MIN(nap_track_n_channels, NAP_MAX_N_TRACK_CHANNELS);

  /* Configure ACP AXI attributes to match page table. */
  NAP->ACQ_AXI_ATTRIBUTES = ( 0b1111 << NAP_ACQ_AXI_ATTRIBUTES_ARCACHE_Pos) |
                            ( 0b1111 << NAP_ACQ_AXI_ATTRIBUTES_AWCACHE_Pos) |
                            (0b11111 << NAP_ACQ_AXI_ATTRIBUTES_ARUSER_Pos)  |
                            (0b11111 << NAP_ACQ_AXI_ATTRIBUTES_AWUSER_Pos);

  axi_dma_init();
  axi_dma_start(&AXIDMADriver1);

  /* Set acquisiton decimation factor */
  NAP_FE->ACQ_CONTROL = (NAP_ACQ_DECIMATION_RATE << NAP_FE_ACQ_CONTROL_DECIMATION_Pos);

  /* FE_PINC0 initialization for GPS L1C/A processing */
  NAP_FE->PINC[0] = NAP_FE_L1CA_BASEBAND_MIXER_PINC;

  /* FE_PINC3 initialization for GPS L2C processing */
  NAP_FE->PINC[3] = NAP_FE_L2C_BASEBAND_MIXER_PINC;

  /* Enable frontend channel 0 (RF1) and frontend channel 3 (RF4) */
  NAP_FE->CONTROL = (1 << NAP_FE_CONTROL_ENABLE_RF1_Pos) |
                    (1 << NAP_FE_CONTROL_ENABLE_RF4_Pos);

  /* Enable NAP interrupt */
  chThdCreateStatic(wa_nap_irq, sizeof(wa_nap_irq), HIGHPRIO-2, nap_irq_thread, NULL);
  gic_handler_register(IRQ_ID_NAP, nap_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_NAP, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_NAP, NAP_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_NAP);

  /* Enable NAP tracking interrupt */
  chThdCreateStatic(wa_nap_track_irq, sizeof(wa_nap_track_irq), HIGHPRIO-1, nap_track_irq_thread, NULL);
  gic_handler_register(IRQ_ID_NAP_TRACK, nap_track_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_NAP_TRACK, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_NAP_TRACK, NAP_TRACK_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_NAP_TRACK);
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

static void nap_track_isr(void *context)
{
  (void)context;
  chSysLockFromISR();

  /* Wake up processing thread */
  chBSemSignalI(&nap_track_irq_sem);

  chSysUnlockFromISR();
}

static void handle_nap_irq(void)
{
  u32 irq = NAP->IRQ;

  while (irq) {
    if (irq & NAP_IRQ_EXT_EVENT_Msk) {
      ext_event_service();
    }

    NAP->IRQ = irq;

    asm("dsb");
    irq = NAP->IRQ;
  }

  u32 err = NAP->IRQ_ERROR;
  if (err) {
    NAP->IRQ_ERROR = err;
    if (err & NAP_IRQ_EXT_EVENT_Msk) {
      log_error("NAP Error: Too many external events.");
    } else {
      log_error("NAP Interrupt Error: 0x%08X", (unsigned int)err);
    }
  }
}

static void handle_nap_track_irq(void)
{
  u32 irq = NAP->TRK_IRQ;

  while (irq) {
    tracking_channels_update(irq);
    NAP->TRK_IRQ = irq;

    asm("dsb");
    irq = NAP->TRK_IRQ;
  }

  u32 err = NAP->TRK_IRQ_ERROR;
  if (err) {
    NAP->TRK_IRQ_ERROR = err;
    log_error("NAP Tracking Interrupt Error: 0x%08X", (unsigned int)err);
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
    chBSemWaitTimeout(&nap_irq_sem, MS2ST(PROCESS_PERIOD_ms));

    handle_nap_irq();
  }
}

static void nap_track_irq_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("NAP Tracking");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWaitTimeout(&nap_track_irq_sem, MS2ST(PROCESS_PERIOD_ms));

    handle_nap_track_irq();
    tracking_channels_process();
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
  return (NAP->STATUS & NAP_STATUS_PPS_ARMED_Msk) >> NAP_STATUS_PPS_ARMED_Pos;
}

u32 nap_rw_ext_event(u8 *event_pin, ext_event_trigger_t *event_trig,
    ext_event_trigger_t next_trig)
{
  if (event_pin) {
    *event_pin = 0;
  }

  if (event_trig) {
    *event_trig = (NAP->CONTROL & NAP_CONTROL_EXT_EVENT_EDGE_Msk) >>
        NAP_CONTROL_EXT_EVENT_EDGE_Pos;
  }

  NAP->CONTROL = (next_trig << NAP_CONTROL_EXT_EVENT_EDGE_Pos);

  return NAP->EVENT_TIMING_SNAPSHOT + NAP_EXT_TIMING_COUNT_OFFSET;
}

