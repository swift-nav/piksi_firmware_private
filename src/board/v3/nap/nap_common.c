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

static BSEMAPHORE_DECL(nap_exti_sem, TRUE);
static WORKING_AREA_CCM(wa_nap_exti, 2000);
static void nap_exti_thread(void *arg);

static u8 nap_dna[NAP_DNA_LENGTH] = {0};
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

  /* FE_BB0_PINC initialization for GPS L1C/A processing */
  NAP_FE->BB_PINC[0] = NAP_FE_L1CA_BASEBAND_MIXER_PINC;

  /* FE_BB3_PINC initialization for GPS L2C processing */
  NAP_FE->BB_PINC[3] = NAP_FE_L2C_BASEBAND_MIXER_PINC;

  /* Enable NAP interrupt */
  chThdCreateStatic(wa_nap_exti, sizeof(wa_nap_exti), HIGHPRIO-1, nap_exti_thread, NULL);
  gic_handler_register(IRQ_ID_NAP_TRACK, nap_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_NAP_TRACK, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_NAP_TRACK, NAP_IRQ_PRIORITY);
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

static void nap_isr(void *context)
{
  (void)context;
  chSysLockFromISR();

  /* Wake up processing thread */
  chBSemSignalI(&nap_exti_sem);

  chSysUnlockFromISR();
}

static void handle_nap_exti(void)
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
    log_error("SwiftNAP Error: 0x%08X", (unsigned int)err);
    tracking_channels_missed_update_error(err);
  }

  watchdog_notify(WD_NOTIFY_NAP_ISR);
}

static void nap_exti_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("NAP ISR");

  while (TRUE) {
    /* Waiting for the IRQ to happen.*/
    chBSemWaitTimeout(&nap_exti_sem, MS2ST(PROCESS_PERIOD_ms));

    handle_nap_exti();
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
