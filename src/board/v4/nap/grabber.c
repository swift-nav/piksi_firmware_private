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
#include <assert.h>
#include <ch.h>

#include <swiftnav/logging.h>

#include "axi_dma.h"
#include "nap_constants.h"
#include "nap_hw.h"

#include "nap/grabber.h"
#include "nap/nap_common.h"

#define DATA_MEMORY_BARRIER() asm volatile("dmb" : : : "memory")

#define GRABBER_TIMEOUT_ms (100)
#define TIMING_COMPARE_DELTA (NAP_FRONTEND_SAMPLE_RATE_Hz * 1e-3) /* 1ms */

static BSEMAPHORE_DECL(axi_dma_rx_bsem, 0);

u8 pRawGrabberBuffer[FIXED_GRABBER_LENGTH] __attribute__((aligned(32)));

static bool raw_samples(u8 *out, u32 len_bytes, u64 *sample_count);

/** Grab raw samples from NAP.
 *
 * \param[out]  length   Number of samples in the buffer.
 * \param[out]  p_count  Output sample count of the first sample used.
 *
 * \return Pointer to the beginning of the sample buffer.
 */
u8 *grab_samples(u32 *length, u64 *p_count) {
  *length = FIXED_GRABBER_LENGTH;
  if (raw_samples(pRawGrabberBuffer, FIXED_GRABBER_LENGTH, p_count)) {
    return pRawGrabberBuffer;
  }
  return NULL;
}

/** Exposes raw samples buffer pointer (to spectrum analyzer as of now)
 *
 * \param[out]  length  Number of samples in the buffer.
 *
 * \return Pointer to the beginning of the sample buffer.
 */
u8 *GrabberGetBufferPt(u32 *length) {
  *length = FIXED_GRABBER_LENGTH;
  return (u8 *)pRawGrabberBuffer;
}

/** Callback for AXI DMA TX.
 */
static void axi_dma_tx_callback(bool success) {
  (void)success;
  assert(success);
}

/** Callback for AXI DMA RX.
 */
static void axi_dma_rx_callback(bool success) {
  /* Signal RX semaphore */
  chSysLockFromISR();
  if (success) {
    chBSemSignalI(&axi_dma_rx_bsem);
  } else {
    chBSemResetI(&axi_dma_rx_bsem, 0);
  }
  chSysUnlockFromISR();
}

/** Set the ACQ control register for raw samples input and start stream.
 *
 * \param  len_words  Number of words.
 */
static void samples_start(u32 len_words) {
  NAP->RAW_CONTROL = (1 << NAP_RAW_CONTROL_RUN_Pos) |
                     ((len_words - 1) << NAP_RAW_CONTROL_LENGTH_Pos);
}

/** Start a DMA transfer.
 *
 * \param in              Input buffer, NULL if not used.
 * \param out             Output buffer.
 * \param len_bytes       Length of transfer (bytes).
 */
static void dma_start(const u8 *in, u8 *out, u32 len_bytes) {
  assert(!((u32)in & (GRABBER_BUFFER_ALIGN - 1)));
  assert(!((u32)out & (GRABBER_BUFFER_ALIGN - 1)));
  assert(!((u32)len_bytes & (GRABBER_LENGTH_ALIGN - 1)));

  if (in != NULL) {
    /* Ensure that input accesses have completed */
    DATA_MEMORY_BARRIER();

    /* Start DMA transfer from in buffer to FPGA */
    axi_dma_write_begin(&AXIDMADriver1, in, len_bytes, axi_dma_tx_callback);
  }

  /* Make sure semaphore is in the TAKEN state. */
  chBSemWaitTimeout(&axi_dma_rx_bsem, TIME_IMMEDIATE);

  /* Start DMA transfer from FPGA to out buffer */
  axi_dma_read_begin(&AXIDMADriver1, out, len_bytes, axi_dma_rx_callback);
}

/** Wait for an FFT result DMA transfer to complete.
 *
 * \return True if a transfer was completed in time, false otherwise.
 */
static bool dma_wait(void) {
  /* Wait for RX semaphore */
  if (chBSemWaitTimeout(&axi_dma_rx_bsem, MS2ST(GRABBER_TIMEOUT_ms)) !=
      MSG_OK) {
    return false;
  }

  /* Ensure that output is not accessed before this point */
  DATA_MEMORY_BARRIER();

  return true;
}

/** Retrieve a buffer of raw samples.
 *
 * \param out             Output buffer.
 * \param len_bytes       Number of bytes.
 * \param sample_count    Output sample count of the first sample used.
 *
 * \return True if the samples were successfully retrieved, false otherwise.
 */
static bool raw_samples(u8 *out, u32 len_bytes, u64 *p_count) {
  u32 len_words = len_bytes / sizeof(u64);
  dma_start(0, (u8 *)out, len_bytes);
  samples_start(len_words);
  bool result = dma_wait();
  (*p_count) = nap_sample_time_to_count(NAP->RAW_TIMING_SNAPSHOT);
  return result;
}
