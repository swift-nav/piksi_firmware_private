/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michael Wurm <mwurm@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_dma.h"
#include <ch.h>
#include <hal.h>
#include <stdint.h>
#include <libswiftnav/logging.h>

#define TRACK_DMA_THREAD_PRIO (NORMALPRIO)
#define TRACK_DMA_THREAD_STACK 500

/** Working area for the track_dma thread. */
static THD_WORKING_AREA(wa_track_dma_thread, TRACK_DMA_THREAD_STACK);

static void track_dma_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("track DMA");

  u32 count = 0;
  while(TRUE) {
  	log_info("track DMA thread test output %0d --------------", count++);
  	chThdSleepMilliseconds(2000);
  }
}

void track_dma_setup(void) {
  	log_info("track DMA starting ----------------");

	chThdCreateStatic(wa_track_dma_thread,
					  sizeof(wa_track_dma_thread),
					  TRACK_DMA_THREAD_PRIO,
					  track_dma_thread,
					  NULL);

  	log_info("track DMA started ------------------");
}
