/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "calc/starling_input_bridge.h"

#include <ch.h>

/* Warn on lack of input after 10 seconds. */
#define STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC 10

static semaphore_t input_sem;

/******************************************************************************/
void starling_input_bridge_init(void) {
  chSemObjectInit(&input_sem, 0);

}

/******************************************************************************/
void starling_send_rover_obs(const obs_array_t *obs_array) {
  (void)obs_array;
  chSemSignal(&input_sem);
}

/******************************************************************************/
void starling_send_base_obs(const obs_array_t *obs_array) {
  (void)obs_array;
  chSemSignal(&input_sem);
}

/******************************************************************************/
void starling_send_ephemerides(const ephemeris_array_t *eph_array) {
  (void)eph_array;
  chSemSignal(&input_sem);
}

/******************************************************************************/
void starling_send_sbas_data(const sbas_raw_data_t *sbas_data,
    const size_t n_sbas_data) {
  (void)sbas_data;
  (void)n_sbas_data;
  chSemSignal(&input_sem);
}

/******************************************************************************/
void starling_wait(void) {
  const systime_t timeout = S2ST(STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC);
  msg_t ret = chSemWaitTimeout(&input_sem, timeout);
  if (ret == MSG_OK) {
    return;
  } else if (ret == MSG_TIMEOUT) {
    log_warn("Starling has not received any input for %d seconds.",
        STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC);
  } else {
    log_error("Starling input semaphore reset unexpectedly.");
  }
}

/******************************************************************************/
int starling_read_sbas_data(/*TODO*/void) {
  return 0;
}

/******************************************************************************/
int starling_read_rover_obs(/*TODO*/void) {
  return 0;
}

int starling_read_base_obs(/*TODO*/void) {
  return 0;
}

int starling_read_ephemerides(/*TODO*/void) {
  return 0;
}



