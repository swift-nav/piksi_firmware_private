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
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/memcpy_s.h>
#include <starling/starling_platform.h>

#include <assert.h>

/* Warn on lack of input after 10 seconds. */
#define STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC 10

static semaphore_t input_sem;

/******************************************************************************/
static void fill_starling_obs_array_from_navigation_measurements(
    obs_array_t *obs_array,
    const gps_time_t *t,
    u8 n,
    const navigation_measurement_t nm[]) {
  assert(n <= STARLING_MAX_OBS_COUNT);
  obs_array->n = n;
  for (size_t i = 0; i < obs_array->n; ++i) {
    obs_array->observations[i].sid = nm[i].sid;
    obs_array->observations[i].pseudorange = nm[i].raw_pseudorange;
    obs_array->observations[i].carrier_phase = nm[i].raw_carrier_phase;
    obs_array->observations[i].doppler = nm[i].raw_measured_doppler;
    obs_array->observations[i].cn0 = nm[i].cn0;
    obs_array->observations[i].lock_time = nm[i].lock_time;
    obs_array->observations[i].flags = nm[i].flags;

    /* TOT is special. We want to recompute from the observation time and raw
     * pseudorange because the navigation measurement tot will have already had
     * clock corrections applied. */
    obs_array->observations[i].tot = GPS_TIME_UNKNOWN;
    if (t) {
      obs_array->observations[i].tot = *t;
      obs_array->observations[i].tot.tow -=
          obs_array->observations[i].pseudorange / GPS_C;
      normalize_gps_time(&obs_array->observations[i].tot);
    }
  }
}

/******************************************************************************/
void starling_input_bridge_init(void) {
  chSemObjectInit(&input_sem, 0);
  platform_mailbox_init(MB_ID_ME_OBS);
  platform_mailbox_init(MB_ID_BASE_OBS);
  platform_mailbox_init(MB_ID_SBAS_DATA);
  platform_mailbox_init(MB_ID_EPHEMERIS);
}

/******************************************************************************/
int starling_send_rover_obs(const gps_time_t *t,
                            const navigation_measurement_t *nm,
                            size_t n) {
  obs_array_t *obs_array = platform_mailbox_item_alloc(MB_ID_ME_OBS);
  if (NULL == obs_array) {
    log_error("Could not allocate pool for obs!");
    return STARLING_SEND_ERROR;
  }

  obs_array->sender = 0;
  obs_array->t = GPS_TIME_UNKNOWN;
  if (NULL != t) {
    obs_array->t = *t;
  }
  fill_starling_obs_array_from_navigation_measurements(obs_array, t, n, nm);

  errno_t ret = platform_mailbox_post(MB_ID_ME_OBS, obs_array, MB_NONBLOCKING);
  if (ret != 0) {
    /* We could grab another item from the mailbox, discard it and then
     * post our obs again but if the size of the mailbox and the pool
     * are equal then we should have already handled the case where the
     * mailbox is full when we handled the case that the pool was full.
     * */
    log_error("Mailbox should have space for obs!");
    platform_mailbox_item_free(MB_ID_ME_OBS, obs_array);
    return STARLING_SEND_ERROR;
  }

  chSemSignal(&input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_base_obs(const obs_array_t *obs_array) {
  /* Before doing anything, try to get new observation to post to. */
  obs_array_t *new_obs_array = platform_mailbox_item_alloc(MB_ID_BASE_OBS);
  if (new_obs_array == NULL) {
    log_warn("Base obs pool full, discarding base obs at: wn: %d, tow: %.2f",
             obs_array->t.wn,
             obs_array->t.tow);
    return STARLING_SEND_ERROR;
  }

  // TODO(Kevin) remove this copy.
  *new_obs_array = *obs_array;
  /* If we successfully get here without returning early, then go ahead and
   * post into the Starling engine. */
  errno_t post_error =
      platform_mailbox_post(MB_ID_BASE_OBS, new_obs_array, MB_NONBLOCKING);
  if (post_error) {
    log_error("Base obs mailbox should have space!");
    platform_mailbox_item_free(MB_ID_BASE_OBS, new_obs_array);
    return STARLING_SEND_ERROR;
  }

  chSemSignal(&input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_ephemerides(const ephemeris_t *ephemerides, size_t n) {
  ephemeris_array_t *eph_array = platform_mailbox_item_alloc(MB_ID_EPHEMERIS);
  if (NULL == eph_array) {
    /* If we can't get allocate an item, fetch the oldest one and use that
     * instead. */
    int error = platform_mailbox_fetch(
        MB_ID_EPHEMERIS, (void **)&eph_array, MB_NONBLOCKING);
    if (error) {
      log_error(
          "Unable to allocate ephemeris array, and mailbox is empty.");
      if (eph_array) {
        platform_mailbox_item_free(MB_ID_EPHEMERIS, eph_array);
      }
      return STARLING_SEND_ERROR;
    }
  }

  assert(NULL != eph_array);
  /* Copy in all of the information. */
  eph_array->n = n;
  if (n > 0) {
    MEMCPY_S(eph_array->ephemerides,
             sizeof(eph_array->ephemerides),
             ephemerides,
             n * sizeof(ephemeris_t));
  }
  /* Try to post to Starling. */
  int error = platform_mailbox_post(MB_ID_EPHEMERIS, eph_array, MB_BLOCKING);
  if (error) {
    platform_mailbox_item_free(MB_ID_EPHEMERIS, eph_array);
    return STARLING_SEND_ERROR;
  }

  chSemSignal(&input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_sbas_data(const sbas_raw_data_t *sbas_data) {
  sbas_raw_data_t *sbas_data_msg = platform_mailbox_item_alloc(MB_ID_SBAS_DATA);
  if (NULL == sbas_data_msg) {
    log_error("platform_mailbox_item_alloc(MB_ID_SBAS_DATA) failed!");
    return STARLING_SEND_ERROR;
  }
  assert(sbas_data);
  *sbas_data_msg = *sbas_data;
  errno_t ret = platform_mailbox_post(MB_ID_SBAS_DATA, sbas_data_msg, MB_NONBLOCKING);
  if (ret != 0) {
    log_error("platform_mailbox_post(MB_ID_SBAS_DATA) failed!");
    platform_mailbox_item_free(MB_ID_SBAS_DATA, sbas_data_msg);
    return STARLING_SEND_ERROR;
  }
  chSemSignal(&input_sem);
  return STARLING_SEND_OK;
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
int starling_read_rover_obs(int blocking, obs_array_t *obs_array) {
  (void)blocking;
  (void)obs_array;
  return 0;
}

/******************************************************************************/
int starling_read_base_obs(int blocking, obs_array_t *obs_array) {
  (void)blocking;
  (void)obs_array;
  return 0;
}

/******************************************************************************/
int starling_read_ephemeris_array(int blocking, ephemeris_array_t *eph_array) {
  (void)blocking;
  (void)eph_array;
  return 0;
}

/******************************************************************************/
int starling_read_sbas_data(int blocking, sbas_raw_data_t *sbas_data) {
  (void)blocking;
  (void)sbas_data;
  return 0;
}


