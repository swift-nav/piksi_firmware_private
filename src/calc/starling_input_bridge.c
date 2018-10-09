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

#include "calc/starling_platform_extra.h"

#include <starling/starling_platform.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/memcpy_s.h>

#include <assert.h>
#include <stdbool.h>

/* Warn on lack of input after 10 seconds. */
#define STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC 10

/* Convenience macro used to short-circuit out of functions
 * when Starling is disabled. */
#define RETURN_IF_STARLING_BYPASS_ENABLED(code) \
  {                                             \
    if (is_bypass_enabled) return code;         \
  }

static bool is_bypass_enabled = true;

static platform_sem_t *input_sem = NULL;

/******************************************************************************/
static void meas_to_obs(obs_array_t *obs_array,
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
  platform_mailbox_init(MB_ID_ME_OBS);
  platform_mailbox_init(MB_ID_BASE_OBS);
  platform_mailbox_init(MB_ID_SBAS_DATA);
  platform_mailbox_init(MB_ID_EPHEMERIS);
  platform_mailbox_init(MB_ID_IMU);

  input_sem = platform_sem_create();
  assert(NULL != input_sem);
  starling_input_bridge_set_mode(STARLING_BRIDGE_MODE_DEFAULT);
}

/******************************************************************************/
void starling_input_bridge_set_mode(int mode) {
  switch(mode) {
    case STARLING_BRIDGE_MODE_BYPASS: 
      is_bypass_enabled = true; 
      break;
    case STARLING_BRIDGE_MODE_DEFAULT: 
    default:
      is_bypass_enabled = false; 
      break;
  }
}

/******************************************************************************/
int starling_send_rover_obs(const gps_time_t *t,
                            const navigation_measurement_t *nm,
                            size_t n) {
  RETURN_IF_STARLING_BYPASS_ENABLED(STARLING_SEND_OK);

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

  if (n > STARLING_MAX_OBS_COUNT) {
    log_warn("Trying to send %u/%u observations, extra will be discarded.",
             n,
             STARLING_MAX_OBS_COUNT);
    n = STARLING_MAX_OBS_COUNT;
  }
  meas_to_obs(obs_array, t, n, nm);

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

  platform_sem_signal(input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_base_obs(const obs_array_t *obs_array) {
  RETURN_IF_STARLING_BYPASS_ENABLED(STARLING_SEND_OK);

  /* Before doing anything, try to get new observation to post to. */
  obs_array_t *new_obs_array = platform_mailbox_item_alloc(MB_ID_BASE_OBS);
  if (NULL == new_obs_array) {
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

  platform_sem_signal(input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_ephemerides(const ephemeris_t *ephemerides, size_t n) {
  RETURN_IF_STARLING_BYPASS_ENABLED(STARLING_SEND_OK);

  ephemeris_array_t *eph_array = platform_mailbox_item_alloc(MB_ID_EPHEMERIS);
  if (NULL == eph_array) {
    /* If we can't get allocate an item, fetch the oldest one and use that
     * instead. */
    int error = platform_mailbox_fetch(
        MB_ID_EPHEMERIS, (void **)&eph_array, MB_NONBLOCKING);
    if (error) {
      log_error("Unable to allocate ephemeris array, and mailbox is empty.");
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

  platform_sem_signal(input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_sbas_data(const sbas_raw_data_t *sbas_data) {
  RETURN_IF_STARLING_BYPASS_ENABLED(STARLING_SEND_OK);

  sbas_raw_data_t *sbas_data_msg = platform_mailbox_item_alloc(MB_ID_SBAS_DATA);
  if (NULL == sbas_data_msg) {
    log_error("platform_mailbox_item_alloc(MB_ID_SBAS_DATA) failed!");
    return STARLING_SEND_ERROR;
  }
  assert(sbas_data);
  *sbas_data_msg = *sbas_data;
  errno_t ret =
      platform_mailbox_post(MB_ID_SBAS_DATA, sbas_data_msg, MB_NONBLOCKING);
  if (0 != ret) {
    log_error("platform_mailbox_post(MB_ID_SBAS_DATA) failed!");
    platform_mailbox_item_free(MB_ID_SBAS_DATA, sbas_data_msg);
    return STARLING_SEND_ERROR;
  }
  platform_sem_signal(input_sem);
  return STARLING_SEND_OK;
}

/******************************************************************************/
int starling_send_imu_data(const imu_data_t *imu_data) {
  RETURN_IF_STARLING_BYPASS_ENABLED(STARLING_SEND_OK);

  imu_data_t *imu_msg = platform_mailbox_item_alloc(MB_ID_IMU);
  /* For IMU data we simply want it to behave like a FIFO implemented as
   * circular buffer. We overwrite the oldest message if it is full. */
  if (NULL == imu_msg) {
    int ret =
        platform_mailbox_fetch(MB_ID_IMU, (void **)&imu_msg, MB_NONBLOCKING);
    if (0 != ret || NULL == imu_msg) {
      log_error("platform_mailbox_item_alloc(MB_ID_IMU) failed!");
      return STARLING_SEND_ERROR;
    }
  }
  assert(imu_data);
  *imu_msg = *imu_data;
  errno_t ret = platform_mailbox_post(MB_ID_IMU, imu_msg, MB_NONBLOCKING);
  if (0 != ret) {
    log_error("platform_mailbox_post(MB_ID_IMU) failed!");
    platform_mailbox_item_free(MB_ID_IMU, imu_msg);
    return STARLING_SEND_ERROR;
  }
  platform_sem_signal(input_sem);
  return STARLING_SEND_OK;
}

/**
 * A note about the use of semaphores in the "receive" implementation.
 *
 * It is a guarantee by the Starling interface that all IO functions are
 * called from a single thread. This means that all semaphore decrement
 * operations will never race with one another.
 *
 * However, we are going to ignore this altogether. Since Starling is
 * robust in the face of a spuriously returning "wait" function, we don't
 * bother trying to decrement the semaphore upon each read. Instead we
 * just accept the fact that we will allow Starling to "wake" exactly once for
 * every input that is posted. In the case where multiple inputs are processed
 * during a single "wake" period, the remaining count in the semaphore will
 * be expended as spurious wakeups.
 */

/******************************************************************************/
void starling_wait(void) {
  const unsigned long millis = 1000 * STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC;
  /* If Starling engine is disabled, repeatedly wait on the input semaphore
   * with no action when it times out. When Starling is enabled, move on to
   * check result of waiting on the semaphore. */
  int ret = PLATFORM_SEM_OK;
  do {
    ret = platform_sem_wait_timeout(input_sem, millis);
  } while (is_bypass_enabled);

  if (PLATFORM_SEM_OK == ret) {
    return;
  } else if (PLATFORM_SEM_TIMEOUT == ret) {
    log_warn("Starling has not received any input for over %d seconds.",
             STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC);
  } else {
    log_error("Starling input semaphore reset unexpectedly.");
  }
}

/******************************************************************************/
int starling_receive_rover_obs(int blocking, obs_array_t *obs_array) {
  obs_array_t *new_obs_array = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_ME_OBS, (void **)&new_obs_array, blocking);
  if (new_obs_array) {
    *obs_array = *new_obs_array;
    platform_mailbox_item_free(MB_ID_ME_OBS, new_obs_array);
  }
  return ret;
}

/******************************************************************************/
int starling_receive_base_obs(int blocking, obs_array_t *obs_array) {
  obs_array_t *new_obs_array = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_BASE_OBS, (void **)&new_obs_array, blocking);
  if (new_obs_array) {
    if (STARLING_READ_OK == ret) {
      *obs_array = *new_obs_array;
    } else {
      /* Erroneous behavior for fetch to return non-NULL pointer and indicate
       * read failure. */
      log_error("Base obs mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_BASE_OBS, new_obs_array);
  }
  return ret;
}

/******************************************************************************/
int starling_receive_ephemeris_array(int blocking,
                                     ephemeris_array_t *eph_array) {
  ephemeris_array_t *local_eph_arr = NULL;
  errno_t ret = platform_mailbox_fetch(
      MB_ID_EPHEMERIS, (void **)&local_eph_arr, blocking);
  if (local_eph_arr) {
    if (STARLING_READ_OK == ret) {
      eph_array->n = local_eph_arr->n;
      if (local_eph_arr->n > 0) {
        MEMCPY_S(eph_array->ephemerides,
                 sizeof(eph_array->ephemerides),
                 local_eph_arr->ephemerides,
                 local_eph_arr->n * sizeof(ephemeris_t));
      }
    } else {
      log_error("STARLING: ephemeris mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_EPHEMERIS, local_eph_arr);
  }
  return ret;
}

/******************************************************************************/
int starling_receive_sbas_data(int blocking, sbas_raw_data_t *sbas_data) {
  sbas_raw_data_t *local_data = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_SBAS_DATA, (void **)&local_data, blocking);
  if (local_data) {
    if (STARLING_READ_OK == ret) {
      *sbas_data = *local_data;
    } else {
      /* Erroneous behavior for fetch to return non-NULL pointer and indicate
       * read failure. */
      log_error("STARLING: sbas mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_SBAS_DATA, local_data);
  }
  return ret;
}

/******************************************************************************/
int starling_receive_imu_data(int blocking, imu_data_t *imu_data) {
  imu_data_t *local_data = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_IMU, (void **)&local_data, blocking);
  if (local_data) {
    if (STARLING_READ_OK == ret) {
      *imu_data = *local_data;
    } else {
      /* Erroneous behavior for fetch to return non-NULL pointer and indicate
       * read failure. */
      log_error("STARLING: imu mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_IMU, local_data);
  }
  return ret;
}
