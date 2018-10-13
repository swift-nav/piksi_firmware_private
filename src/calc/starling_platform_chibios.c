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
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libsbp/sbp.h>
#include <starling/observation.h>
#include <starling/platform/starling_platform.h>
#include <starling/pvt_engine/firmware_binding.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/sid_set.h>
#include <swiftnav/single_epoch_solver.h>
#include <swiftnav/troposphere.h>

#include "calc_base_obs.h"
#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "main.h"
#include "manage.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "piksi_systime.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define MAILBOX_BLOCKING_TIMEOUT_MS 5000

#define SBAS_DATA_N_BUFF 6

#define EPHEMERIS_N_BUFF 10

#define IMU_N_BUFF 10

#define NUM_MUTEXES STARLING_MAX_NUM_MUTEXES

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/* Time-matched observations data-structures. */
static msg_t paired_obs_mailbox_buff[PAIRED_OBS_N_BUFF];
static paired_obss_t paired_obs_buff[PAIRED_OBS_N_BUFF] _CCM;

/* Keep a mailbox of received base obs so we can process all of them in
 * order even if we have a bursty base station connection. */
static msg_t base_obs_mailbox_buff[BASE_OBS_N_BUFF];
static obs_array_t base_obs_buff[BASE_OBS_N_BUFF] _CCM;

/* ME Data API data-structures. */
static msg_t me_obs_mailbox_buff[ME_OBS_MSG_N_BUFF];
static obs_array_t me_obs_buff[ME_OBS_MSG_N_BUFF];

/* SBAS Data API data-structures. */
static msg_t sbas_data_mailbox_buff[SBAS_DATA_N_BUFF];
static sbas_raw_data_t sbas_data_buff[SBAS_DATA_N_BUFF];

/* Ephemeris Data API data-structures. */
static msg_t ephemeris_mailbox_buff[EPHEMERIS_N_BUFF];
static ephemeris_array_t ephemeris_buff[EPHEMERIS_N_BUFF];

/* Imu Data API data-structures. */
static msg_t imu_mailbox_buff[IMU_N_BUFF];
static imu_data_t imu_buff[IMU_N_BUFF];

static mutex_t mutexes[NUM_MUTEXES];

/*******************************************************************************
 * Platform Shim Calls
 ******************************************************************************/

/* Mutex */
int platform_mutex_init(mtx_id_t id) {
  if (id >= NUM_MUTEXES) {
    return -1;
  }
  chMtxObjectInit(&mutexes[id]);
  return 0;
}

void platform_mutex_lock(mtx_id_t id) { chMtxLock(&mutexes[id]); }

void platform_mutex_unlock(mtx_id_t id) { chMtxUnlock(&mutexes[id]); }

/* Threading */

typedef struct platform_thread_info_s {
  void *wsp;
  size_t size;
  int prio;
} platform_thread_info_t;

static THD_WORKING_AREA(wa_time_matched_obs_thread,
                        TIME_MATCHED_OBS_THREAD_STACK);

/* Working area for the main starling thread. */
static THD_WORKING_AREA(wa_starling_thread, STARLING_THREAD_STACK);

static void platform_thread_info_init(const thread_id_t id,
                                      platform_thread_info_t *info) {
  switch (id) {
    case THREAD_ID_TMO:
      info->wsp = wa_time_matched_obs_thread;
      info->size = sizeof(wa_time_matched_obs_thread);
      info->prio = NORMALPRIO + TIME_MATCHED_OBS_THREAD_PRIORITY;
      break;

    case THREAD_ID_STARLING:
      info->wsp = wa_starling_thread;
      info->size = sizeof(wa_starling_thread);
      info->prio = HIGHPRIO + STARLING_THREAD_PRIORITY;
      break;

    default:
      assert(!"Unknown thread ID");
      break;
  }
}

void platform_thread_create(const thread_id_t id, platform_routine_t *fn) {
  assert(fn);
  platform_thread_info_t info;
  platform_thread_info_init(id, &info);
  chThdCreateStatic(info.wsp, info.size, info.prio, fn, NULL);
}

void platform_thread_set_name(const char *name) { chRegSetThreadName(name); }

/* NDB */

void platform_watchdog_notify_starling_main_thread() {
  watchdog_notify(WD_NOTIFY_STARLING);
}

/* Mailbox */

typedef struct mailbox_info_s {
  mailbox_t mailbox;
  memory_pool_t mpool;
  msg_t *mailbox_buf;
  void *mpool_buf;
  uint8_t mailbox_len;
  size_t item_size;
} mailbox_info_t;

static mailbox_info_t mailbox_info[MB_ID_COUNT] =
    {[MB_ID_PAIRED_OBS] = {{0},
                           {0},
                           paired_obs_mailbox_buff,
                           paired_obs_buff,
                           PAIRED_OBS_N_BUFF,
                           sizeof(paired_obss_t)},
     [MB_ID_BASE_OBS] = {{0},
                         {0},
                         base_obs_mailbox_buff,
                         base_obs_buff,
                         BASE_OBS_N_BUFF,
                         sizeof(obs_array_t)},
     [MB_ID_ME_OBS] = {{0},
                       {0},
                       me_obs_mailbox_buff,
                       me_obs_buff,
                       ME_OBS_MSG_N_BUFF,
                       sizeof(obs_array_t)},
     [MB_ID_SBAS_DATA] = {{0},
                          {0},
                          sbas_data_mailbox_buff,
                          sbas_data_buff,
                          SBAS_DATA_N_BUFF,
                          sizeof(sbas_raw_data_t)},
     [MB_ID_EPHEMERIS] = {{0},
                          {0},
                          ephemeris_mailbox_buff,
                          ephemeris_buff,
                          EPHEMERIS_N_BUFF,
                          sizeof(ephemeris_array_t)},
     [MB_ID_IMU] = {
         {0}, {0}, imu_mailbox_buff, imu_buff, IMU_N_BUFF, sizeof(imu_data_t)}};

void platform_mailbox_init(mailbox_id_t id) {
  chMBObjectInit(&mailbox_info[id].mailbox,
                 mailbox_info[id].mailbox_buf,
                 mailbox_info[id].mailbox_len);
  chPoolObjectInit(&mailbox_info[id].mpool, mailbox_info[id].item_size, NULL);
  chPoolLoadArray(&mailbox_info[id].mpool,
                  mailbox_info[id].mpool_buf,
                  mailbox_info[id].mailbox_len);
}

errno_t platform_mailbox_post(mailbox_id_t id, void *msg, int blocking) {
  uint32_t timeout_ms =
      (MB_BLOCKING == blocking) ? MAILBOX_BLOCKING_TIMEOUT_MS : 0;
  if (MSG_OK !=
      chMBPost(&mailbox_info[id].mailbox, (msg_t)msg, MS2ST(timeout_ms))) {
    /* Full or mailbox reset while waiting */
    return EBUSY;
  }

  return 0;
}

errno_t platform_mailbox_fetch(mailbox_id_t id, void **msg, int blocking) {
  uint32_t timeout_ms =
      (MB_BLOCKING == blocking) ? MAILBOX_BLOCKING_TIMEOUT_MS : 0;
  if (MSG_OK !=
      chMBFetch(&mailbox_info[id].mailbox, (msg_t *)msg, MS2ST(timeout_ms))) {
    /* Empty or mailbox reset while waiting */
    return EBUSY;
  }

  return 0;
}

void *platform_mailbox_item_alloc(mailbox_id_t id) {
  return chPoolAlloc(&mailbox_info[id].mpool);
}

void platform_mailbox_item_free(mailbox_id_t id, const void *ptr) {
  chPoolFree(&mailbox_info[id].mpool, (void *)ptr);
}
