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

#include <libsbp/sbp.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/sbas_raw_data.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/single_epoch_solver.h>
#include <libswiftnav/troposphere.h>

#include "calc_base_obs.h"
#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "main.h"
#include "manage.h"
#include "me_msg/me_msg.h"
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
#include "starling_platform_shim.h"
#include "starling_threads.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

/* Time-matched observations data-structures. */
static memory_pool_t time_matched_obs_buff_pool;
static mailbox_t time_matched_obs_mailbox;

/* SBAS Data API data-structures. */
#define SBAS_DATA_N_BUFF 6
static mailbox_t sbas_data_mailbox;
static memory_pool_t sbas_data_buff_pool;
static msg_t sbas_data_mailbox_buff[SBAS_DATA_N_BUFF];
static sbas_raw_data_t sbas_data_buff[SBAS_DATA_N_BUFF];

/*******************************************************************************
 * Platform Shim Calls
 ******************************************************************************/

struct platform_thread_info_s {
  void *wsp;
  size_t size;
  int prio;
};

static THD_WORKING_AREA(wa_time_matched_obs_thread,
                        TIME_MATCHED_OBS_THREAD_STACK);

void platform_mutex_lock(void *mtx) { chMtxLock((mutex_t *)mtx); }

void platform_mutex_unlock(void *mtx) { chMtxUnlock((mutex_t *)mtx); }

void platform_pool_free(void *pool, void *buf) { chPoolFree(pool, buf); }

void platform_thread_info_init(const thread_id_t id,
                               platform_thread_info_t *info) {
  info = (platform_thread_info_t *)malloc(sizeof(platform_thread_info_t));
  switch (id) {
    case THREAD_ID_TMO:
      info->wsp = wa_time_matched_obs_thread;
      info->size = sizeof(wa_time_matched_obs_thread);
      info->prio = NORMALPRIO + TIME_MATCHED_OBS_THREAD_PRIORITY;
      break;

    default:
      assert(!"Unkonwn thread ID");
      break;
  }
}

void platform_thread_create(platform_thread_info_t *info,
                            int prio,
                            platform_routine_t *fn,
                            void *arg) {
  chThdCreateStatic(info->wsp, info->size, prio, fn, arg);
}

void platform_thread_set_name(const char *name) { chRegSetThreadName(name); }

/* Return true on success. */
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph) {
  return (ndb_ephemeris_read(sid, eph) == NDB_ERR_NONE);
}

/* Return true on success. */
bool platform_try_read_iono_corr(ionosphere_t *params) {
  return (ndb_iono_corr_read(params) == NDB_ERR_NONE);
}

void platform_watchdog_notify_starling_main_thread() {
  watchdog_notify(WD_NOTIFY_STARLING);
}

void platform_time_matched_obs_mailbox_init() {
  static msg_t time_matched_obs_mailbox_buff[STARLING_OBS_N_BUFF];
  chMBObjectInit(&time_matched_obs_mailbox,
                 time_matched_obs_mailbox_buff,
                 STARLING_OBS_N_BUFF);
  chPoolObjectInit(&time_matched_obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[STARLING_OBS_N_BUFF] _CCM;
  chPoolLoadArray(&time_matched_obs_buff_pool, obs_buff, STARLING_OBS_N_BUFF);
}

int32_t platform_time_matched_obs_mailbox_post(int32_t msg, uint32_t timeout_ms) {
  return chMBPost(&time_matched_obs_mailbox, (msg_t)msg, MS2ST(timeout_ms));
}

int32_t platform_time_matched_obs_mailbox_post_ahead(int32_t msg,
                                                     uint32_t timeout_ms) {
  return chMBPostAhead(
      &time_matched_obs_mailbox, (msg_t)msg, MS2ST(timeout_ms));
}

int32_t platform_time_matched_obs_mailbox_fetch(int32_t *msg,
                                                uint32_t timeout_ms) {
  return chMBFetch(&time_matched_obs_mailbox, (msg_t *)msg, MS2ST(timeout_ms));
}

obss_t *platform_time_matched_obs_alloc(void) {
  return chPoolAlloc(&time_matched_obs_buff_pool);
}
void platform_time_matched_obs_free(obss_t *ptr) {
  chPoolFree(&time_matched_obs_buff_pool, ptr);
}

void platform_base_obs_free(obss_t *ptr) {
  chPoolFree(&base_obs_buff_pool, ptr);
}

int32_t platform_base_obs_mailbox_fetch(int32_t *msg, uint32_t timeout_ms) {
  return chMBFetch(&base_obs_mailbox, (msg_t *)msg, MS2ST(timeout_ms));
}

/* ME obs messages */
int32_t platform_me_obs_msg_mailbox_fetch(int32_t *msg, uint32_t timeout_ms) {
  return chMBFetch(&me_obs_msg_mailbox, (msg_t *)msg, MS2ST(timeout_ms));
}

void platform_me_obs_msg_free(me_msg_obs_t *ptr) {
  chPoolFree(&me_obs_msg_buff_pool, ptr);
}

/* SBAS messages */
void platform_sbas_data_mailbox_setup(void) {
  chMBObjectInit(&sbas_data_mailbox, sbas_data_mailbox_buff, SBAS_DATA_N_BUFF);
  chPoolObjectInit(&sbas_data_buff_pool, sizeof(sbas_raw_data_t), NULL);
  chPoolLoadArray(&sbas_data_buff_pool, sbas_data_buff, SBAS_DATA_N_BUFF);
}

/* TODO(kevin) error handling by return code for platform functions. */
void platform_sbas_data_mailbox_post(const sbas_raw_data_t *sbas_data) {
  sbas_raw_data_t *sbas_data_msg = chPoolAlloc(&sbas_data_buff_pool);
  if (NULL == sbas_data_msg) {
    log_error("ME: Could not allocate pool for SBAS!");
    return;
  }
  assert(sbas_data);
  *sbas_data_msg = *sbas_data;
  msg_t ret =
      chMBPost(&sbas_data_mailbox, (msg_t)sbas_data_msg, TIME_IMMEDIATE);
  if (ret != MSG_OK) {
    log_error("ME: Mailbox should have space for SBAS!");
    chPoolFree(&sbas_data_buff_pool, sbas_data_msg);
  }
}

int32_t platform_sbas_data_mailbox_fetch(int32_t *msg, uint32_t timeout_ms) {
  return chMBFetch(&sbas_data_mailbox, (msg_t *)msg, MS2ST(timeout_ms));
}

void platform_sbas_data_free(sbas_raw_data_t *ptr) {
  chPoolFree(&sbas_data_buff_pool, ptr);
}
