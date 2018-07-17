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
#include <mqueue.h>
#include <pthread.h>
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
#define TMO_QUEUE_NAME "time-matched-obs"
#define TMO_QUEUE_NORMAL_PRIO 0
#define TMO_QUEUE_HIGH_PRIO 1
static mqd_t tmo_mqdes;

/** Keep a mailbox of received base obs so we can process all of them in
 * order even if we have a bursty base station connection. */
#define BO_QUEUE_NAME "base-obs"
#define BO_QUEUE_NORMAL_PRIO 0
static mqd_t bo_mqdes;

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
  pthread_t id;
  size_t size;
  int prio;
};

void platform_mutex_lock(void *mtx) {
  pthread_mutex_lock((pthread_mutex_t *)mtx);
}

void platform_mutex_unlock(void *mtx) {
  pthread_mutex_unlock((pthread_mutex_t *)mtx);
}

/* phtread_create expects a pointer to type (void *)()(void *).
 * starling routines are of type (void)()(void *) */
static void *start_routine_wrapper(void *arg) {
  ((platform_routine_t *)arg)(NULL);
  return NULL;
}

static int sch_policy = SCHED_FIFO;

void platform_thread_info_init(const thread_id_t id,
                               platform_thread_info_t *info) {
  info = (platform_thread_info_t *)malloc(sizeof(platform_thread_info_t));

  int max_prio = sched_get_priority_max(sch_policy);

  assert(0 < max_prio);

  switch (id) {
    case THREAD_ID_TMO:
      info->size = TIME_MATCHED_OBS_THREAD_STACK;
      /* TODO: scale priority properly to pthread context
       * See http://man7.org/linux/man-pages/man7/sched.7.html 
       * Processes scheduled under one of the real-time policies (SCHED_FIFO,
       * SCHED_RR) have a sched_priority value in the range 1 (low) to 99
       * (high). */
      info->prio = max_prio + TIME_MATCHED_OBS_THREAD_PRIORITY;
      break;

    default:
      assert(!"Unkonwn thread ID");
      break;
  }

  assert(0 < info->prio);
}

void platform_thread_create(platform_thread_info_t *info,
                            int prio,
                            platform_routine_t *fn,
                            void *arg) {
  (void)arg;
  pthread_attr_t attr;
  struct sched_param sch_params;
  sch_params.sched_priority = prio;

  if (0 != pthread_attr_init(&attr)) {
    assert(!"pthread_attr_init()");
  }

  if (0 < info->size) {
    if (0 != pthread_attr_setstacksize(&attr, info->size)) {
      assert(!"pthread_attr_setstacksize");
    }
  }

  if (0 != pthread_create(&info->id, &attr, &start_routine_wrapper, fn)) {
    assert(!"pthread_create()");
  }

  if (0 != pthread_setschedparam(info->id, sch_policy, &sch_params)) {
    assert(!"pthread_setschedparam()");
  }

  if (0 != pthread_attr_destroy(&attr)) {
    assert(!"pthread_attr_destroy()");
  }
}

void platform_thread_set_name(const char *name) {
  pthread_setname_np(pthread_self(), name);
}

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
  struct mq_attr attr;

  attr.mq_maxmsg = STARLING_OBS_N_BUFF;
  attr.mq_msgsize = sizeof(obss_t);
  attr.mq_flags = 0;

  /* Blocking / non-blocking? */
  tmo_mqdes = mq_open(TMO_QUEUE_NAME, O_RDWR | O_CREAT, 0777, &attr);

  /* Temporary queue. As soon as it's closed, it will be removed */
  mq_unlink(TMO_QUEUE_NAME);
}

static void platform_get_timeout(const uint32_t timeout_ms, struct timespec *ts) {
  if (0 != clock_gettime(CLOCK_REALTIME, ts)) {
    assert(!"clock_gettime");
  }
  ts->tv_nsec += timeout_ms * 1e6;
}

static int32_t platform_tmo_mb_post_internal(int32_t msg, uint32_t timeout_ms, uint32_t msg_prio) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedsend(tmo_mqdes, (char *)msg, sizeof(obss_t), msg_prio, &ts);
}

int32_t platform_time_matched_obs_mailbox_post(int32_t msg, uint32_t timeout_ms) {
  return platform_tmo_mb_post_internal(msg, timeout_ms, TMO_QUEUE_NORMAL_PRIO);
}

int32_t platform_time_matched_obs_mailbox_post_ahead(int32_t msg,
                                                     uint32_t timeout_ms) {
  return platform_tmo_mb_post_internal(msg, timeout_ms, TMO_QUEUE_HIGH_PRIO);
}

static int platform_mb_fetch(int32_t *msg, uint32_t timeout_ms, mqd_t mqdes) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedreceive(mqdes, (char *)msg, sizeof(obss_t), NULL, &ts);
}

int32_t platform_time_matched_obs_mailbox_fetch(int32_t *msg,
                                                uint32_t timeout_ms) {
  return platform_mb_fetch(msg, timeout_ms, tmo_mqdes);
}

obss_t *platform_time_matched_obs_alloc(void) {
  /* Do we want memory pool rather than straight from heap? */
  return malloc(sizeof(obss_t));
}

void platform_time_matched_obs_free(obss_t *ptr) {
  free(ptr);
}

/* Base obs */

void platform_base_obs_mailbox_init() {
  struct mq_attr attr;

  attr.mq_maxmsg = BASE_OBS_N_BUFF;
  attr.mq_msgsize = sizeof(obss_t);
  attr.mq_flags = 0;

  /* Blocking / non-blocking? */
  tmo_mqdes = mq_open(BO_QUEUE_NAME, O_RDWR | O_CREAT, 0777, &attr);

  /* Temporary queue. As soon as it's closed, it will be removed */
  mq_unlink(BO_QUEUE_NAME);
}

int32_t platform_base_obs_mailbox_post(int32_t msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedsend(bo_mqdes, (char *)msg, sizeof(obss_t), BO_QUEUE_NORMAL_PRIO, &ts);
}

int32_t platform_base_obs_mailbox_fetch(int32_t *msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedreceive(bo_mqdes, (char *)msg, sizeof(obss_t), NULL, &ts);
}

obss_t *platform_base_obs_alloc(void) {
  /* Do we want memory pool rather than straight from heap? */
  return malloc(sizeof(obss_t));
}

void platform_base_obs_free(obss_t *ptr) {
  free(ptr);
}

/* ME obs messages */
int32_t platform_me_obs_msg_mailbox_fetch(int32_t *msg, uint32_t timeout) {
  return chMBFetch(&me_obs_msg_mailbox, (msg_t *)msg, (systime_t)timeout);
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

int32_t platform_sbas_data_mailbox_fetch(int32_t *msg, uint32_t timeout) {
  return chMBFetch(&sbas_data_mailbox, (msg_t *)msg, (systime_t)timeout);
}

void platform_sbas_data_free(sbas_raw_data_t *ptr) {
  chPoolFree(&sbas_data_buff_pool, ptr);
}
