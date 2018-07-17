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

#define MEO_QUEUE_NAME "meo-obs"
#define MEO_QUEUE_NORMAL_PRIO 0
static mqd_t meo_mqdes;

/* SBAS Data API data-structures. */
#define SBAS_DATA_N_BUFF 6
#define SBAS_QUEUE_NAME "sbas-data"
static mqd_t sbas_mqdes;

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

/* Time Matchied Obs */

void platform_time_matched_obs_mailbox_init() {
  struct mq_attr attr;

  attr.mq_maxmsg = STARLING_OBS_N_BUFF;
  attr.mq_msgsize = sizeof(obss_t *);
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

static int32_t platform_tmo_mb_post_internal(obss_t *msg, uint32_t timeout_ms, uint32_t msg_prio) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedsend(tmo_mqdes, (char *)&msg, sizeof(obss_t *), msg_prio, &ts);
}

int32_t platform_time_matched_obs_mailbox_post(obss_t *msg, uint32_t timeout_ms) {
  return platform_tmo_mb_post_internal(msg, timeout_ms, TMO_QUEUE_NORMAL_PRIO);
}

int32_t platform_time_matched_obs_mailbox_post_ahead(obss_t *msg,
                                                     uint32_t timeout_ms) {
  return platform_tmo_mb_post_internal(msg, timeout_ms, TMO_QUEUE_HIGH_PRIO);
}

int32_t platform_time_matched_obs_mailbox_fetch(obss_t **msg,
                                                uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedreceive(tmo_mqdes, (char *)*msg, sizeof(obss_t *), NULL, &ts);
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
  attr.mq_msgsize = sizeof(obss_t *);
  attr.mq_flags = 0;

  /* Blocking / non-blocking? */
  tmo_mqdes = mq_open(BO_QUEUE_NAME, O_RDWR | O_CREAT, 0777, &attr);

  /* Temporary queue. As soon as it's closed, it will be removed */
  mq_unlink(BO_QUEUE_NAME);
}

int32_t platform_base_obs_mailbox_post(obss_t *msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedsend(bo_mqdes, (char *)&msg, sizeof(obss_t *), BO_QUEUE_NORMAL_PRIO, &ts);
}

int32_t platform_base_obs_mailbox_fetch(obss_t **msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedreceive(bo_mqdes, (char *)*msg, sizeof(obss_t *), NULL, &ts);
}

obss_t *platform_base_obs_alloc(void) {
  /* Do we want memory pool rather than straight from heap? */
  return malloc(sizeof(obss_t));
}

void platform_base_obs_free(obss_t *ptr) {
  free(ptr);
}

/* ME obs messages */

void platform_me_obs_mailbox_init(void) {
  struct mq_attr attr;

  attr.mq_maxmsg = ME_OBS_MSG_N_BUFF;
  attr.mq_msgsize = sizeof(me_msg_obs_t *);
  attr.mq_flags = 0;

  /* Blocking / non-blocking? */
  tmo_mqdes = mq_open(MEO_QUEUE_NAME, O_RDWR | O_CREAT, 0777, &attr);

  /* Temporary queue. As soon as it's closed, it will be removed */
  mq_unlink(MEO_QUEUE_NAME);
}

int32_t platform_me_obs_mailbox_post(me_msg_obs_t *msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedsend(meo_mqdes, (char *)msg, sizeof(me_msg_obs_t *), MEO_QUEUE_NORMAL_PRIO, &ts);
}

int32_t platform_me_obs_mailbox_fetch(me_msg_obs_t **msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedreceive(meo_mqdes, (char *)msg, sizeof(me_msg_obs_t *), NULL, &ts);
}

me_msg_obs_t *platform_me_obs_alloc(void) {
  /* Do we want memory pool rather than straight from heap? */
  return malloc(sizeof(me_msg_obs_t));
}

void platform_me_obs_free(me_msg_obs_t *ptr) {
  free(ptr);
}

/* SBAS messages */
void platform_sbas_data_mailbox_setup(void) {
  struct mq_attr attr;

  attr.mq_maxmsg = SBAS_DATA_N_BUFF;
  attr.mq_msgsize = sizeof(sbas_raw_data_t *);
  attr.mq_flags = 0;

  /* Blocking / non-blocking? */
  sbas_mqdes = mq_open(SBAS_QUEUE_NAME, O_RDWR | O_CREAT, 0777, &attr);

  /* Temporary queue. As soon as it's closed, it will be removed */
  mq_unlink(SBAS_QUEUE_NAME);
}

/* TODO(kevin) error handling by return code for platform functions. */
void platform_sbas_data_mailbox_post(const sbas_raw_data_t *sbas_data) {
  sbas_raw_data_t *msg = malloc(sizeof(sbas_raw_data_t));
  if (NULL == msg) {
    log_error("ME: Could not allocate pool for SBAS!");
    return;
  }
  assert(sbas_data);
  *msg = *sbas_data;

  if (0 != mq_send(meo_mqdes, (char *)&msg, sizeof(sbas_raw_data_t *), MEO_QUEUE_NORMAL_PRIO)) {
    log_error("ME: Mailbox should have space for SBAS!");
    platform_sbas_data_free(msg);
  }
}

int32_t platform_sbas_data_mailbox_fetch(sbas_raw_data_t **msg, uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  return mq_timedreceive(meo_mqdes, (char *)*msg, sizeof(sbas_raw_data_t *), NULL, &ts);
}

void platform_sbas_data_free(sbas_raw_data_t *ptr) {
  free(ptr);
}
