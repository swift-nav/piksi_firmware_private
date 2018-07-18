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

#define MSG_PRIO_NORMAL 0
#define MSG_PRIO_HIGH 1

/* Time-matched observations data-structures. */
#define TMO_QUEUE_NAME "time-matched-obs"

/** Keep a mailbox of received base obs so we can process all of them in
 * order even if we have a bursty base station connection. */
#define BO_QUEUE_NAME "base-obs"

#define MEO_QUEUE_NAME "meo-obs"

/* SBAS Data API data-structures. */
#define SBAS_DATA_N_BUFF 6
#define SBAS_DATA_QUEUE_NAME "sbas-data"

/*******************************************************************************
 * Platform Shim Calls
 ******************************************************************************/

/* Mutex */

void platform_mutex_lock(void *mtx) {
  pthread_mutex_lock((pthread_mutex_t *)mtx);
}

void platform_mutex_unlock(void *mtx) {
  pthread_mutex_unlock((pthread_mutex_t *)mtx);
}

/* Threading */

typedef struct platform_thread_info_s {
  pthread_t thread_id;
  size_t size;
  int prio;
} platform_thread_info_t;

/* phtread_create expects a pointer to type (void *)()(void *).
 * starling routines are of type (void)()(void *) */
static void *start_routine_wrapper(void *arg) {
  ((platform_routine_t *)arg)(NULL);
  return NULL;
}

static int sch_policy = SCHED_FIFO;

static void platform_thread_info_init(const thread_id_t id,
                                      platform_thread_info_t *info) {
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

void platform_thread_create(const thread_id_t id,
                            platform_routine_t *fn) {
  assert(fn);
  pthread_attr_t attr;
  platform_thread_info_t info;
  struct sched_param sch_params;

  platform_thread_info_init(id, &info);

  sch_params.sched_priority = info.prio;

  if (0 != pthread_attr_init(&attr)) {
    assert(!"pthread_attr_init()");
  }

  if (0 < info.size) {
    if (0 != pthread_attr_setstacksize(&attr, info.size)) {
      assert(!"pthread_attr_setstacksize");
    }
  }

  if (0 != pthread_create(&info.thread_id, &attr, &start_routine_wrapper, fn)) {
    assert(!"pthread_create()");
  }

  if (0 != pthread_setschedparam(info.thread_id, sch_policy, &sch_params)) {
    assert(!"pthread_setschedparam()");
  }

  if (0 != pthread_attr_destroy(&attr)) {
    assert(!"pthread_attr_destroy()");
  }
}

void platform_thread_set_name(const char *name) {
  pthread_setname_np(pthread_self(), name);
}

/* NDB */

/* Return true on success. */
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph) {
  return (ndb_ephemeris_read(sid, eph) == NDB_ERR_NONE);
}

/* Return true on success. */
bool platform_try_read_iono_corr(ionosphere_t *params) {
  return (ndb_iono_corr_read(params) == NDB_ERR_NONE);
}

void platform_watchdog_notify_starling_main_thread() {
  /* TODO */
}

/* Mailbox */

typedef struct mailbox_info_s {
  mqd_t mailbox;
  char *mailbox_name;
  uint8_t mailbox_len;
  size_t item_size;
} mailbox_info_t;

static mailbox_info_t mailbox_info[MB_ID_COUNT] =
    {[MB_ID_TIME_MATCHED_OBS] = {0,
                                 TMO_QUEUE_NAME,
                                 STARLING_OBS_N_BUFF,
                                 sizeof(obss_t *)},
     [MB_ID_BASE_OBS] = {0, BO_QUEUE_NAME, BASE_OBS_N_BUFF, sizeof(obss_t *)},
     [MB_ID_ME_OBS] = {0,
                       MEO_QUEUE_NAME,
                       ME_OBS_MSG_N_BUFF,
                       sizeof(me_msg_obs_t *)},
     [MB_ID_SBAS_DATA] = {
         0, SBAS_DATA_QUEUE_NAME, SBAS_DATA_N_BUFF, sizeof(sbas_raw_data_t *)}};

void platform_mailbox_init(mailbox_id_t id) {
  struct mq_attr attr;

  attr.mq_maxmsg = mailbox_info[id].mailbox_len;
  attr.mq_msgsize = mailbox_info[id].item_size;
  attr.mq_flags = 0;

  /* Blocking / non-blocking? */
  mailbox_info[id].mailbox =
      mq_open(mailbox_info[id].mailbox_name, O_RDWR | O_CREAT, 0777, &attr);

  /* Temporary queue. As soon as it's closed, it will be removed */
  mq_unlink(mailbox_info[id].mailbox_name);
}

static void platform_get_timeout(const uint32_t timeout_ms,
                                 struct timespec *ts) {
  if (0 != clock_gettime(CLOCK_REALTIME, ts)) {
    assert(!"clock_gettime");
  }
  ts->tv_nsec += timeout_ms * 1e6;
}

static int platform_mailbox_post_internal(mailbox_id_t id,
                                                 void *msg,
                                                 uint32_t timeout_ms,
                                                 uint32_t msg_prio) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  /* Similar to ChibiOS mail system, do not post the actual struct but the
   * pointer value indicating the memory address where the struct is. */
  errno = 0;
  if (0 != mq_timedsend(mailbox_info[id].mailbox,
                        (char *)&msg,
                        mailbox_info[id].item_size,
                        msg_prio,
                        &ts)) {
    return errno;
  }

  return 0;
}

int platform_mailbox_post(mailbox_id_t id,
                                 void *msg,
                                 uint32_t timeout_ms) {
  return platform_mailbox_post_internal(id, msg, timeout_ms, MSG_PRIO_NORMAL);
}

int platform_mailbox_post_ahead(mailbox_id_t id,
                                       void *msg,
                                       uint32_t timeout_ms) {
  return platform_mailbox_post_internal(id, msg, timeout_ms, MSG_PRIO_HIGH);
}

int platform_mailbox_fetch(mailbox_id_t id,
                                  void **msg,
                                  uint32_t timeout_ms) {
  struct timespec ts = {0};
  platform_get_timeout(timeout_ms, &ts);

  errno = 0;
  if (0 != mq_timedreceive(mailbox_info[id].mailbox,
                           (char *)*msg,
                           mailbox_info[id].item_size,
                           NULL,
                           &ts)) {
    return errno;
  }

  return 0;
}

void *platform_mailbox_item_alloc(mailbox_id_t id) {
  /* Do we want memory pool rather than straight from heap? */
  return malloc(mailbox_info[id].item_size);
}

void platform_mailbox_item_free(mailbox_id_t id, void *ptr) {
  (void)id;
  free(ptr);
}
