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

#include <starling/platform/mq.h>
#include <starling/platform/starling_platform.h>
#include <starling/platform/starling_platform_semaphore.h>

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libsbp/sbp.h>
#include <starling/observation.h>
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
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

/*******************************************************************************
 * Mutex
 ******************************************************************************/

#define NUM_MUTEXES STARLING_MAX_NUM_MUTEXES

static mutex_t mutexes[NUM_MUTEXES];

static int chibios_mutex_init(mtx_id_t id) {
  if (id >= NUM_MUTEXES) {
    return -1;
  }
  chMtxObjectInit(&mutexes[id]);
  return 0;
}

static void chibios_mutex_lock(mtx_id_t id) { chMtxLock(&mutexes[id]); }

static void chibios_mutex_unlock(mtx_id_t id) { chMtxUnlock(&mutexes[id]); }

/*******************************************************************************
 * Thread
 ******************************************************************************/

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

static void chibios_thread_create(const thread_id_t id, platform_routine_t *fn) {
  assert(fn);
  platform_thread_info_t info;
  platform_thread_info_init(id, &info);
  chThdCreateStatic(info.wsp, info.size, info.prio, fn, NULL);
}

static void chibios_thread_set_name(const char *name) { chRegSetThreadName(name); }

/*******************************************************************************
 * Watchdog
 ******************************************************************************/


static void chibios_watchdog_notify_starling_main_thread(void) {
  watchdog_notify(WD_NOTIFY_STARLING);
}

/*******************************************************************************
 * Queue
 ******************************************************************************/

#define MAILBOX_BLOCKING_TIMEOUT_MS 5000

typedef struct mailbox_info_s {
  mailbox_t mailbox;
  memory_pool_t mpool;
  msg_t *mailbox_buf;
  void *mpool_buf;
} mailbox_info_t;

static mailbox_info_t mailbox_info[MQ_ID_COUNT] =
    {[MQ_ID_PAIRED_OBS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_BASE_OBS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_ME_OBS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_SBAS_DATA] = {{0}, {0}, NULL, NULL},
     [MQ_ID_EPHEMERIS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_IMU] = {{0}, {0}, NULL, NULL}};

static void chibios_mq_init(msg_queue_id_t id, size_t msg_size, size_t max_length) {
  mailbox_info[id].mailbox_buf = chCoreAlloc(sizeof(msg_t) * max_length);
  mailbox_info[id].mpool_buf = chCoreAlloc(msg_size * max_length);
  assert(mailbox_info[id].mailbox_buf);
  assert(mailbox_info[id].mpool_buf);
  chMBObjectInit(
      &mailbox_info[id].mailbox, mailbox_info[id].mailbox_buf, max_length);
  chPoolObjectInit(&mailbox_info[id].mpool, msg_size, NULL);
  chPoolLoadArray(
      &mailbox_info[id].mpool, mailbox_info[id].mpool_buf, max_length);
}

static errno_t chibios_mq_push(msg_queue_id_t id,
                         void *msg,
                         mq_blocking_mode_t should_block) {
  uint32_t timeout_ms =
      (MQ_BLOCKING == should_block) ? MAILBOX_BLOCKING_TIMEOUT_MS : 0;
  if (MSG_OK !=
      chMBPost(&mailbox_info[id].mailbox, (msg_t)msg, MS2ST(timeout_ms))) {
    /* Full or mailbox reset while waiting */
    return EBUSY;
  }

  return 0;
}

static errno_t chibios_mq_pop(msg_queue_id_t id,
                        void **msg,
                        mq_blocking_mode_t should_block) {
  uint32_t timeout_ms =
      (MQ_BLOCKING == should_block) ? MAILBOX_BLOCKING_TIMEOUT_MS : 0;
  if (MSG_OK !=
      chMBFetch(&mailbox_info[id].mailbox, (msg_t *)msg, MS2ST(timeout_ms))) {
    /* Empty or mailbox reset while waiting */
    *msg = NULL;
    return EBUSY;
  }

  return 0;
}

static void *chibios_mq_alloc_msg(msg_queue_id_t id) {
  return chPoolAlloc(&mailbox_info[id].mpool);
}

static void chibios_mq_free_msg(msg_queue_id_t id, void *msg) {
  chPoolFree(&mailbox_info[id].mpool, msg);
}

/*******************************************************************************
 * Semaphore
 ******************************************************************************/

#define MAX_N_SEMAPHORES 8

static int convert_chibios_ret(msg_t ret) {
  switch (ret) {
    case MSG_OK:
      return PLATFORM_SEM_OK;
    case MSG_TIMEOUT:
      return PLATFORM_SEM_TIMEOUT;
    default:
      return PLATFORM_SEM_ERROR;
  }
}

static platform_sem_t *chibios_sem_create(void) {
  return platform_sem_create_count(0);
}

/**
 * We make no effort here to reuse destroyed semaphores,
 * there is an upper bound on the number of semaphores which
 * may be created during a single execution, and that is that.
 */
static platform_sem_t *chibios_sem_create_count(int count) {
  static int n_semaphores = 0;
  static semaphore_t semaphores[MAX_N_SEMAPHORES];

  if (n_semaphores >= MAX_N_SEMAPHORES) {
    return NULL;
  }

  semaphore_t *sem = &semaphores[n_semaphores++];

  chSemObjectInit(sem, count);
  return (platform_sem_t *)sem;
}

static void chibios_sem_destroy(platform_sem_t **sem_loc) {
  if (sem_loc) {
    *sem_loc = NULL;
  }
}

static void chibios_sem_signal(platform_sem_t *sem) {
  chSemSignal((semaphore_t *)sem);
}

static int chibios_sem_wait(platform_sem_t *sem) {
  int ret = chSemWait((semaphore_t *)sem);
  return convert_chibios_ret(ret);
}

static int chibios_sem_wait_timeout(platform_sem_t *sem, unsigned long millis) {
  const systime_t timeout = MS2ST(millis);
  int ret = chSemWaitTimeout((semaphore_t *)sem, timeout);
  return convert_chibios_ret(ret);
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void init_starling_platform_chibios_implementation(void) {
  /* Mutex */
  mutex_impl_t mutex_impl = {
    .mutex_init = chibios_mutex_init,
    .mutex_lock = chibios_mutex_lock,
    .mutex_unlock = chibios_mutex_unlock,
  };
  platform_set_implementation_mutex(&mutex_impl); 
  /* Thread */
  thread_impl_t thread_impl = {
    .thread_create = chibios_thread_create,
    .thread_set_name = chibios_thread_set_name,
  };
  platform_set_implementation_thread(&thread_impl);
  /* Watchdog */
  platform_set_implementation_watchdog(chibios_watchdog_notify_starling_main_thread);
  /* Queue */
  mq_impl_t mq_impl = {
    .mq_init = chibios_mq_init,
    .mq_push = chibios_mq_push,
    .mq_pop = chibios_mq_pop,
    .mq_alloc_msg = chibios_mq_alloc_msg,
    .mq_free_msg = chibios_mq_free_msg,
  };
  platform_set_implementation_mq(&mq_impl);
  /* Semaphore */
  sem_impl_t sem_impl = {
    .sem_create = chibios_sem_create,
    .sem_create_count = chibios_sem_create_count,
    .sem_destroy = chibios_sem_destroy,
    .sem_signal = chibios_sem_signal,
    .sem_wait = chibios_sem_wait,
    .sem_wait_timeout = chibios_sem_wait_timeout,
  };
  platform_set_implementation_semaphore(&sem_impl);
}

