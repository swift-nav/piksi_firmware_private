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

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libsbp/sbp.h>
#include <starling/integration/starling_input_bridge.h>
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
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define MAILBOX_BLOCKING_TIMEOUT_MS 5000

#define NUM_MUTEXES STARLING_MAX_NUM_MUTEXES

/*******************************************************************************
 * Local Variables
 ******************************************************************************/

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
} mailbox_info_t;

static mailbox_info_t mailbox_info[MQ_ID_COUNT] =
    {[MQ_ID_PAIRED_OBS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_BASE_OBS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_ME_OBS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_SBAS_DATA] = {{0}, {0}, NULL, NULL},
     [MQ_ID_EPHEMERIS] = {{0}, {0}, NULL, NULL},
     [MQ_ID_IMU] = {{0}, {0}, NULL, NULL}};

void platform_mq_init(msg_queue_id_t id, size_t msg_size, size_t max_length) {
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

errno_t platform_mq_push(msg_queue_id_t id,
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

errno_t platform_mq_pop(msg_queue_id_t id,
                        void **msg,
                        mq_blocking_mode_t should_block) {
  uint32_t timeout_ms =
      (MQ_BLOCKING == should_block) ? MAILBOX_BLOCKING_TIMEOUT_MS : 0;
  if (MSG_OK !=
      chMBFetch(&mailbox_info[id].mailbox, (msg_t *)msg, MS2ST(timeout_ms))) {
    /* Empty or mailbox reset while waiting */
    msg = NULL;
    return EBUSY;
  }

  return 0;
}

void *platform_mq_alloc_msg(msg_queue_id_t id) {
  return chPoolAlloc(&mailbox_info[id].mpool);
}

void platform_mq_free_msg(msg_queue_id_t id, void *msg) {
  chPoolFree(&mailbox_info[id].mpool, msg);
}
