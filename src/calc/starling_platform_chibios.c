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
#include <ch.h>
#include <libpal/pal.h>
#include <starling/platform/mq.h>
#include <starling/platform/watchdog.h>
#include <string.h>

/* Used for watchdog implementation. */
#include "system_monitor/system_monitor.h"

/* From libpal for unimplemented functions */
#include "not_implemented.h"

/*******************************************************************************
 * Memory
 ******************************************************************************/

static void *chibios_mem_alloc(size_t size) { return chCoreAlloc(size); }

static void chibios_mem_free(void *mem) {
  NOT_IMPLEMENTED();
  (void)mem;
}

/*******************************************************************************
 * Mutex
 ******************************************************************************/

#define NUM_MUTEXES (20u)

static mutex_t mutexes[NUM_MUTEXES];
static size_t mutexes_used = 0;
static size_t mutexes_initd = 0;

static int chibios_mutex_init(size_t max_mutexes) {
  size_t new_total = mutexes_initd + max_mutexes;
  if (new_total > NUM_MUTEXES) {
    return PAL_INVALID;
  }
  for (; mutexes_initd < new_total; mutexes_initd++) {
    chMtxObjectInit(&mutexes[mutexes_initd]);
  }
  return PAL_SUCCESS;
}

static pal_mutex_t chibios_mutex_alloc(void) {
  assert(mutexes_used < mutexes_initd);
  /* not thread safe!!! */
  mutex_t *mutex = &mutexes[mutexes_used++];
  return (pal_mutex_t)mutex;
}

static void chibios_mutex_free(pal_mutex_t mutex) {
  NOT_IMPLEMENTED();
  (void)mutex;
}

static void chibios_mutex_lock(pal_mutex_t mutex) {
  chMtxLock((mutex_t *)mutex);
}

static void chibios_mutex_unlock(pal_mutex_t mutex) {
  chMtxUnlock((mutex_t *)mutex);
}

/*******************************************************************************
 * Thread
 ******************************************************************************/

#define STARLING_MAX_THREAD_STACK (3 * 1024 * 1024)

static THD_WORKING_AREA(wa_time_matched_obs_thread, STARLING_MAX_THREAD_STACK);
static THD_WORKING_AREA(wa_starling_thread, STARLING_MAX_THREAD_STACK);

typedef struct chibios_thread_working_area_s {
  void *work_area;
  size_t size;
  bool in_use;
} chibios_thread_working_area_t;

static chibios_thread_working_area_t chibios_thread_working_areas[] = {
    {.work_area = (void *)wa_starling_thread,
     .size = sizeof(wa_starling_thread),
     false},
    {.work_area = (void *)wa_time_matched_obs_thread,
     .size = sizeof(wa_time_matched_obs_thread),
     false},
};

static int chibios_thread_find_working_area(void **work_area_loc,
                                            size_t *size_loc,
                                            size_t stacksize) {
  int ret = 1;  // 1 indicates failure
  for (int i = 0; i < (int)ARRAY_SIZE(chibios_thread_working_areas); i++) {
    chibios_thread_working_area_t *was = &chibios_thread_working_areas[i];
    if (!was->in_use && was->size >= stacksize) {
      *work_area_loc = was->work_area;
      *size_loc = was->size;
      /* not thread safe!!! */
      was->in_use = true;
      ret = 0;  // 0 indicates success
      break;
    }
  }
  return ret;
}

static tprio_t chibios_prio_from_pal_prio(uint8_t prio) {
  tprio_t base_prio = pal_thread_is_high_prio(prio) ? HIGHPRIO : NORMALPRIO;
  uint8_t prio_diff = PAL_THREAD_MAX_PRIO - pal_thread_get_prio(prio);
  tprio_t chibios_prio = (tprio_t)((uint8_t)base_prio - prio_diff);
  return chibios_prio < LOWPRIO ? LOWPRIO : chibios_prio;
}

typedef struct chibios_thread_fn_wrapper_s {
  pal_thread_entry_t fn;
  void *ctx;
} chibios_thread_fn_wrapper_t;

static void chibios_thread_fn_wrapper(void *context) {
  chibios_thread_fn_wrapper_t *ctx = (chibios_thread_fn_wrapper_t *)context;
  (void)(ctx->fn(ctx->ctx));
}

static chibios_thread_fn_wrapper_t *chibios_make_thread_fn_wrapper_context(
    pal_thread_entry_t fn, void *context) {
  chibios_thread_fn_wrapper_t *ctx =
      (chibios_thread_fn_wrapper_t *)pal_mem_alloc(
          sizeof(chibios_thread_fn_wrapper_t));
  if (ctx != NULL) {
    ctx->fn = fn;
    ctx->ctx = context;
  }
  return ctx;
}

typedef struct chibios_thread_info_s {
  void *wsp;
  size_t size;
  tprio_t prio;
  tfunc_t fn;
  void *ctx;
} chibios_thread_info_t;

static void chibios_thread_info_init(chibios_thread_info_t *info,
                                     pal_thread_entry_t fn,
                                     void *context,
                                     size_t stacksize,
                                     uint8_t prio) {
  info->fn = chibios_thread_fn_wrapper;
  info->ctx = (void *)chibios_make_thread_fn_wrapper_context(fn, context);
  assert(info->ctx != 0);
  int chibios_thread_find_working_area_result =
      chibios_thread_find_working_area(&info->wsp, &info->size, stacksize);
  assert(chibios_thread_find_working_area_result == 0);
  info->prio = chibios_prio_from_pal_prio(prio);
}

static pal_thread_t chibios_thread_create(pal_thread_entry_t fn,
                                          void *ctx,
                                          size_t stacksize,
                                          uint8_t prio) {
  assert(fn);
  chibios_thread_info_t info;
  chibios_thread_info_init(&info, fn, ctx, stacksize, prio);
  thread_t *handle =
      chThdCreateStatic(info.wsp, info.size, info.prio, info.fn, info.ctx);
  return (pal_thread_t)handle;
}

static void chibios_thread_set_name(const char *name) {
  assert(name != NULL);
  chRegSetThreadName(name);
}

static void chibios_thread_join(pal_thread_t handle, void **retval) {
  assert(handle != NULL);
  assert(retval != NULL);
  *retval = (void *)chThdWait((thread_t *)handle);
}

static void chibios_thread_exit(void *code) { chThdExit((msg_t)code); }

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
  msg_t *mailbox_buf;
} mailbox_info_t;

static mailbox_info_t mailbox_info[MQ_ID_COUNT] = {
    [MQ_ID_PAIRED_OBS] = {{0}, NULL}, [MQ_ID_PRIMARY_DATA] = {{0}, NULL}};

static void chibios_mq_init(msg_queue_id_t id, size_t max_length) {
  mailbox_info[id].mailbox_buf = chCoreAlloc(sizeof(msg_t) * max_length);
  assert(mailbox_info[id].mailbox_buf);
  chMBObjectInit(
      &mailbox_info[id].mailbox, mailbox_info[id].mailbox_buf, max_length);
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

static void *chibios_mq_alloc(size_t size) { return chCoreAlloc(size); }

/*******************************************************************************
 * Condition Variable
 ******************************************************************************/

#define NUM_COND_VARS 8

static int convert_chibios_ret(msg_t ret) {
  switch (ret) {
    case MSG_OK:
      return PAL_SUCCESS;
    case MSG_TIMEOUT:
      return PAL_TIMEOUT;
    default:
      return PAL_ERROR;
  }
}

static condition_variable_t cond_vars[NUM_COND_VARS];
static size_t cond_vars_used = 0;
static size_t cond_vars_initd = 0;

/**
 * We make no effort here to reuse destroyed condition variables,
 * there is an upper bound on the number of condition variables which
 * may be created during a single execution, and that is that.
 */
static int chibios_cv_init(size_t max_cv) {
  size_t new_total = cond_vars_initd + max_cv;
  if (new_total > NUM_COND_VARS) {
    return PAL_INVALID;
  }
  for (; cond_vars_initd < new_total; cond_vars_initd++) {
    chCondObjectInit(&cond_vars[cond_vars_initd]);
  }
  return PAL_SUCCESS;
}

static pal_cv_t chibios_cv_alloc(void) {
  assert(cond_vars_used < cond_vars_initd);
  /* not thread safe!!! */
  condition_variable_t *cv = &cond_vars[cond_vars_used++];
  return (pal_cv_t)cv;
}

static void chibios_cv_free(pal_cv_t cv) { (void)cv; }

static void chibios_cv_notify_one(pal_cv_t cv) {
  chCondSignal((condition_variable_t *)cv);
}

static void chibios_cv_notify_all(pal_cv_t cv) {
  chCondBroadcast((condition_variable_t *)cv);
}

// lock must be both locked, and the most recently locked mutex
// before calling this function.
// Should not be used within ISRs
static void chibios_cv_wait(pal_cv_t cv, pal_mutex_t lock) {
  (void) lock;
  chCondWait((condition_variable_t *)cv);
}

// lock must be both locked, and the most recently locked mutex
// before calling this function.
// Should not be used within ISRs
static int chibios_cv_wait_for(pal_cv_t cv,
                                pal_mutex_t lock,
                                unsigned long millis) {
  (void) lock;
  const systime_t timeout = MS2ST(millis);
  int ret = chCondWaitTimeout((condition_variable_t *)cv, timeout);
  return convert_chibios_ret(ret);
}

/*******************************************************************************
 * PAL Initialization
 ******************************************************************************/
static bool pal_initialized = false;

void pal_init_impl(void) {
  if (!pal_initialized) {
    struct pal_impl_mem mem_impl = {
        .alloc = chibios_mem_alloc,
        .free = chibios_mem_free,
    };
    pal_set_impl_mem(&mem_impl);
    struct pal_impl_mutex mutex_impl = {
        .init = chibios_mutex_init,
        .alloc = chibios_mutex_alloc,
        .free = chibios_mutex_free,
        .lock = chibios_mutex_lock,
        .unlock = chibios_mutex_unlock,
    };
    pal_set_impl_mutex(&mutex_impl);
    struct pal_impl_thread thread_impl = {
        .create = chibios_thread_create,
        .set_name = chibios_thread_set_name,
        .join = chibios_thread_join,
        .exit = chibios_thread_exit,
    };
    pal_set_impl_thread(&thread_impl);
    cv_impl_t cv_impl = {
      .cv_init = chibios_cv_init,
      .cv_alloc = chibios_cv_alloc,
      .cv_free = chibios_cv_free,
      .cv_notify_one = chibios_cv_notify_one,
      .cv_notify_all = chibios_cv_notify_all,
      .cv_wait = chibios_cv_wait,
      .cv_wait_for = chibios_cv_wait_for,
    };
    pal_set_impl_cv(&cv_impl);
    
    pal_initialized = true;
  }
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void starling_initialize_platform(void) {
  /* Watchdog */
  platform_set_implementation_watchdog(
      chibios_watchdog_notify_starling_main_thread);
  /* Queue */
  mq_impl_t mq_impl = {
      .mq_init = chibios_mq_init,
      .mq_push = chibios_mq_push,
      .mq_pop = chibios_mq_pop,
      .mq_alloc = chibios_mq_alloc,
  };
  platform_set_implementation_mq(&mq_impl);
}
