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
#include <starling/platform/semaphore.h>
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

enum PAL_MUTEX_INIT_RESULT {
  PAL_MUTEX_INIT_MAX_BEYOND_SUPPLY = -1,
  PAL_MUTEX_INIT_SUCCESS = 0,
};

static int chibios_mutex_init(size_t max_mutexes) {
  if (max_mutexes > NUM_MUTEXES) {
    return (int)PAL_MUTEX_INIT_MAX_BEYOND_SUPPLY;
  }
  for (size_t i = 0; i < max_mutexes; i++) {
    chMtxObjectInit(&mutexes[i]);
  }
  return (int)PAL_MUTEX_INIT_SUCCESS;
}

static pal_mutex_t chibios_mutex_alloc(void) {
  assert(mutexes_used < NUM_MUTEXES);
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
  /* this is a total hack to keep TM thread at correct value */
  if (prio == PAL_THREAD_DEFAULT_PRIO) {
    chibios_prio = (tprio_t)((uint8_t)base_prio - 3);
  }
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

/**
 * We make no effort here to reuse destroyed semaphores,
 * there is an upper bound on the number of semaphores which
 * may be created during a single execution, and that is that.
 */
static platform_sem_t *chibios_sem_create(void) {
  static int n_semaphores = 0;
  static semaphore_t semaphores[MAX_N_SEMAPHORES];

  if (n_semaphores >= MAX_N_SEMAPHORES) {
    return NULL;
  }

  semaphore_t *sem = &semaphores[n_semaphores++];
  int count = 0;
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
 * PAL Initialization
 ******************************************************************************/

void pal_init_impl(void) {
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
  /* Semaphore */
  sem_impl_t sem_impl = {
      .sem_create = chibios_sem_create,
      .sem_destroy = chibios_sem_destroy,
      .sem_signal = chibios_sem_signal,
      .sem_wait = chibios_sem_wait,
      .sem_wait_timeout = chibios_sem_wait_timeout,
  };
  platform_set_implementation_semaphore(&sem_impl);
}
