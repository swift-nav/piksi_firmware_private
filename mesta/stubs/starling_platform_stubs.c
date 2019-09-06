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

#include <libpal/pal.h>
#include <starling/platform/mq.h>
#include <starling/platform/semaphore.h>
#include <starling/platform/watchdog.h>

/*******************************************************************************
 * Mutex
 ******************************************************************************/

static int stub_mutex_init(size_t max_mutexes) {
  (void)max_mutexes;
  return PAL_INVALID;
}

static pal_mutex_t stub_mutex_alloc(void) { return NULL; }

static void stub_mutex_free(pal_mutex_t mutex) { (void)mutex; }

static void stub_mutex_lock(pal_mutex_t mutex) { (void)mutex; }

static void stub_mutex_unlock(pal_mutex_t mutex) { (void)mutex; }

/*******************************************************************************
 * Thread
 ******************************************************************************/

static pal_thread_t stub_thread_create(pal_thread_entry_t fn,
                                       void *ctx,
                                       size_t stacksize,
                                       uint8_t prio) {
  (void)fn;
  (void)ctx;
  (void)stacksize;
  (void)prio;
  return NULL;
}

static void stub_thread_set_name(const char *name) { (void)name; }

static void stub_thread_join(pal_thread_t thread, void **retval) {
  (void)thread;
  (void)retval;
}

static void stub_thread_exit(void *code) { (void)code; }

/*******************************************************************************
 * Watchdog
 ******************************************************************************/

static void stub_watchdog_notify_starling_main_thread(void) {}

/*******************************************************************************
 * Queue
 ******************************************************************************/

static void stub_mq_init(msg_queue_id_t id, size_t max_length) {
  (void)id;
  (void)max_length;
}

static errno_t stub_mq_push(msg_queue_id_t id,
                            void *msg,
                            mq_blocking_mode_t should_block) {
  (void)id;
  (void)msg;
  (void)should_block;
  return 0;
}

static errno_t stub_mq_pop(msg_queue_id_t id,
                           void **msg,
                           mq_blocking_mode_t should_block) {
  (void)id;
  (void)msg;
  (void)should_block;
  return 0;
}

static void *stub_mq_alloc(size_t size) {
  (void)size;
  return NULL;
}

/*******************************************************************************
 * Condition Variable
 ******************************************************************************/

/**
 * We make no effort here to reuse destroyed condition variables,
 * there is an upper bound on the number of condition variables which
 * may be created during a single execution, and that is that.
 */
static int stub_cv_init(size_t max_cv) {
  (void)max_cv;
  return PAL_INVALID;
}

static pal_cv_t stub_cv_alloc(void) { return NULL; }

static void stub_cv_free(pal_cv_t cv_loc) { (void)cv_loc; }

static void stub_cv_notify_one(pal_cv_t cv) { (void)cv; }

static void stub_cv_notify_all(pal_cv_t cv) { (void)cv; }

static void stub_cv_wait(pal_cv_t cv, pal_mutex_t lock) { (void)cv; }

static int stub_cv_wait_for(pal_cv_t cv, pal_mutex_t lock, uint32_t millis) {
  (void)cv;
  (void)millis;
  return PAL_INVALID;
}

/*******************************************************************************
 * Semaphore
 ******************************************************************************/

/**
 * We make no effort here to reuse destroyed semaphores,
 * there is an upper bound on the number of semaphores which
 * may be created during a single execution, and that is that.
 */
static platform_sem_t *stub_sem_create(void) { return NULL; }

static void stub_sem_destroy(platform_sem_t **sem_loc) { (void)sem_loc; }

static void stub_sem_signal(platform_sem_t *sem) { (void)sem; }

static int stub_sem_wait(platform_sem_t *sem) {
  (void)sem;
  return 0;
}

static int stub_sem_wait_timeout(platform_sem_t *sem, uint32_t millis) {
  (void)sem;
  (void)millis;
  return 0;
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void init_starling_platform_stub_implementation(void) {
  /* Mutex */
  struct pal_impl_mutex mutex_impl = {
      .init = stub_mutex_init,
      .alloc = stub_mutex_alloc,
      .free = stub_mutex_free,
      .lock = stub_mutex_lock,
      .unlock = stub_mutex_unlock,
  };
  pal_set_impl_mutex(&mutex_impl);
  /* Thread */
  struct pal_impl_thread thread_impl = {
      .create = stub_thread_create,
      .set_name = stub_thread_set_name,
      .join = stub_thread_join,
      .exit = stub_thread_exit,
  };
  pal_set_impl_thread(&thread_impl);
  /* Watchdog */
  platform_set_implementation_watchdog(
      stub_watchdog_notify_starling_main_thread);
  /* Queue */
  mq_impl_t mq_impl = {
      .mq_init = stub_mq_init,
      .mq_push = stub_mq_push,
      .mq_pop = stub_mq_pop,
      .mq_alloc = stub_mq_alloc,
  };
  platform_set_implementation_mq(&mq_impl);
  /* Semaphore */
  sem_impl_t sem_impl = {
      .sem_create = stub_sem_create,
      .sem_destroy = stub_sem_destroy,
      .sem_signal = stub_sem_signal,
      .sem_wait = stub_sem_wait,
      .sem_wait_timeout = stub_sem_wait_timeout,
  };
  platform_set_implementation_semaphore(&sem_impl);
  /* Condition Variable */
  cv_impl_t cv_impl = {
    .cv_init = stub_cv_init,
    .cv_alloc = stub_cv_alloc,
    .cv_free = stub_cv_free,
    .cv_notify_one = stub_cv_notify_one,
    .cv_notify_all = stub_cv_notify_all,
    .cv_wait = stub_cv_wait,
    .cv_wait_for = stub_cv_wait_for,
  };
  pal_set_impl_cv(&cv_impl);
}
