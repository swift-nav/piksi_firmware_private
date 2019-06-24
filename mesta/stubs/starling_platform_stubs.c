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
#include <starling/platform/mutex.h>
#include <starling/platform/semaphore.h>
#include <starling/platform/thread.h>
#include <starling/platform/watchdog.h>

/*******************************************************************************
 * Mutex
 ******************************************************************************/

static int stub_mutex_init(mtx_id_t id) {
  (void)id;
  return 0;
}

static void stub_mutex_lock(mtx_id_t id) { (void)id; }

static void stub_mutex_unlock(mtx_id_t id) { (void)id; }

/*******************************************************************************
 * Thread
 ******************************************************************************/

static platform_thread_t *stub_thread_create(const thread_id_t id,
                                             platform_routine_t *fn) {
  (void)id;
  (void)fn;
  return NULL;
}

static void stub_thread_set_name(const platform_thread_t *thread,
                                 const char *name) {
  (void)name;
  (void)thread;
}

static void stub_thread_join(const platform_thread_t *thread) { (void)thread; }

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
  mutex_impl_t mutex_impl = {
      .mutex_init = stub_mutex_init,
      .mutex_lock = stub_mutex_lock,
      .mutex_unlock = stub_mutex_unlock,
  };
  platform_set_implementation_mutex(&mutex_impl);
  /* Thread */
  thread_impl_t thread_impl = {
      .thread_create = stub_thread_create,
      .thread_set_name = stub_thread_set_name,
      .thread_join = stub_thread_join,
  };
  platform_set_implementation_thread(&thread_impl);
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
}
