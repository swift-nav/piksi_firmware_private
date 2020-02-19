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

#include <libpal/impl/ipc/mq.h>
#include <libpal/impl/synch/condition_var.h>
#include <libpal/impl/synch/mutex.h>
#include <libpal/impl/thread/thread.h>
#include <libpal/impl/watchdog/watchdog.h>
#include <libpal/pal.h>

/*******************************************************************************
 * Mutex
 ******************************************************************************/

static enum pal_error stub_mutex_alloc(pal_mutex_t *mutex) {
  return PAL_INVALID;
}

static enum pal_error stub_mutex_free(pal_mutex_t mutex) {
  (void)mutex;
  return PAL_INVALID;
}

static enum pal_error stub_mutex_lock(pal_mutex_t mutex) {
  (void)mutex;
  return PAL_INVALID;
}

static enum pal_error stub_mutex_unlock(pal_mutex_t mutex) {
  (void)mutex;
  return PAL_INVALID;
}

/*******************************************************************************
 * Thread
 ******************************************************************************/

static enum pal_error stub_thread_create(pal_thread_t *thread,
                                         pal_thread_entry_t fn,
                                         void *ctx,
                                         size_t stacksize,
                                         uint8_t prio) {
  (void)thread;
  (void)fn;
  (void)ctx;
  (void)stacksize;
  (void)prio;
  return PAL_INVALID;
}

static enum pal_error stub_thread_set_name(const char *name) {
  (void)name;
  return PAL_INVALID;
}

static enum pal_error stub_thread_join(pal_thread_t thread, void **retval) {
  (void)thread;
  (void)retval;
  return PAL_INVALID;
}

static void stub_thread_exit(void *code) { (void)code; }

static enum pal_error stub_thread_interrupt(pal_thread_t thread) {
  (void)thread;
  return PAL_INVALID;
}

/*******************************************************************************
 * Watchdog
 ******************************************************************************/

static enum pal_error stub_watchdog_notify_starling_main_thread(void) {
  return PAL_INVALID;
}

/*******************************************************************************
 * Queue
 ******************************************************************************/

static enum pal_error stub_mq_alloc(size_t max_length, pal_mq_t *mq) {
  (void)max_length;
  return PAL_INVALID;
}

static enum pal_error stub_mq_free(pal_mq_t mq) {
  (void)mq;
  return PAL_INVALID;
}

static enum pal_error stub_mq_push(pal_mq_t mq,
                                   void *msg,
                                   enum pal_mq_blocking_mode mode,
                                   uint64_t timeout_us) {
  (void)mq;
  (void)msg;
  (void)mode;
  (void)timeout_us;
  return PAL_INVALID;
}

static enum pal_error stub_mq_pop(pal_mq_t mq,
                                  void **msg,
                                  enum pal_mq_blocking_mode mode,
                                  uint64_t timeout_us) {
  (void)mq;
  (void)msg;
  (void)mode;
  (void)timeout_us;
  return PAL_INVALID;
}

/*******************************************************************************
 * Condition Variable
 ******************************************************************************/

static enum pal_error stub_cv_alloc(pal_cv_t *cv) {
  (void)cv;
  return PAL_INVALID;
}

static enum pal_error stub_cv_free(pal_cv_t cv_loc) {
  (void)cv_loc;
  return PAL_INVALID;
}

static enum pal_error stub_cv_notify_one(pal_cv_t cv) {
  (void)cv;
  return PAL_INVALID;
}

static enum pal_error stub_cv_notify_all(pal_cv_t cv) {
  (void)cv;
  return PAL_INVALID;
}

static enum pal_error stub_cv_wait(pal_cv_t cv, pal_mutex_t lock) {
  (void)cv;
  return PAL_INVALID;
}

static enum pal_error stub_cv_wait_for(pal_cv_t cv,
                                       pal_mutex_t lock,
                                       uint64_t timeout_us) {
  (void)cv;
  (void)timeout_us;
  return PAL_INVALID;
}

/*******************************************************************************
 * Initialization
 ******************************************************************************/

void pal_impl_init(void) {
  /* Mutex */
  struct pal_impl_mutex mutex_impl = {
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
      .interrupt = stub_thread_interrupt,
  };
  pal_set_impl_thread(&thread_impl);
  /* Watchdog */
  struct pal_impl_watchdog watchdog_impl = {
      .notify = stub_watchdog_notify_starling_main_thread,
  };
  pal_set_impl_watchdog(&watchdog_impl);
  /* Queue */
  struct pal_impl_mq mq_impl = {
      .alloc = stub_mq_alloc,
      .free = stub_mq_free,
      .push = stub_mq_push,
      .pop = stub_mq_pop,
  };
  pal_set_impl_mq(&mq_impl);
  /* Condition Variable */
  struct pal_impl_cv cv_impl = {
      .alloc = stub_cv_alloc,
      .free = stub_cv_free,
      .notify_one = stub_cv_notify_one,
      .notify_all = stub_cv_notify_all,
      .wait = stub_cv_wait,
      .wait_for = stub_cv_wait_for,
  };
  pal_set_impl_cv(&cv_impl);
}

void pal_impl_deinit(void) {  // Nothing to do
}
