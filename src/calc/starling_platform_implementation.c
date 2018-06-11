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

#include "starling_platform.h"

#include "system_monitor/system_monitor.h"

#include <libswiftnav/logging.h>

#include <assert.h>
#include <ch.h>
#include <inttypes.h>
#include <stdlib.h>

/******************************************************************************
 * GENERAL
 *****************************************************************************/
static bool is_platform_initialized = false;

void pal_watchdog_kick(void) { watchdog_notify(WD_NOTIFY_STARLING); }

/**
 * Figure out which subsystem of the PAL this error came from and
 * then print an appropriate message.
 */
void pal_handle_error(pal_rc_t code) {
  if (STARLING_PAL_OK == code) {
    return;
  }

  /**
   * TODO(kevin) It may make sense to also encode the source of the
   * error into the actual return codes so that the handle error
   * function can reconstruct the information.
   */
  log_error("Error in Starling PAL implementation.");
}

/**
 * On the Piksi Multi, all data structures are statically allocated and all
 * we need to do is register them with the OS.
 */
pal_rc_t pal_init(void) {
  if (is_platform_initialized) {
    // TODO(kevin) is this an error?
    return STARLING_PAL_OK;
  }

  is_platform_initialized = true;

  return STARLING_PAL_OK;
}

/******************************************************************************
 * THREAD
 *****************************************************************************/

#define PIKSI_MULTI_NUM_STARLING_THREADS 2
#define PIKSI_MULTI_STARLING_THREAD_WORKING_AREA_SIZE 0x600000
#define PIKSI_MULTI_STARLING_THREAD_PRIO_REALTIME (HIGHPRIO - 4)
#define PIKSI_MULTI_STARLING_THREAD_PRIO_BACKGROUND (NORMALPRIO - 3)

#define PIKSI_MULTI_PAL_THREAD_RUN_FAILURE -1

/**
 * For Piksi Multi, we support a maximum of two auxiliary threads.
 */
_Static_assert(PIKSI_MULTI_NUM_STARLING_THREADS >= STARLING_MAX_NUM_THREADS,
               "Piksi Multi only has support for 2 Starling threads.");

/**
 * Ensure that the allocated thread stacks are large enough.
 */
_Static_assert(PIKSI_MULTI_STARLING_THREAD_WORKING_AREA_SIZE >=
                   STARLING_MAX_THREAD_STACK,
               "Piksi Multi thread stack is too small for Starling.");

static THD_WORKING_AREA(wa_thread_1,
                        PIKSI_MULTI_STARLING_THREAD_WORKING_AREA_SIZE);
static THD_WORKING_AREA(wa_thread_2,
                        PIKSI_MULTI_STARLING_THREAD_WORKING_AREA_SIZE);

static void *const working_areas[] = {wa_thread_1, wa_thread_2};

/**
 * Helper function to validate that a task has acceptable
 * values. Assumes a non-null task.
 */
static bool is_task_valid(pal_thread_task_t *task) {
  /* Priority must have a valid value. */
  if (STARLING_PAL_PRIORITY_REALTIME != task->priority &&
      STARLING_PAL_PRIORITY_BACKGROUND != task->priority) {
    return false;
  }
  /* Task function must exist. */
  if (NULL == task->fn) {
    return false;
  }
  return true;
}

/**
 * Helper function to convert the PAL API thread priority level
 * to a meaningful OS priority level.
 */
static tprio_t convert_priority(pal_priority_t priority) {
  switch (priority) {
    case STARLING_PAL_PRIORITY_REALTIME:
      return PIKSI_MULTI_STARLING_THREAD_PRIO_REALTIME;
    case STARLING_PAL_PRIORITY_BACKGROUND:
      return PIKSI_MULTI_STARLING_THREAD_PRIO_BACKGROUND;
    default:
      return PIKSI_MULTI_STARLING_THREAD_PRIO_BACKGROUND;
  }
}

/**
 * Thread wrapper function which applies the thread name before invoking
 * the user function.
 */
static void apply_name_and_run_task(void *context) {
  const pal_thread_task_t *task = (pal_thread_task_t *)context;
  if (task->name) {
    chRegSetThreadName(task->name);
  } else {
    chRegSetThreadName("starling anonymous thread");
  }
  /* Invoke the user task. */
  task->fn(task->context);
}

/**
 * Iterate through the array of tasks and spawn each on a separate thread.
 */
pal_rc_t pal_run_thread_tasks(
    pal_thread_task_t *const tasks[STARLING_MAX_NUM_THREADS]) {
  for (int i = 0; i < STARLING_MAX_NUM_THREADS; ++i) {
    pal_thread_task_t *const task = tasks[i];
    if (task) {
      thread_t *thread = NULL;
      if (is_task_valid(task)) {
        /* Convert to appropriate priority and dispatch. */
        thread =
            chThdCreateStatic(working_areas[i],
                              PIKSI_MULTI_STARLING_THREAD_WORKING_AREA_SIZE,
                              convert_priority(task->priority),
                              apply_name_and_run_task,
                              task);
      }
      /* Stop early if something went wrong. */
      if (!thread) {
        return PIKSI_MULTI_PAL_THREAD_RUN_FAILURE;
      }
    }
  }
  return STARLING_PAL_OK;
}

/******************************************************************************
 * MUTEX 
 *****************************************************************************/

#define PIKSI_MULTI_MAX_NUM_MUTEXES 16

_Static_assert(STARLING_MAX_NUM_MUTEXES <= PIKSI_MULTI_MAX_NUM_MUTEXES,
               "Piksi Multi cannot provide required number of mutexes.");

typedef struct mutex_object_t {
  bool is_initialized;
  mutex_t chibios_mutex;
} mutex_object_t;

static mutex_object_t mutex_objects[STARLING_MAX_NUM_MUTEXES];

/* Helper function to check that a mutex id is in range. */
static bool is_valid_mutex_id(mutex_id_t id) {
  return (id < STARLING_MAX_NUM_MUTEXES);
}

/* Helper function for getting the mutex pointer from an id. */
static mutex_object_t *get_mutex_for_id(mutex_id_t id) {
  return &mutex_objects[id];
}

void pal_mutex_init(mutex_id_t id) {
  assert(is_valid_mutex_id(id));
  mutex_object_t *mtx = get_mutex_for_id(id);
  assert(!mtx->is_initialized);
  chMtxObjectInit(&mtx->chibios_mutex);
  mtx->is_initialized = true;
}

void pal_mutex_lock(mutex_id_t id) {
  assert(is_valid_mutex_id(id));
  mutex_object_t *mtx = get_mutex_for_id(id);
  assert(mtx->is_initialized);
  chMtxLock(&mtx->chibios_mutex);
}

void pal_mutex_unlock(mutex_id_t id) {
  assert(is_valid_mutex_id(id));
  mutex_object_t *mtx = get_mutex_for_id(id);
  assert(mtx->is_initialized);
  chMtxUnlock(&mtx->chibios_mutex);
}

/******************************************************************************
 * MAILBOX
 *****************************************************************************/

#define PIKSI_MULTI_MAX_NUM_MAILBOXES 16
#define PIKSI_MULTI_MAX_MAILBOX_CAPACITY 6

_Static_assert(STARLING_MAX_NUM_MAILBOXES <= PIKSI_MULTI_MAX_NUM_MAILBOXES,
               "Piksi Multi cannot provide required number of mailboxes.");


_Static_assert(STARLING_MAX_NUM_MAILBOXES <= PIKSI_MULTI_MAX_NUM_MAILBOXES,
               "Piksi Multi cannot provide required mailbox capacity.");

/**
 * Timeout fetch operations on mailboxes will return after
 * a certain number of ticks if nothing is there in
 * this implementation.
 */
#define PIKSI_MULTI_MAILBOX_TIMEOUT 5000

/**
 * A fundamental assumption of this implementation is that a pointer
 * is exactly the size of a ChibiOS mailbox message.
 */
_Static_assert(sizeof(msg_t) == sizeof(void *),
               "Mailbox message type insufficient for pointer passing.");

/* Everything needed to represent the state of a platform mailbox. */
typedef struct mailbox_object_t {
  bool is_initialized;
  mailbox_t chibios_mailbox;
  msg_t chibios_msg_buffer[STARLING_MAX_MAILBOX_CAPACITY];
  memory_pool_t chibios_mem_pool;
  void *chibios_mem_pool_buffer;
} mailbox_object_t;

/* Collection of mailbox state objects used in this implementation. */
static mailbox_object_t mailbox_objects[STARLING_MAX_NUM_MAILBOXES];

/* Helper function to check that a mailbox id is in range. */
static bool is_valid_mailbox_id(mailbox_id_t id) {
  return (id < STARLING_MAX_NUM_MAILBOXES);
}

/* Helper function for getting the mailbox pointer from an id. */
static mailbox_object_t *get_mailbox_for_id(mailbox_id_t id) {
  return &mailbox_objects[id];
}

/**
 */
static pal_rc_t mailbox_fetch(mailbox_id_t id, void **p, uint32_t timeout_ms) {
  assert(is_valid_mailbox_id(id));
  assert(p);
  mailbox_object_t *mb = get_mailbox_for_id(id);
  assert(mb->is_initialized);
  msg_t result =
      chMBFetch(&mb->chibios_mailbox, (msg_t *)p, (systime_t)timeout_ms);
  if (MSG_OK != result) {
    *p = NULL;
    return -1;
  }
  return STARLING_PAL_OK;
}

/**
 * Initializing a mailbox is a little bit complicated. Because messages are passed
 * by pointer, a mailbox has also an accompanying object pool with which messages 
 * are allocated and freed.
 */
pal_rc_t pal_mailbox_init(mailbox_id_t id, size_t msg_size, size_t capacity) {
  assert(is_valid_mailbox_id(id));
  mailbox_object_t *mb = get_mailbox_for_id(id);

  /* Error to request capacity larger than max value. */
  if (STARLING_MAX_MAILBOX_CAPACITY < capacity) {
    return -1;
  }

  /* Error to initialize an already initialized mailbox. */
  if (mb->is_initialized) {
    return -1;
  }

  /* Attempt to create an appropriately sized memory region for message
   * allocations. */
  mb->chibios_mem_pool_buffer = malloc(msg_size * capacity);
  if (!mb->chibios_mem_pool_buffer) {
    return -1;
  }

  /* Initialize the mailbox and it's corresponding message object pool. */
  chMBObjectInit(&mb->chibios_mailbox, mb->chibios_msg_buffer, capacity);
  chPoolObjectInit(&mb->chibios_mem_pool, msg_size, NULL);
  chPoolLoadArray(&mb->chibios_mem_pool, mb->chibios_mem_pool_buffer, capacity);

  mb->is_initialized = true;
  return STARLING_PAL_OK;
}

/**
 */
pal_rc_t pal_mailbox_post(mailbox_id_t id, const void *p) {
  assert(is_valid_mailbox_id(id));
  mailbox_object_t *mb = get_mailbox_for_id(id);
  assert(mb->is_initialized);
  msg_t result = chMBPost(&mb->chibios_mailbox, (msg_t)p, TIME_IMMEDIATE);
  if (MSG_OK != result) {
    // TODO(kevin) report errors.
    return -1;
  }
  return STARLING_PAL_OK;
}

pal_rc_t pal_mailbox_fetch_immediate(mailbox_id_t id, void **p) {
  return mailbox_fetch(id, p, TIME_IMMEDIATE);
}

pal_rc_t pal_mailbox_fetch_timeout(mailbox_id_t id, void **p) {
  return mailbox_fetch(id, p, PIKSI_MULTI_MAILBOX_TIMEOUT);
}

