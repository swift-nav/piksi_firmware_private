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

#include <libswiftnav/logging.h>
#include <ch.h>

/******************************************************************************
 * GENERAL
 *****************************************************************************/
static bool is_platform_initialized = false;

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
  log_error("Error in Starling PAL implementation: " PRIi32, (int32_t)code);
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
}

/******************************************************************************
 * THREAD
 *****************************************************************************/

/**
 * For Piksi Multi, we support a maximum of two auxiliary threads.
 */
static_assert(STARLING_MAX_NUM_THREADS <= 2);


pal_rc_t pal_thread_run_tasks(pal_thread_task_t tasks[STARLING_MAX_NUM_THREADS]) {

}

/******************************************************************************
 * QUEUE 
 *****************************************************************************/

/**
 * A fundamental assumption of this implementation is that a pointer 
 * is exactly the size of a ChibiOS mailbox message.
 */
static_assert(sizeof(msg_t) == sizeof(void*));

/* Everything needed to represent the state of a platform mailbox. */
struct mailbox_object_t {
  bool      is_initialized;
  mailbox_t chibios_mailbox;
  msg_t     chibios_msg_buffer[STARLING_MAX_MAILBOX_CAPACITY];
};

/* Collection of mailbox state objects used in this implementation. */
static mailbox_object_t mailbox_objects[STARLING_MAX_NUM_MAILBOXES];

/* Helper function to check that a mailbox id is in range. */
static bool is_valid_mailbox_id(mailbox_id_t id) {
  return mbid < STARLING_MAX_NUM_MAILBOXES;
}

/* Helper function for getting the mailbox pointer from an id. */
static mailbox_object_t *get_mailbox_for_id(mailbox_id_t id) {
  return &mailbox_objects[id];
}

/**
 */
pal_rc_t pal_mailbox_init(mailbox_id_t id, const size_t capacity) {
  assert(is_valid_mailbox_id(id));

  mailbox_object_t *mb = get_mailbox_for_id(id);

  /* Error to request capacity larger than max value. */
  if (STARLING_MAX_QUEUE_CAPACITY < capacity) {
    return -1;
  }

  /* Error to initialize an already initialized queue. */
  if (mb->is_initialized) {
    return -1;
  }

  chMBObjectInit(&mb->chibios_mailbox, &chibios_msg_buffer, capacity);
  mb->is_initialized = true;

  return STARLING_PAL_OK;
}

/**
 */
pal_rc_t pal_mailbox_post(mailbox_id_t id, const void *p) {
  assert(is_valid_mailbox_id(id));
  mailbox_object_t *mb = get_mailbox_for_id(id);
  msg_t = chMBPost(mb, (msg_t)p, TIME_IMMEDIATE);
  if (MSG_OK != msg_t) {
    // TODO(kevin) report errors.
    return -1;
  }
  return STARLING_PAL_OK;
}

/**
 */
pal_rc_t pal_mailbox_fetch(mailbox_id_t id, void **p, uint32_t timeout_ms) {
  assert(is_valid_mailbox_id(id));
  assert(p);
  mailbox_object_t *mb = get_mailbox_for_id(id);
  msg_t = chMBFetch(mb, (msg_t*)p, (systime_t)timeout_ms);
  if (MSG_OK != msg_t) {
    *p = NULL;
    return -1;
  }
  return STARLING_PAL_OK;
}

/******************************************************************************
 * ALLOCATOR
 *****************************************************************************/

// STARTUP_ONLY
pal_rc_t pal_allocator_register(const size_t block_size, const size_t n_blocks) {
  return -1;
}

pal_rc_t pal_allocator_alloc(const size_t block_size, void **p) {
  return -1;
}

pal_rc_t pal_allocator_free(void *p) {
  return -1;
}


