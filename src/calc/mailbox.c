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

#include <calc/mailbox.h>

#include <libswiftnav/logging.h>

#include <ch.h>
#include <stdint.h>

/**
 * Here we provide an implementation of Starling's required mailbox
 * interface for Piksi Multi. Some things to note:
 *
 * - There is a finite number of mailboxes.
 * - Mailboxes can hold messages up to a fixed capacity.
 * - Everything is handled with the ChibiOS primitives.
 * - All return codes are taken directly from ChibiOS (where 0 means success).
 */

/* Implementation assumptions. */
_Static_assert(MSG_OK == 0, "Mailbox Error: chibiOS does not return 0 for success.");

#define MAILBOX_MAX_COUNT 8 
#define MAILBOX_MAX_CAPACITY 150 

/* This is an implementation-specific timeout for the fetch with timeout
 * operation. */
#define MAILBOX_IMPLEMENTATION_TIMEOUT 5000

/* clang-format off */
struct mailbox_impl_t {
  mailbox_t chibios_mailbox;
  msg_t     chibios_msg_buffer[MAILBOX_MAX_CAPACITY];
};
/* clang-format on */

static mailbox_impl_t mailboxes[MAILBOX_MAX_COUNT];

static int num_mailboxes_in_use = 0;

/*******************************************************************************/
mailbox_impl_t *mailbox_init(size_t capacity) {
  if (capacity > MAILBOX_MAX_CAPACITY) {
    log_error("Mailbox Error: request for unsupported capacity.");
    return NULL;
  }
  if (num_mailboxes_in_use >= MAILBOX_MAX_COUNT) {
    log_error("Mailbox Error: no more mailboxes available.");
    return NULL;
  }
  chMBObjectInit(&mailboxes[num_mailboxes_in_use].chibios_mailbox,
                 mailboxes[num_mailboxes_in_use].chibios_msg_buffer,
                 capacity);
  return &mailboxes[num_mailboxes_in_use++];
}

/*******************************************************************************/
int mailbox_post_back(mailbox_impl_t *self, const void *msg) {
  return chMBPost(&self->chibios_mailbox, (msg_t)msg, TIME_IMMEDIATE);
}

/*******************************************************************************/
int mailbox_post_front(mailbox_impl_t *self, const void *msg) {
  return chMBPostAhead(&self->chibios_mailbox, (msg_t)msg, TIME_IMMEDIATE);
}

/*******************************************************************************/
int mailbox_fetch_immediate(mailbox_impl_t *self, void **msg) {
  return chMBFetch(&self->chibios_mailbox, (msg_t *)(*msg), TIME_IMMEDIATE);
}

/*******************************************************************************/
int mailbox_fetch_timeout(mailbox_impl_t *self, void **msg) {
  return chMBFetch(&self->chibios_mailbox, (msg_t *)(*msg), MAILBOX_IMPLEMENTATION_TIMEOUT);
}

/*******************************************************************************/
size_t mailbox_get_max_count(void) { return MAILBOX_MAX_COUNT; }

/*******************************************************************************/
size_t mailbox_get_max_capacity(void) { return MAILBOX_MAX_CAPACITY; }


