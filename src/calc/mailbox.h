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

#ifndef MAILBOX_H
#define MAILBOX_H

/**
 * Mailbox
 * =======
 * Starling uses "mailboxes" to buffer and serialize its asynchronous
 * inputs and outputs. Any platform wishing to integrate the Starling
 * engine should provide a sensible implementation of the following
 * generic mailbox interface.
 *
 * Inherent in this interface is the assumption that the Starling engine
 * must run on constrained platforms -- you will notice that while there
 * is an initialization call, there is no matching "de-init" call. This is
 * intentional, and serves to encode the fact that the platform likely
 * provides a finite number of mailboxes, and they are not available
 * for dynamic use. Namely, mailboxes can be initialized only once,
 * after which they should be used for the remaining duration of the
 * program.
 *
 * Furthermore, it is assumed that in many cases, messages will be
 * passed by pointer. For this reason, the mailbox implementation will
 * often have some kind of object pool for message allocation. This
 * mechanism is exposed via the functions for allocating and freeing
 * message buffers.
 *
 * Return codes always use 0 to indicate success.
 */

#include <stddef.h>

typedef struct mailbox_impl_t mailbox_impl_t;

/**
 * Functions for querying the "finite-ness" of the implementation.
 * These should be provided so that the Starling engine can include
 * some sanity checking that the implementation will be sufficient.
 */
size_t mailbox_get_max_count(void);
size_t mailbox_get_max_capacity(void);

/**
 * Should return a pointer to the next available mailbox instance, or
 * NULL if there are no more available.
 *
 * The size of messages is required so that the appropriate underlying
 * object pool can be created. The capacity is the maximum number of 
 * messages allowed in the mailbox at any given time.
 *
 * Requests for a mailbox with message size of 0 will not allocate
 * an object pool.
 */
mailbox_impl_t *mailbox_init(size_t msg_size, size_t capacity);

/**
 * Allocate and free message buffers used in the implementation.
 * For large objects, we don't really want to perform unnecessary 
 * copies, so the mailbox will allow allocation of the messages
 */
void *mailbox_alloc_message(mailbox_impl_t *self);
void mailbox_free_message(mailbox_impl_t *self, void *msg);

/**
 * Post a message to the back or front of the mailbox queue.
 *
 * Returns 0 on success, implementation-specific return code on
 * failure.
 */
int mailbox_post_back(mailbox_impl_t *self, const void *msg);
int mailbox_post_front(mailbox_impl_t *self, const void *msg);

/**
 * Fetch the next message from the mailbox.
 *
 * Available in non-blocking (immediate) and blocking (timeout)
 * variants. Specificall, the timeout variant is used in place of 
 * the immediate version to communicate that the caller would like 
 * to pend on a message becoming available. It is up to the 
 * implementation to sleep/wait/return in a sensible manner.
 *
 * Returns 0 on success, implementation-specific return code on
 * failure or if there are no available messages.
 */
int mailbox_fetch_immediate(mailbox_impl_t *self, void **msg);
int mailbox_fetch_timeout(mailbox_impl_t *self, void **msg);

#endif
