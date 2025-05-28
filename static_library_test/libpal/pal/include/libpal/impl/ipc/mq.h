/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_IPC_MQ_H
#define LIBPAL_IMPL_IPC_MQ_H

#include <stddef.h>
#include <stdint.h>

#include <libpal/error.h>
#include <libpal/impl/blocking_mode.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL Message Queue Context
 *
 * pal_mq_t holds only pointers to messages. The memory for the messages
 * themselves shall be managed separately.
 */
typedef void *pal_mq_t;

/**
 * Allocate a message queue
 *
 * This function must create and initialize a new message queue capable of
 * storing up to \p max_length items of type void*
 *
 * If successful this function must create a new MQ device handle and store it
 * in the value pointed at by \p mq. This handle will be passed in to later
 * calls to pal_mq_push/pal_mq_pop and eventually pal_mq_free
 *
 * @param mq On success must be updated to a new message queue handle
 * @param max_length Maximum number of items to store on the new message queue
 * @return PAL error code
 */
typedef enum pal_error (*pal_mq_alloc_t)(pal_mq_t *mq, size_t max_length);

/**
 * Free a previously allocated message queue
 *
 * The handle provided to this function will be a handle previously returned
 * from a call to pal_mq_alloc. When this function is called the platform
 * implementation must release any resources associated with the message queue
 * and destroy the handle
 *
 * @param mq Message queue handle
 * @return PAL error code
 */
typedef enum pal_error (*pal_mq_free_t)(pal_mq_t mq);

/**
 * Push a message on to a queue
 *
 * Starling will call this function to push a new message on to a message queue.
 * Messages are always of type void*.
 *
 * The \p mode parameter controls the blocking behaviour of this function. When
 * set to PAL_NONBLOCKING this function must immediately return PAL_WOULD_BLOCK
 * without taking any other action if the message queue is already full and not
 * able to accept any more messages.
 *
 * When set to PAL_BLOCKING this function may block the caller for up to the
 * microsecond duration given in the \p timeout_us parameter. Should space on
 * the message queue become available before the timeout period has expires this
 * function must push the message on to the queue and return PAL_SUCCESS. If the
 * timeout period expires before space becomes available this function must
 * return PAL_TIMEOUT without taking any other action.
 *
 * The sepcial timeout value of 0 means block indefinitely
 *
 * @param mq Message queue handle
 * @param msg Message to push
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration
 * @return PAL error code
 */
typedef enum pal_error (*pal_mq_push_t)(pal_mq_t mq, void *msg,
                                        enum pal_blocking_mode mode,
                                        uint64_t timeout_us);

/**
 * Pop a message on to a queue
 *
 * Starling will call this function to pop a message off a message queue.
 *
 * On success this function must update the \p msg parameter to point to the
 * popped message.
 *
 * The \p mode parameter controls the blocking behaviour of this function. When
 * set to PAL_NONBLOCKING this function must immediately return PAL_WOULD_BLOCK
 * without taking any other action is the message queue is empty and there are
 * no message waiting to be popped
 *
 * When set to PAL_BLOCKING this function may block the caller for up to the
 * microsecond duration given in the \p timeout_us parameter. Should a message
 * be pushed on to the queue and become available before the timeout period
 * expires this function must pop the message and return PAL_SUCCESS. If the
 * timeout period expires before a message becomes available this function must
 * return PAL_TIMEOUT without taking any other action or updating any other
 * parameters.
 *
 * The special timeout value of 0 means block indefinitely.
 *
 * @param mq Message queue handle
 * @param msg On success must be updated with the popped message
 * @param mode Blocking mode
 * @param timeout_us Maximum blocking duration
 * @return PAL error code;
 */
typedef enum pal_error (*pal_mq_pop_t)(pal_mq_t mq, void **msg,
                                       enum pal_blocking_mode mode,
                                       uint64_t timeout_us);

/**
 * PAL Message Queue implementation definition
 */
struct pal_impl_mq {
  /// Implementation message queue allocate routine
  pal_mq_alloc_t alloc;
  /// Implementation message queue free routine
  pal_mq_free_t free;
  /// Implementation message queue push routine
  pal_mq_push_t push;
  /// Implementation message queue pop routine
  pal_mq_pop_t pop;
};

/**
 * Install PAL message queue implementation in to API
 *
 * Call this function during pal_impl_init to register the implementation's
 * message queue module with the libpal API
 *
 * @param impl Message queue implementation definition
 * @return PAL error code
 */
enum pal_error pal_set_impl_mq(struct pal_impl_mq *impl);

#ifdef __cplusplus
}
#endif

#endif
