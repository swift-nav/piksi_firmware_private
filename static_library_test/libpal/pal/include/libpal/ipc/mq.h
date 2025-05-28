/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * This file defines the Message Queue system that each platform must implement
 * for Starling.
 *
 * The message queues are used for sending messages between threads in the Swift
 * applications, so they must be thread safe. It is usual for only one thread
 * will be pushing and only one thread will be popping, but that is not a
 * requirements. The implementation must be safe for multiple concurrent pushers
 * and poppers.
 *
 */

#ifndef LIBPAL_IPC_MQ_H
#define LIBPAL_IPC_MQ_H

#include <libpal/impl/ipc/mq.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PAL_MQ_DEFAULT_TIMEOUT_US 5000000

/**
 * Check global PAL message queue implementation
 *
 * @return true if the PAL MQ implementation has been set up, false otherwise
 */
bool pal_has_impl_mq(void);

/**
 * Allocate a PAL message queue
 *
 * Create/allocate a message queue which is able to hold up
 * to max_length elements
 *
 * @param mq On success will be set to a handle representing a newly allocated
 * message queue
 * @param max_length Maximum number of elements to be held in this queue
 * @return PAL error code
 */
enum pal_error pal_mq_alloc(pal_mq_t *mq, size_t max_length);

/**
 * Free a previously created message queue, release resource
 *
 * Once this function returns PAL_SUCCESS the caller's handle will be
 * automatically set to NULL, it must not be reused.
 *
 * @param mq Message Queue handle returned from pal_mq_alloc
 * @return PAL error code
 */
enum pal_error pal_mq_free(pal_mq_t *mq);

/**
 * Push a message on to the queue
 *
 * The message can be of any type and the memory must be allocated and
 * controlled by the caller. On success the message will be placed on to the
 * queue ready to be pop'd off in FIFO order. Memory management is the sole
 * responsibility of the user of this message queue, care must be taken to
 * ensure that dangling pointers are not left on the queue.
 *
 * When called in non-blocking mode this function will return PAL_WOULD_BLOCK
 * without taking any action if the queue is already full. In blocking mode this
 * function will not return until space becomes available in the queue to hold
 * the message. The timeout_us argument controls how long the function will wait
 * for space to become available before exiting without doing anything. The
 * special value of timeout_us == 0 means it will block indefinitely. If the
 * timeout expires this function returns PAL_TIMEOUT and the message will not
 * be pushed on to the queue.
 *
 * @param mq PAL message queue handle
 * @param msg Message pointer
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL Error code
 */
enum pal_error pal_mq_push(pal_mq_t mq, void *msg, enum pal_blocking_mode mode,
                           uint64_t timeout_us);

/**
 * Pop a message from the queue
 *
 * On success the message will be removed from the queue and placed in to the
 * msg output parameter.
 *
 * When called in non-blocking mode this function will return PAL_WOULD_BLOCK
 * without taking any action if the queue is empty. In blocking mode this
 * function will not return until something else has pushed a message on to the
 * queue. The timeout_us argument controls how long the function will wait for a
 * message to be pushed on to the queue before exiting without doing anything.
 * The special value of timeout_us == 0 means it will block indefinitely. If the
 * timeout expires this function returns PAL_TIMEOUT without updating the value
 * of *msg.
 *
 * @param mq PAL message queue handle
 * @param msg On success will be updated to point to the pop'd message
 * @param mode Blocking mode
 * @param timeout_us Timeout in microseconds
 * @return PAL error code
 */
enum pal_error pal_mq_pop(pal_mq_t mq, void **msg, enum pal_blocking_mode mode,
                          uint64_t timeout_us);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IPC_MQ_H
