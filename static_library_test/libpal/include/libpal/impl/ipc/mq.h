/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBPAL_IMPL_IPC_MQ_H
#define LIBPAL_IMPL_IPC_MQ_H

#include <libpal/ipc/mq.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * PAL Message Queue Implementation
 *
 * This file defines the interface a PAL implementation must use to install its
 * own message queue ability in to the libpal API
 *
 * The function pointer names and signatures in this file match those in
 * libpal/ipc/mq.h. The PAL implementation must provide a version of these
 * functions which meet the requirements stated in the documentation contained
 * in that file.
 */

/**
 * Allocate a message queue
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mq_alloc() (see libpal/ipc/mq.h)
 *
 */
typedef enum pal_error (*pal_mq_alloc_t)(size_t max_length, pal_mq_t *mq);

/**
 * Free a previously allocated message queue
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mq_free() (see libpal/ipc/mq.h)
 *
 */
typedef enum pal_error (*pal_mq_free_t)(pal_mq_t mq);

/**
 * Push a message on to a queue
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mq_push() (see libpal/ipc/mq.h)
 *
 */
typedef enum pal_error (*pal_mq_push_t)(pal_mq_t mq, void *msg,
                                        enum pal_mq_blocking_mode mode,
                                        uint64_t timeout_us);

/**
 * Pop a message on to a queue
 *
 * A PAL implementation must define this function according to the requirements
 * of pal_mq_pop() (see libpal/ipc/mq.h)
 *
 */
typedef enum pal_error (*pal_mq_pop_t)(pal_mq_t mq, void **msg,
                                       enum pal_mq_blocking_mode mode,
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
 */
void pal_set_impl_mq(struct pal_impl_mq *impl);

#ifdef __cplusplus
}
#endif

#endif
