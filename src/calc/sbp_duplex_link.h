/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SBP_DUPLEX_LINK_H_
#define SBP_DUPLEX_LINK_H_

/* Rudimentary bidirectional SBP interface. To be replaced by
 * C++ version when possible. */

#include <libsbp/sbp.h>
#include <stddef.h>

// TODO(kevin, jangelo) replace with the SBP refactor interface.
typedef struct SbpDuplexLink {
  /* Local sender id. */
  uint16_t loc_sender_id;

  /* Forward sender id. */
  uint16_t fwd_sender_id;

  /* Send message using local sender ID. */
  int (*send)(uint16_t msg_type, uint8_t len, uint8_t *payload);

  /* Send message using given sender ID. */
  int (*send_from)(uint16_t msg_type,
                   uint8_t len,
                   uint8_t *payload,
                   uint16_t sender);

  /* Register callback on incoming messages. */
  int (*register_cb)(uint16_t msg_type,
                     sbp_msg_callback_t cb,
                     sbp_msg_callbacks_node_t *node,
                     void *cb_context);

  /* Unregister callback on incoming messages. */
  int (*unregister_cb)(sbp_msg_callbacks_node_t *node);
} SbpDuplexLink;

#endif
