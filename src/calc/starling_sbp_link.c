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

#include "starling_sbp_link.h"

#include "cfg/init.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"

#include <libsbp/sbp.h>
#include <swiftnav/logging.h>

#include <assert.h>
#include <stdlib.h>

/******************************************************************************/
SbpDuplexLink *sbp_link = NULL;

/******************************************************************************/
static int impl_sbp_send(uint16_t msg_type, uint8_t len, uint8_t *payload) {
  return (int)sbp_send_msg(msg_type, len, payload);
}

/******************************************************************************/
static int impl_sbp_send_from(uint16_t msg_type,
                              uint8_t len,
                              uint8_t *payload,
                              uint16_t sender) {
  return (int)sbp_send_msg_(msg_type, len, payload, sender);
}

/******************************************************************************/
static int impl_sbp_register_cb(uint16_t msg_type,
                                sbp_msg_callback_t cb,
                                sbp_msg_callbacks_node_t *node,
                                void *context) {
  sbp_register_cbk_with_closure(msg_type, cb, node, context);
  return 0;
}

/******************************************************************************/
static int impl_sbp_unregister_cb(sbp_msg_callbacks_node_t *node) {
  sbp_remove_cbk(node);
  return 0;
}

/******************************************************************************/
void starling_sbp_link_setup(void) {
  if (sbp_link) {
    log_error("unexpected attempt to reinitialize starling sbp link");
    return;
  }

  sbp_link = malloc(sizeof(*sbp_link));
  if (!sbp_link) {
    log_error("failed to alloc SBP link");
  }

  SbpDuplexLink implemented_sbp_link = {
      .loc_sender_id = sender_id_get(),
      .fwd_sender_id = MSG_FORWARD_SENDER_ID,
      .send = impl_sbp_send,
      .send_from = impl_sbp_send_from,
      .register_cb = impl_sbp_register_cb,
      .unregister_cb = impl_sbp_unregister_cb,
  };

  *sbp_link = implemented_sbp_link;
}
