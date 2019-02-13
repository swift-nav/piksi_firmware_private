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

#ifndef STARLING_SBP_SETTINGS_CLIENT_H_
#define STARLING_SBP_SETTINGS_CLIENT_H_

#include <libsbp/sbp.h>
#include <libsettings/settings.h>

#include <stddef.h>

// TODO(kevin, jangelo) replace with the SBP refactor interface.
typedef struct SbpDuplexLink {
  /* Local sender id. */
  const uint16_t loc_sender_id;

  /* Forward sender id. */
  const uint16_t fwd_sender_id;

  /* Send message using local sender ID. */
  int (*send)          (uint16_t msg_type, 
                        uint8_t len, 
                        uint8_t *payload);

  /* Send message using given sender ID. */
  int (*send_from)     (uint16_t msg_type, 
                        uint8_t len, 
                        uint8_t *payload,
                        uint16_t sender);

  /* Register callback on incoming messages. */
  int (*register_cb)   (uint16_t msg_type, 
                        sbp_msg_callback_t cb, 
                        void *cb_context, 
                        sbp_msg_callbacks_node_t **node);

  /* Unregister callback on incoming messages. */
  int (*unregister_cb) (sbp_msg_callbacks_node_t **node);
} SbpDuplexLink;

typedef struct SbpSettingsClient SbpSettingsClient;

SbpSettingsClient *sbp_settings_client_create(const SbpDuplexLink *sbp_link);

#endif
