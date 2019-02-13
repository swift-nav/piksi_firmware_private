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

/* NOTE: Implementation based off of PFWP/src/util/settings/settings_client.c 
 *
 * TODO(kevin, jangelo) 
 * We should audit the dynamic memory allocation in this module. Perhaps there
 * is a better way to enforce that either it is all performed once at startup,
 * or use an object pool to ensure limitations on the number of objects.
 */ 

#include "sbp_settings_client.h"

#include <starling/platform/semaphore.h>
#include <swiftnav/logging.h>

#include <assert.h>
#include <stdlib.h>

#define CLASS_PREFIX "Starling Settings Client: "

/********************************************************************************/
struct SbpSettingsClient {
  /* Underlying SBP link. */
  SbpDuplexLink sbp_link;

  /* Semaphore for used for waiting on replies. */
  platform_sem_t *sem;
  
  /* Settings context object. */
  settings_t *settings_context;
};

/********************************************************************************/
static int impl_send(void *ctx, uint16_t msg_type, uint8_t len, uint8_t *payload) {
  SbpSettingsClient *client = (SbpSettingsClient*)ctx;
  assert(client);
  assert(client->sbp_link.send);
  return client->sbp_link.send(msg_type, len, payload);
}

/********************************************************************************/
static int impl_send_from(void *ctx, 
                          uint16_t msg_type, 
                          uint8_t len, 
                          uint8_t *payload,
                          uint16_t sender) {
  SbpSettingsClient *client = (SbpSettingsClient*)ctx;
  assert(client);
  assert(client->sbp_link.send_from);
  return client->sbp_link.send_from(msg_type, len, payload, sender);
}

/********************************************************************************/
static int impl_wait(void *ctx, int timeout_ms) {
  SbpSettingsClient *client = (SbpSettingsClient*)ctx;
  assert(client);
  assert(client->sem);
  timeout_ms = timeout_ms > 0 ? timeout_ms : 0;
  return platform_sem_wait_timeout(client->sem, (unsigned long)timeout_ms);
}

/********************************************************************************/
static void impl_signal(void *ctx) {
  SbpSettingsClient *client = (SbpSettingsClient*)ctx;
  assert(client);
  assert(client->sem);
  platform_sem_signal(client->sem);
}

/********************************************************************************/
static int impl_register_cb(void *ctx,
                            uint16_t msg_type,
                            sbp_msg_callback_t cb,
                            void *cb_context,
                            sbp_msg_callbacks_node_t **node) {
  SbpSettingsClient *client = (SbpSettingsClient*)ctx;
  assert(client);
  assert(client->sbp_link.register_cb);
  assert(node);

  /* Apparently, we are supposed to allocate the node ourselves here?
   *
   * TODO(kevin, jangelo) 
   * Audit this allocation and do one of:
   *   A. make sure it only occurs at runtime
   *   B. replace with an object pool
   */
  *node = malloc(sizeof(sbp_msg_callbacks_node_t));
  if (!*node) {
    log_error(CLASS_PREFIX "unable to alloc callback node");
    return -1;
  }

  int ret = client->sbp_link.register_cb(msg_type, cb, *node, cb_context);
  if (ret != 0) {
    log_error(CLASS_PREFIX "unable to register callback node");
  }
  return ret;
}

/********************************************************************************/
static int impl_unregister_cb(void *ctx, sbp_msg_callbacks_node_t **node) {
  SbpSettingsClient *client = (SbpSettingsClient*)ctx;
  assert(client);
  assert(client->sbp_link.unregister_cb);
  assert(node);

  int ret = client->sbp_link.unregister_cb(*node);
  if (ret == 0) {
    free(*node);
  } else {
    log_error(CLASS_PREFIX "unable to unregister callback node");
  }
  return ret;
}


/********************************************************************************/
static void impl_log(int priority, const char *fmt, ...) {
  log_warn("Trying to log from libsettings: %d, %s", priority, fmt);
}

/********************************************************************************/
static settings_api_t settings_api_for_client(SbpSettingsClient *client) {
  settings_api_t impl = {
    .ctx = client,
    .send = impl_send,
    .send_from = impl_send_from,
    .wait_init = NULL,
    .wait = impl_wait,
    .wait_deinit = NULL,
    .signal = impl_signal,
    .register_cb = impl_register_cb,
    .unregister_cb = impl_unregister_cb,
    .log = impl_log,
  };
  return impl;
}

/********************************************************************************/
SbpSettingsClient *sbp_settings_client_create(const SbpDuplexLink *sbp_link) {
  SbpSettingsClient *client = malloc(sizeof(SbpSettingsClient));
  if (!client) {
    return NULL;
  }

  assert(sbp_link);
  client->sbp_link = *sbp_link;

  settings_api_t impl = settings_api_for_client(client);
  client->settings_context = settings_create(client->sbp_link.loc_sender_id, &impl);  
  if (!client->settings_context) {
    log_error(CLASS_PREFIX "unable to create settings context");
    goto FREE_AND_RETURN_NULL;
  }

  client->sem = platform_sem_create();
  if (!client->sem) {
    log_error(CLASS_PREFIX "unable to create semaphore");
    goto FREE_AND_RETURN_NULL;
  }
  
  return client;

/* Cleanup and return on error. */
FREE_AND_RETURN_NULL:
  if (client->sem) {
    platform_sem_destroy(&client->sem);
  }
  if (client->settings_context) {
    settings_destroy(&client->settings_context);
  }
  free(client);
  return NULL;
};

/********************************************************************************/
int sbp_settings_client_register_enum(SbpSettingsClient *client,
                                      const char *const enum_names[], 
                                      settings_type_t *type) {
  return settings_register_enum(client->settings_context, enum_names, type);
}

/********************************************************************************/
int sbp_settings_client_register(SbpSettingsClient *client,
                                 const char *section,
                                 const char *name,
                                 void *var,
                                 size_t var_len,
                                 settings_type_t type,
                                 settings_notify_fn notify,
                                 void *notify_context) {
  return settings_register_setting(client->settings_context, section, name, 
                                   var, var_len, type, notify, notify_context);
}
