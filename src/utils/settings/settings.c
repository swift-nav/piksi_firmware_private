/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <stdlib.h> 
#include <string.h>

#include <ch.h>

#include <libsbp/settings.h>
#include <swiftnav/logging.h>

#include "sbp.h"
#include "settings/settings.h"

static setreg_t *setreg = NULL;

typedef struct settings_ctx_s {
  binary_semaphore_t sem;
} settings_ctx_t;

static settings_ctx_t settings_api_ctx = {0};

static int send_wrap(void *ctx, uint16_t msg_type, uint8_t len, uint8_t *payload)
{
  (void)ctx;
  return sbp_send_msg(msg_type, len, payload);
}

static int send_from_wrap(void *ctx, uint16_t msg_type, uint8_t len, uint8_t *payload, uint16_t sbp_sender_id)
{
  (void)ctx;
  return sbp_send_msg_(msg_type, len, payload, sbp_sender_id);
}

static int wait_init_wrap(void *ctx)
{
  settings_ctx_t *settings_ctx = (settings_ctx_t *)ctx;

  /* Take semaphore */
  chBSemReset(&settings_ctx->sem, true);

  return 0;
}

static int wait_wrap(void *ctx, int timeout_ms)
{
  settings_ctx_t *settings_ctx = (settings_ctx_t *)ctx;

  int ret = 0;

  if (chBSemWaitTimeout(&settings_ctx->sem, MS2ST(timeout_ms)) != MSG_OK) {
    ret = 1;
  }

  return ret;
}

static int wait_deinit_wrap(void *ctx)
{
  settings_ctx_t *settings_ctx = (settings_ctx_t *)ctx;

  /* Give semaphore */
  chBSemReset(&settings_ctx->sem, false);

  return 0;
}

static void signal_wrap(void *ctx)
{
  settings_ctx_t *settings_ctx = (settings_ctx_t *)ctx;
  chBSemSignal(&settings_ctx->sem);
}

static int reg_cb_wrap(void *ctx,
                       uint16_t msg_type,
                       sbp_msg_callback_t cb,
                       void *cb_context,
                       sbp_msg_callbacks_node_t **node)
{
  (void)ctx;
  assert(NULL != cb);
  assert(NULL != node);

  sbp_msg_callbacks_node_t *n = (sbp_msg_callbacks_node_t *)malloc(sizeof(*n));
  if (NULL == n) {
    log_error("error allocating callback node");
    return -1;
  }

  *node = n;

  sbp_register_cbk_with_closure(msg_type, cb, *node, cb_context);

  return 0;
}

static int unreg_cb_wrap(void *ctx, sbp_msg_callbacks_node_t **node)
{
  (void)ctx;
  assert(NULL != node);
  assert(NULL != *node);

  sbp_remove_cbk(*node);

  free(*node);
  *node = NULL;

  return 0;
}

void settings_setup(void) {
  chBSemObjectInit(&settings_api_ctx.sem, false);

  setreg = setreg_create();

  setreg_api_t api = {0};
  api.ctx = (void *)&settings_api_ctx;
  api.send = send_wrap;
  api.send_from = send_from_wrap;
  api.wait_init = wait_init_wrap;
  api.wait = wait_wrap;
  api.wait_deinit = wait_deinit_wrap;
  api.signal = signal_wrap;
  api.register_cb = reg_cb_wrap;
  api.unregister_cb = unreg_cb_wrap;
  api.log = log_;

  setreg_api_init(&api);
}

int settings_type_register_enum(const char *const enum_names[],
                                settings_type_t *type) {
  assert(setreg != NULL);
  assert(enum_names != NULL);
  assert(type != NULL);

  return setreg_add_enum(setreg, enum_names, type);
}

static int settings_default_notify(void *ctx) {
  (void)ctx;
  return SBP_SETTINGS_WRITE_STATUS_OK;
}

int settings_register(struct setting *setting, settings_type_t type) {
  return setreg_add_setting(setreg,
                            setting->section,
                            setting->name,
                            setting->addr,
                            setting->len,
                            type,
                            (NULL == setting->notify) ? settings_default_notify : setting->notify,
                            setting->notify_ctx);
}

int settings_register_readonly(struct setting *setting, settings_type_t type) {
  return setreg_add_readonly(setreg,
                             setting->section,
                             setting->name,
                             setting->addr,
                             setting->len,
                             type);
}

int settings_watch(struct setting *setting, settings_type_t type) {
  return setreg_add_watch(setreg,
                          setting->section,
                          setting->name,
                          setting->addr,
                          setting->len,
                          type,
                          (NULL == setting->notify) ? settings_default_notify : setting->notify,
                          setting->notify_ctx);
}
