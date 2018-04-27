/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Measurement Engine team <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "me_msg.h"

#define ME_OBS_MSG_N_BUFF 6

memory_pool_t me_obs_msg_buff_pool;
mailbox_t me_obs_msg_mailbox;

static msg_t me_obs_msg_mailbox_buff[ME_OBS_MSG_N_BUFF];
static me_msg_obs_t me_obs_msg_buff[ME_OBS_MSG_N_BUFF];

void me_obs_msg_setup(void) {
  chMBObjectInit(
      &me_obs_msg_mailbox, me_obs_msg_mailbox_buff, ME_OBS_MSG_N_BUFF);
  chPoolObjectInit(&me_obs_msg_buff_pool, sizeof(me_msg_obs_t), NULL);
  chPoolLoadArray(&me_obs_msg_buff_pool, me_obs_msg_buff, ME_OBS_MSG_N_BUFF);
}

#define SBAS_MSG_N_BUFF 6
mailbox_t sbas_msg_mailbox;
memory_pool_t sbas_msg_buff_pool;

static msg_t sbas_msg_mailbox_buff[SBAS_MSG_N_BUFF];
static msg_sbas_raw_t sbas_msg_buff[SBAS_MSG_N_BUFF];

void sbas_msg_setup(void) {
  chMBObjectInit(&sbas_msg_mailbox, sbas_msg_mailbox_buff, SBAS_MSG_N_BUFF);
  chPoolObjectInit(&sbas_msg_buff_pool, sizeof(msg_sbas_raw_t), NULL);
  chPoolLoadArray(&sbas_msg_buff_pool, sbas_msg_buff, SBAS_MSG_N_BUFF);
}
