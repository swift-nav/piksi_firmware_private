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

#include <libswiftnav/sbas_raw_data.h>

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

#define SBAS_DATA_N_BUFF 6
mailbox_t sbas_data_mailbox;
memory_pool_t sbas_data_buff_pool;

static msg_t sbas_data_mailbox_buff[SBAS_DATA_N_BUFF];
static sbas_raw_data_t sbas_data_buff[SBAS_DATA_N_BUFF];

void sbas_msg_setup(void) {
  chMBObjectInit(&sbas_data_mailbox, sbas_data_mailbox_buff, SBAS_DATA_N_BUFF);
  chPoolObjectInit(&sbas_data_buff_pool, sizeof(sbas_raw_data_t), NULL);
  chPoolLoadArray(&sbas_data_buff_pool, sbas_data_buff, SBAS_DATA_N_BUFF);
}
