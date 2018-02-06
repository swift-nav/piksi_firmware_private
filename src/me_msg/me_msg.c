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

#define ME_MSG_N_BUFF 6

memory_pool_t me_msg_buff_pool;
mailbox_t me_msg_mailbox;

static msg_t me_msg_mailbox_buff[ME_MSG_N_BUFF];
static me_msg_t me_msg_buff[ME_MSG_N_BUFF];

void me_msg_setup(void) {
  chMBObjectInit(&me_msg_mailbox, me_msg_mailbox_buff, ME_MSG_N_BUFF);
  chPoolObjectInit(&me_msg_buff_pool, sizeof(me_msg_t), NULL);
  chPoolLoadArray(&me_msg_buff_pool, me_msg_buff, ME_MSG_N_BUFF);
}
