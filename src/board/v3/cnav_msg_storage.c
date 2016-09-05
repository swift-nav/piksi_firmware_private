/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>
#include <libswiftnav/signal.h>
#include <ch.h>
#include <assert.h>
#include "cnav_msg_storage.h"
#include "shm.h"

static MUTEX_DECL(cnav_msg_mutex);
static cnav_msg_storage_t cnav_msg_storage[NUM_SATS_GPS][CNAV_MSG_TYPE_NUM];

static cnav_msg_idx_t cnav_msg_type_to_idx(cnav_msg_type_t t)
{
  switch (t) {
    case CNAV_MSG_TYPE_10: return CNAV_MSG_TYPE_IDX_10;
    case CNAV_MSG_TYPE_11: return CNAV_MSG_TYPE_IDX_11;
    case CNAV_MSG_TYPE_30: return CNAV_MSG_TYPE_IDX_30;
    case CNAV_MSG_TYPE_32: return CNAV_MSG_TYPE_IDX_32;
    case CNAV_MSG_TYPE_33: return CNAV_MSG_TYPE_IDX_33;
    default:
      assert(!"Unsupported CNAV message type");
  }
}

static bool cnav_msg_type_id_valid(u8 msg_id)
{
  switch(msg_id) {
    case CNAV_MSG_TYPE_10:
    case CNAV_MSG_TYPE_11:
    case CNAV_MSG_TYPE_30:
    case CNAV_MSG_TYPE_32:
    case CNAV_MSG_TYPE_33:
      return true;
  }

  return false;
}

void cnav_msg_put(const cnav_msg_t *msg)
{
  if (cnav_msg_type_id_valid(msg->msg_id) && (msg->prn <= NUM_SATS_GPS))
  {
    u8 msg_idx = cnav_msg_type_to_idx(msg->msg_id);
    gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, msg->prn);
    u16 sat_idx = sid_to_code_index(sid);
    chMtxLock(&cnav_msg_mutex);
    cnav_msg_storage_t *storage_cell = &(cnav_msg_storage[sat_idx][msg_idx]);
    storage_cell->msg = *msg;
    storage_cell->msg_set = true;
    chMtxUnlock(&cnav_msg_mutex);
    log_debug_sid(sid, "CNAV message type %d saved", msg->msg_id);
    shm_log_sat_state(msg->prn);
  }
}

bool cnav_msg_get(gnss_signal_t sid, cnav_msg_type_t type, cnav_msg_t *msg)
{
  bool res = false;

  if (cnav_msg_type_id_valid(type) &&
      (sid_valid(sid) && sid_to_constellation(sid) == CONSTELLATION_GPS))
  {
    u16 sat_idx = sid_to_code_index(sid);
    u8 msg_idx = cnav_msg_type_to_idx(type);
    chMtxLock(&cnav_msg_mutex);
    cnav_msg_storage_t *storage_cell = &(cnav_msg_storage[sat_idx][msg_idx]);
    if (storage_cell->msg_set) {
      *msg = storage_cell->msg;
      res = true;
    }
    chMtxUnlock(&cnav_msg_mutex);
  }

  return res;
}
