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

#include "cnav_msg_storage.h"
#include <assert.h>
#include <ch.h>
#include <libswiftnav/signal.h>
#include <string.h>
#include "shm.h"

static MUTEX_DECL(cnav_msg_mutex);
static cnav_msg_storage_t cnav_msg_storage[NUM_SATS_GPS][CNAV_MSG_TYPE_NUM];

static cnav_msg_idx_t cnav_msg_type_to_idx(cnav_msg_type_t t) {
  switch (t) {
    case CNAV_MSG_TYPE_10:
      return CNAV_MSG_TYPE_IDX_10;
    case CNAV_MSG_TYPE_11:
      return CNAV_MSG_TYPE_IDX_11;
    case CNAV_MSG_TYPE_30:
      return CNAV_MSG_TYPE_IDX_30;
    case CNAV_MSG_TYPE_32:
      return CNAV_MSG_TYPE_IDX_32;
    case CNAV_MSG_TYPE_33:
      return CNAV_MSG_TYPE_IDX_33;
    default:
      assert(!"Unsupported CNAV message type");
  }
}

static bool cnav_msg_type_id_valid(u8 msg_id) {
  switch (msg_id) {
    case CNAV_MSG_TYPE_10:
    case CNAV_MSG_TYPE_11:
    case CNAV_MSG_TYPE_30:
    case CNAV_MSG_TYPE_32:
    case CNAV_MSG_TYPE_33:
      return true;
    default:
      assert(!"Unsupported CNAV message type");
  }

  return false;
}

/** Store decoded CNAV (GPS L2CM) navigation message
 *
 * \param msg pointer to message to be stored
 */
void cnav_msg_put(const cnav_msg_t *msg) {
  if (cnav_msg_type_id_valid(msg->msg_id) &&
      (sid_valid(construct_sid(CODE_GPS_L2CM, msg->prn)))) {
    u8 msg_idx = cnav_msg_type_to_idx(msg->msg_id);
    gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, msg->prn);
    u16 sat_idx = sid_to_code_index(sid);
    chMtxLock(&cnav_msg_mutex);
    cnav_msg_storage_t *storage_cell = &(cnav_msg_storage[sat_idx][msg_idx]);
    storage_cell->msg = *msg;
    storage_cell->msg_valid = true;
    chMtxUnlock(&cnav_msg_mutex);
    log_debug_sid(sid, "CNAV message type %d saved", msg->msg_id);
    shm_log_sat_state("SHI5", msg->prn);
  }
}

/** Retrieves content of CNAV (GPS L2CM) navigation message
 *
 * \param sid Signal ID
 * \param type message type to be retrieved
 * \param msg pointer to message to be retrieved
 *
 * \returns true if required CNAV message for specified signal was found
 *          and copied to provided buffer, false otherwise.
 */
bool cnav_msg_get(gnss_signal_t sid, cnav_msg_type_t type, cnav_msg_t *msg) {
  bool res = false;

  if (cnav_msg_type_id_valid(type) && sid_valid(sid) && IS_GPS(sid)) {
    u16 sat_idx = sid_to_code_index(sid);
    u8 msg_idx = cnav_msg_type_to_idx(type);
    chMtxLock(&cnav_msg_mutex);
    cnav_msg_storage_t *storage_cell = &(cnav_msg_storage[sat_idx][msg_idx]);
    if (storage_cell->msg_valid) {
      *msg = storage_cell->msg;
      res = true;
    }
    chMtxUnlock(&cnav_msg_mutex);
  }

  return res;
}
