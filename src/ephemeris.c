/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *          Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <string.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/logging.h>
#include <ch.h>
#include <assert.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "timing.h"
#include "ephemeris.h"
#include "signal.h"
#include "ndb.h"

void ephemeris_new(ephemeris_t *e)
{
  assert(sid_supported(e->sid));

  if (!e->valid) {
    log_warn_sid(e->sid, "invalid ephemeris");
    return;
  }

  ndb_op_code_t oc = ndb_ephemeris_store(e, NDB_DS_RECEIVER);
  switch (oc) {
  case NDB_ERR_NONE:
    log_debug_sid(e->sid, "ephemeris saved");
    break;
  case NDB_ERR_NO_CHANGE:
    log_debug_sid(e->sid, "ephemeris is already present");
    break;
  case NDB_ERR_UNRELIABLE_DATA:
    log_debug_sid(e->sid, "ephemeris is unconfirmed, not saved");
    break;
  case NDB_ERR_OLDER_DATA:
    log_warn_sid(e->sid, "ephemeris is older than one in DB, not saved");
    break;
  case NDB_ERR_MISSING_IE:
  case NDB_ERR_UNSUPPORTED:
  case NDB_ERR_FILE_IO:
  case NDB_ERR_INIT_DONE:
  case NDB_ERR_BAD_PARAM:
  case NDB_ERR_ALGORITHM_ERROR:
  case NDB_ERR_NO_DATA:
  default:
    log_warn_sid(e->sid, "error %d storing ephemeris", (int)oc);
    break;
  }
}

static void ephemeris_msg_callback(u16 sender_id, u8 len, u8 msg[],
                                       void* context)
{
  (void)sender_id; (void)context;

  if (len != sizeof(msg_ephemeris_t)) {
    log_warn("Received bad ephemeris from peer");
    return;
  }

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  unpack_ephemeris((msg_ephemeris_t *)msg, &e);
  if (!sid_supported(e.sid)) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }

  ndb_ephemeris_store(&e, NDB_DS_SBP);
}

void ephemeris_setup(void)
{
  sbp_ephe_reg_cbks(ephemeris_msg_callback);
}
