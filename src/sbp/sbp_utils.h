/*
 * Copyright (C) 2014, 2016 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SBP_UTILS_H
#define SWIFTNAV_SBP_UTILS_H

#include <libsbp/common.h>
#include <libsbp/gnss.h>
#include <libsbp/navigation.h>
#include <libsbp/ndb.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
#include <starling/observation.h>
#include <starling/util/sbp/misc.h>
#include <starling/util/sbp/packers.h>
#include <starling/util/sbp/unpackers.h>
#include <swiftnav/almanac.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/signal.h>
#include <swiftnav/single_epoch_solver.h>

#include "nav_msg/cnav_msg.h"
#include "obs_bias/obs_bias.h"
#include "sbp.h"

typedef enum {
  NDB_EVENT_UNKNOWN = 0,
  NDB_EVENT_STORE = 1,
  NDB_EVENT_FETCH = 2,
  NDB_EVENT_ERASE = 3,
} ndb_event_t;

typedef enum {
  NDB_EVENT_OTYPE_UNKNOWN = 0,
  NDB_EVENT_OTYPE_EPHEMERIS = 1,
  NDB_EVENT_OTYPE_ALMANAC = 2,
  NDB_EVENT_OTYPE_ALMANAC_WN = 3,
  NDB_EVENT_OTYPE_IONO = 4,
  /* NDB_EVENT_OTYPE_L2C_CAP = 5 is obsoleted by NDB_EVENT_OTYPE_GNSS_CAP */
  NDB_EVENT_OTYPE_LGF = 6,
  NDB_EVENT_OTYPE_UTC_PARAMS = 7,
  NDB_EVENT_OTYPE_GNSS_CAPB = 8
} ndb_event_obj_type_t;

#define NDB_EVENT_SENDER_ID_VOID 0

#ifdef __cplusplus
extern "C" {
#endif

void sbp_send_ndb_event(u8 event,
                        u8 obj_type,
                        u8 result,
                        u8 data_source,
                        const gnss_signal_t *object_sid,
                        const gnss_signal_t *src_sid,
                        u16 sender);

#define MSG_FORWARD_SENDER_ID 0

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8 *, void *));

/** Value specifying the size of the SBP framing */
#define SBP_FRAMING_SIZE_BYTES 8
/** Value defining maximum SBP packet size */
#define SBP_FRAMING_MAX_PAYLOAD_SIZE 255

void sbp_send_iono(const ionosphere_t *iono);
void sbp_send_gnss_capb(const gnss_capb_t *gc);
void sbp_send_group_delay(const cnav_msg_t *cnav);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_SBP_UTILS_H */
