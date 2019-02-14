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

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <starling/pvt_engine/firmware_binding.h>
#include <swiftnav/constants.h>
#include <swiftnav/glo_map.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>

#include "ndb/ndb.h"
#include "sbp_utils.h"
#include "timing/timing.h"

/** \addtogroup sbp
 * \{ */

/** \defgroup sbp_utils SBP Utils
 * Convert to and from SBP message types and other useful functions.
 * \{ */

void sbp_send_ndb_event(u8 event,
                        u8 obj_type,
                        u8 result,
                        u8 data_source,
                        const gnss_signal_t *object_sid,
                        const gnss_signal_t *src_sid,
                        u16 sender) {
  msg_ndb_event_t msg;
  memset(&msg, 0, sizeof(msg));

  msg.recv_time = timing_getms();
  msg.event = event;
  msg.object_type = obj_type;
  msg.result = result;
  msg.data_source = data_source;

  if (NULL != object_sid) {
    msg.object_sid = sid_to_sbp(*object_sid);
  }

  if (NULL != src_sid) {
    msg.src_sid = sid_to_sbp(*src_sid);
  }

  msg.original_sender = sender;

  sbp_send_msg(SBP_MSG_NDB_EVENT, sizeof(msg), (u8 *)&msg);
}

#define TYPE_TABLE_INVALID_MSG_ID 0

typedef struct {
  const msg_info_t msg_info;
  sbp_msg_callbacks_node_t cbk_node;
} ephe_type_table_element_t;

static ephe_type_table_element_t ephe_type_table[CONSTELLATION_COUNT] = {
    /* GPS */
    [CONSTELLATION_GPS] = {{SBP_MSG_EPHEMERIS_GPS, sizeof(msg_ephemeris_gps_t)},
                           {0}},

    /* SBAS */
    [CONSTELLATION_SBAS] = {{SBP_MSG_EPHEMERIS_SBAS,
                             sizeof(msg_ephemeris_sbas_t)},
                            {0}},

    /* GLO */
    [CONSTELLATION_GLO] = {{SBP_MSG_EPHEMERIS_GLO, sizeof(msg_ephemeris_glo_t)},
                           {0}},

    /* BDS */
    [CONSTELLATION_BDS] = {{SBP_MSG_EPHEMERIS_BDS, sizeof(msg_ephemeris_bds_t)},
                           {0}},

    /* GAL */
    [CONSTELLATION_GAL] = {{SBP_MSG_EPHEMERIS_GAL, sizeof(msg_ephemeris_gal_t)},
                           {0}},
};

void sbp_ephe_reg_cbks(void (*ephemeris_msg_callback)(u16, u8, u8 *, void *)) {
  assert(ARRAY_SIZE(ephe_type_table) == CONSTELLATION_COUNT);

  for (u8 i = 0; i < ARRAY_SIZE(ephe_type_table); i++) {
    /* check if type is valid */
    if (TYPE_TABLE_INVALID_MSG_ID == ephe_type_table[i].msg_info.msg_id) {
      continue;
    }

    sbp_register_cbk(ephe_type_table[i].msg_info.msg_id,
                     ephemeris_msg_callback,
                     &ephe_type_table[i].cbk_node);
  }
}

/**
 * This is helper function packs and sends iono parameters over SBP
 * @param[in] iono pointer to Iono parameters
 */
void sbp_send_iono(const ionosphere_t *iono) {
  msg_iono_t msg_iono = {.t_nmct =
                             {/* TODO: set this as 0 for now, beccause
                               * functionality decodes tnmct is not available */
                              .tow = 0,
                              .wn = 0},
                         .a0 = iono->a0,
                         .a1 = iono->a1,
                         .a2 = iono->a2,
                         .a3 = iono->a3,
                         .b0 = iono->b0,
                         .b1 = iono->b1,
                         .b2 = iono->b2,
                         .b3 = iono->b3};

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_IONO, sizeof(msg_iono_t), (u8 *)&msg_iono);
}

/**
 * This is helper function packs and sends gnss capabilities over SBP
 * @param[in] gc pointer to gnss capabilities
 */
void sbp_send_gnss_capb(const gnss_capb_t *gc) {
  assert(gc);
  msg_gnss_capb_t msg = {.t_nmct =
                             {/* TODO: set this as 0 for now, beccause
                               * functionality decodes tnmct is not available */
                              .tow = 0,
                              .wn = 0},
                         .gc = *gc};

  sbp_send_msg(SBP_MSG_GNSS_CAPB, sizeof(msg), (u8 *)&msg);
}

/**
 * This is helper function packs and sends Group delay over SBP
 * @param[in] cnav pointer to GPS CNAV message structure
 */
void sbp_send_group_delay(const cnav_msg_t *cnav) {
  gps_time_t t = get_current_time();
  msg_group_delay_t msg_cnav = {
      .t_op =
          {/* Convert from 6 seconds unit */
           .tow = (u32)(cnav->tow * 6),
           .wn = t.wn},
      .sid = (sbp_gnss_signal_t){.code = CODE_GPS_L2CM, .sat = cnav->prn},
      .valid = cnav->data.type_30.tgd_valid |
               cnav->data.type_30.isc_l2c_valid << 1 |
               cnav->data.type_30.isc_l1ca_valid << 2,
      .tgd = cnav->data.type_30.tgd,
      .isc_l1ca = cnav->data.type_30.isc_l1ca,
      .isc_l2c = cnav->data.type_30.isc_l2c};

  /* send data over sbp */
  sbp_send_msg(SBP_MSG_GROUP_DELAY, sizeof(msg_group_delay_t), (u8 *)&msg_cnav);
}

/** \} */
/** \} */
