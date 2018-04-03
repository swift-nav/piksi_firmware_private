/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "platform_signal.h"
#include "decode/decode_bds_b1.h"
#include "decode/decode_bds_b2.h"
#include "decode/decode_glo_l1of.h"
#include "decode/decode_glo_l2of.h"
#include "decode/decode_gps_l1ca.h"
#include "decode/decode_gps_l2c.h"
#include "decode/decode_qzss_l1ca.h"
#include "decode/decode_sbas_l1.h"
#include "ndb/ndb.h"
#include "track/track_sid_db.h"
#include "track/track_signals.h"

void platform_track_setup(void) {
  track_sid_db_init();
  track_gps_l1ca_register();
  track_gps_l2c_register();
  track_glo_l1of_register();
  track_glo_l2of_register();
#ifdef CODE_SBAS_L1CA_SUPPORT
  track_sbas_l1_register();
#endif
#ifdef CODE_QZSS_L1CA_SUPPORT
  track_qzss_l1ca_register();
#endif
#ifdef CODE_QZSS_L2C_SUPPORT
  track_qzss_l2c_register();
#endif
#ifdef CODE_BDS2_B11_SUPPORT
  track_bds2_b11_register();
#endif
#ifdef CODE_BDS2_B2_SUPPORT
  track_bds2_b2_register();
#endif
}

void platform_decode_setup(void) {
  decode_gps_l1ca_register();
  decode_gps_l2c_register();
  decode_glo_l1of_register();
  decode_glo_l2of_register();
#ifdef CODE_SBAS_L1CA_SUPPORT
  decode_sbas_l1_register();
#endif
#ifdef CODE_QZSS_L1CA_SUPPORT
  decode_qzss_l1ca_register();
#endif
#ifdef CODE_BDS2_B11_SUPPORT
  decode_bds_b1_register();
#endif
#ifdef CODE_BDS2_B2_SUPPORT
  decode_bds_b2_register();
#endif
}

void platform_ndb_init(void) {
  ndb_ephemeris_init();
  ndb_almanac_init();
  ndb_l2c_capb_init();
  ndb_iono_init();
  ndb_lgf_init();
  ndb_utc_params_init();
}

void platform_ndb_sbp_updates(void) { ndb_almanac_sbp_update(); }
