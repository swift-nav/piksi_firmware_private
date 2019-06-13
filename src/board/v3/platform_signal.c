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

#include "decode/decode_bds3_b5.h"
#include "decode/decode_bds_b1.h"
#include "decode/decode_bds_b2.h"
#include "decode/decode_gal_e1.h"
#include "decode/decode_gal_e5.h"
#include "decode/decode_gal_e7.h"
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
#if defined CODE_GPS_L1CA_SUPPORT && CODE_GPS_L1CA_SUPPORT > 0
  track_gps_l1ca_register();
#endif
#if defined CODE_GPS_L2C_SUPPORT && CODE_GPS_L2C_SUPPORT > 0
  track_gps_l2c_register();
#endif
#if defined CODE_GPS_L5_SUPPORT && CODE_GPS_L5_SUPPORT > 0
  track_gps_l5_register();
#endif
#if defined CODE_GPS_AUX_SUPPORT && CODE_GPS_AUX_SUPPORT > 0
  track_gps_aux_register();
#endif
#if defined CODE_GLO_L1OF_SUPPORT && CODE_GLO_L1OF_SUPPORT > 0
  track_glo_l1of_register();
#endif
#if defined CODE_GLO_L2OF_SUPPORT && CODE_GLO_L2OF_SUPPORT > 0
  track_glo_l2of_register();
#endif
#if defined CODE_SBAS_L1CA_SUPPORT && CODE_SBAS_L1CA_SUPPORT > 0
  track_sbas_l1_register();
#endif
#if defined CODE_QZSS_L1CA_SUPPORT && CODE_QZSS_L1CA_SUPPORT > 0
  track_qzss_l1ca_register();
#endif
#if defined CODE_QZSS_L2C_SUPPORT && CODE_QZSS_L2C_SUPPORT > 0
  track_qzss_l2c_register();
#endif
#if defined CODE_QZSS_L5_SUPPORT && CODE_QZSS_L5_SUPPORT > 0
  track_qzss_l5_register();
#endif
#if defined CODE_BDS2_B1_SUPPORT && CODE_BDS2_B1_SUPPORT > 0
  track_bds2_b11_register();
#endif
#if defined CODE_BDS2_B2_SUPPORT && CODE_BDS2_B2_SUPPORT > 0
  track_bds2_b2_register();
#endif
#if defined CODE_BDS3_B5_SUPPORT && CODE_BDS3_B5_SUPPORT > 0
  track_bds3_b5_register();
#endif
#if defined CODE_GAL_E1_SUPPORT && CODE_GAL_E1_SUPPORT > 0
  track_gal_e1_register();
#endif
#if defined CODE_GAL_E7_SUPPORT && CODE_GAL_E7_SUPPORT > 0
  track_gal_e7_register();
#endif
#if defined CODE_GAL_E5_SUPPORT && CODE_GAL_E5_SUPPORT > 0
  track_gal_e5_register();
#endif
}

void platform_decode_setup(void) {
#if defined CODE_GPS_L1CA_SUPPORT && CODE_GPS_L1CA_SUPPORT > 0
  decode_gps_l1ca_register();
#endif
#if defined CODE_GPS_L2C_SUPPORT && CODE_GPS_L2C_SUPPORT > 0
  decode_gps_l2c_register();
#endif
#if defined CODE_GLO_L1OF_SUPPORT && CODE_GLO_L1OF_SUPPORT > 0
  decode_glo_l1of_register();
#endif
#if defined CODE_GLO_L2OF_SUPPORT && CODE_GLO_L2OF_SUPPORT > 0
  decode_glo_l2of_register();
#endif
#if defined CODE_SBAS_L1CA_SUPPORT && CODE_SBAS_L1CA_SUPPORT > 0
  decode_sbas_l1_register();
#endif
#if defined CODE_QZSS_L1CA_SUPPORT && CODE_QZSS_L1CA_SUPPORT > 0
  decode_qzss_l1ca_register();
#endif
#if defined CODE_QZSS_L2C_SUPPORT && CODE_QZSS_L2C_SUPPORT > 0
  /*  */
#endif
#if defined CODE_QZSS_L5_SUPPORT && CODE_QZSS_L5_SUPPORT > 0
  /*  */
#endif
#if defined CODE_BDS2_B1_SUPPORT && CODE_BDS2_B1_SUPPORT > 0
  decode_bds_b1_register();
#endif
#if defined CODE_BDS2_B2_SUPPORT && CODE_BDS2_B2_SUPPORT > 0
  decode_bds_b2_register();
#endif
#if defined CODE_BDS3_B5_SUPPORT && CODE_BDS3_B5_SUPPORT > 0
  decode_bds3_b5_register();
#endif
#if defined CODE_GAL_E1_SUPPORT && CODE_GAL_E1_SUPPORT > 0
  decode_gal_e1_register();
#endif
#if defined CODE_GAL_E7_SUPPORT && CODE_GAL_E7_SUPPORT > 0
  decode_gal_e7_register();
#endif
#if defined CODE_GAL_E5_SUPPORT && CODE_GAL_E5_SUPPORT > 0
  decode_gal_e5_register();
#endif
}

void platform_ndb_init(void) {
  ndb_ephemeris_init();
  ndb_almanac_init();
  ndb_gnss_capb_init();
  ndb_iono_init();
  ndb_lgf_init();
  ndb_utc_params_init();
}

void platform_ndb_sbp_updates(void) { ndb_almanac_sbp_update(); }
