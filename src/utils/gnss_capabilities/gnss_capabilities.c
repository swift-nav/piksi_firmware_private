/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <stdio.h>

#include <libsbp/observation.h>
#include <swiftnav/common.h>

#include "gnss_capabilities.h"
#include "ndb/ndb.h"
#include "sbp_utils.h"

/** \defgroup gnss_capabilities Constellation capabilities
 * Functions used in gnss capabilities
 * \{ */

bool gps_l2c_active(const me_gnss_signal_t mesid) {
  assert(IS_GPS(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u64 capb = gc->gps_l2c;
  return (0 != (capb & ((u64)1 << (mesid.sat - GPS_FIRST_PRN))));
}

/** Returns true if Glonass satellite is active.
 * \param sid   satellite identifier
 */
bool glo_active(const gnss_signal_t sid) {
  assert(IS_GLO(sid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u32 capb = gc->glo_active;
  return (0 != (capb & ((u32)1 << (sid.sat - GLO_FIRST_PRN))));
}

/** Returns true if SBAS satellite is active.
 * \param mesid   satellite identifier
 */
bool sbas_active(const me_gnss_signal_t mesid) {
  assert(IS_SBAS(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u32 capb = gc->sbas_active;
  return (0 != (capb & ((u32)1 << (mesid.sat - SBAS_FIRST_PRN))));
}

/** Returns true if Beidou satellite is active.
 * \param mesid   satellite identifier
 */
bool bds_active(const me_gnss_signal_t mesid) {
  assert(IS_BDS2(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u64 capb = gc->bds_active;
  return (0 != (capb & ((u64)1 << (mesid.sat - BDS_FIRST_PRN))));
}

/** Returns true if Beidou satellite is geostationary using D2 navigation
 * message
 * \param mesid   satellite identifier
 */
bool bds_d2nav(const me_gnss_signal_t mesid) {
  assert(IS_BDS2(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u64 capb = gc->bds_d2nav;
  return (0 != (capb & ((u64)1 << (mesid.sat - BDS_FIRST_PRN))));
}

/** Returns true if Beidou satellite uses the B2 (2 MHz) frequency
 * \param mesid   satellite identifier
 */
bool bds_b2(const me_gnss_signal_t mesid) {
  assert(IS_BDS2(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u64 capb = gc->bds_b2;
  return (0 != (capb & ((u64)1 << (mesid.sat - BDS_FIRST_PRN))));
}

/** Returns true if QZSS satellite is active
 * \param mesid   satellite identifier
 */
bool qzss_active(const me_gnss_signal_t mesid) {
  assert(IS_QZSS(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u32 capb = gc->qzss_active;
  return (0 != (capb & ((u32)1 << (mesid.sat - QZS_FIRST_PRN))));
}

/** Returns true if Galileo satellite is active
 * \param mesid   satellite identifier
 */
bool gal_active(const me_gnss_signal_t mesid) {
  assert(IS_GAL(mesid));
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  assert(gc);
  u64 capb = gc->gal_active;
  return (0 != (capb & ((u64)1 << (mesid.sat - GAL_FIRST_PRN))));
}

/** \} */

/** \defgroup l2c_capability L2C Capability
 * Functions used in l2c capability
 * \{ */

/** Decode L2C capability data from Nav message.
 * \param subframe4_words Array containing words 3 through 10
 *                        of subframe 4.
 *                        SV configuration data is in words 3 through 8
 * \param l2c_cpbl output Pointer to 32-bit L2C capability container
 *
 */
void decode_l2c_capability(const u32 *subframe4_words, u32 *l2c_cpbl) {
  assert(subframe4_words != NULL);
  assert(l2c_cpbl != NULL);

  *l2c_cpbl = 0;

  struct sv_conf_location {
    u8 n_sv;      /* for how many SVs config data in the Word */
    u8 end_bit_n; /* position of MSB for the 1st SV in the Word */
  };

  const struct sv_conf_location sv_conf_loc[6] = {
      {4, 12},
      {6, 4},
      {6, 4},
      {6, 4},
      {6, 4},
      {4, 4},
  };

  u8 sv_id = 0;

  /* go through words 3-8 */
  for (u8 i = 3; i <= 8; i++) {
    /* go through all sv inside the word */
    for (u8 j = 0; j < sv_conf_loc[i - 3].n_sv; j++) {
      /* get SV config bits take into account only 3 LSB */
      u8 sv_conf = subframe4_words[i - 3] >>
                       (30 - (sv_conf_loc[i - 3].end_bit_n + j * 4)) &
                   7;

      /* set or clear appropriate capability bit,
       * refer pg. 115-116 of IS-200H for the criteria
       * uses an open upper bound to ensure we track L2C on future satellite
       * generations launched after ICD was updated */
      if (sv_conf >= 2) *l2c_cpbl |= (1u << sv_id);

      sv_id++;
    }
  }
}

void gnss_capb_send_over_sbp(void) {
  const gnss_capb_t *gc = ndb_get_gnss_capb();
  sbp_send_gnss_capb(gc);
}

/** \} */
