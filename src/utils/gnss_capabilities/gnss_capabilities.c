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

#include <libswiftnav/common.h>

#include "gnss_capabilities.h"

/* one can use a simple Matlab script to generate these:
PRNS_PRESENT = [11 12 19  26 24 30 ...
                 8  9  1   2  7  3 ...
                 4  5 21  25 27 31];
mask = repmat('0', 1, 64);
mask(PRNS_PRESENT) = '1';
fl_mask = fliplr(mask);
fprintf(1, '0x%016Xull \n', bin2dec(fl_mask))
fprintf(1, 'E%02d\n', strfind(mask, '1'));
*/

/* as of June 2018 */
gnss_capabilities_t gnss_capab = {.gps_active = 0x0ffffffffULL,
                                  .gps_l2c = 0x0f7814bf5ULL,
                                  .gps_l5 = 0x0a78003a5ULL,

                                  .glo_active = 0x00fff7df,
                                  .glo_l2of = 0x00fff7df,
                                  .glo_l3 = 0x00080100,

                                  .sbas_active = 0x7ffff,
                                  .sbas_l5 = 0x5b8c8,

                                  /* Note: BDS GEO SVs are marked as inactive,
                                  in order to prevent their acquisition.
                                  Configuration compliant with to
                                  http://www.csno-tarc.cn/system/basicinfo
                                  retrieved on May 2nd*/
                                  .bds_active = 0x07fe0ULL,
                                  .bds_d2nav = 0x1001fULL,
                                  .bds_b2 = 0x17fffULL,

                                  .qzss_active = 0x3ff,

                                  .gal_active = 0x0000000067940DDFull,
                                  .gal_e5 = 0x0000000067940DDFull,
                                  .gps_e6 = 0x0000000067940DDFull};

/** \defgroup gnss_capabilities Constellation capabilities
 * Functions used in gnss capabilities
 * \{ */

/** Returns true if Glonass satellite is active.
 * \param mesid   satellite identifier
 */
bool glo_active(const gnss_signal_t sid) {
  assert(IS_GLO(sid));
  return (0 != (gnss_capab.glo_active & (1 << (sid.sat - GLO_FIRST_PRN))));
}

/** Returns true if SBAS satellite is active.
 * \param mesid   satellite identifier
 */
bool sbas_active(const me_gnss_signal_t mesid) {
  assert(IS_SBAS(mesid));
  return (0 != (gnss_capab.sbas_active & (1 << (mesid.sat - SBAS_FIRST_PRN))));
}

/** Returns true if Beidou satellite is active.
 * \param mesid   satellite identifier
 */
bool bds_active(const me_gnss_signal_t mesid) {
  assert(IS_BDS2(mesid));
  return (0 !=
          (gnss_capab.bds_active & (1ULL << (mesid.sat - BDS2_FIRST_PRN))));
}

/** Returns true if Beidou satellite is geostationary using D2 navigation
 * message
 * \param mesid   satellite identifier
 */
bool bds_d2nav(const me_gnss_signal_t mesid) {
  assert(IS_BDS2(mesid));
  return (0 != (gnss_capab.bds_d2nav & (1ULL << (mesid.sat - BDS2_FIRST_PRN))));
}

/** Returns true if Beidou satellite uses the B2 (2 MHz) frequency
 * \param mesid   satellite identifier
 */
bool bds_b2(const me_gnss_signal_t mesid) {
  assert(IS_BDS2(mesid));
  return (0 != (gnss_capab.bds_b2 & (1ULL << (mesid.sat - BDS2_FIRST_PRN))));
}

/** Returns true if QZSS satellite is active
 * \param mesid   satellite identifier
 */
bool qzss_active(const me_gnss_signal_t mesid) {
  assert(IS_QZSS(mesid));
  return (0 != (gnss_capab.qzss_active & (1 << (mesid.sat - QZS_FIRST_PRN))));
}

/** Returns true if Galileo satellite is active
 * \param mesid   satellite identifier
 */
bool gal_active(const me_gnss_signal_t mesid) {
  assert(IS_GAL(mesid));
  return (0 != (gnss_capab.gal_active & (1ULL << (mesid.sat - GAL_FIRST_PRN))));
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
      {4, 12}, {6, 4}, {6, 4}, {6, 4}, {6, 4}, {4, 4},
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

/** \} */
