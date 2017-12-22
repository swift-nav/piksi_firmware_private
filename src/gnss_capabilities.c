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

#include "gnss_capabilities.h"

gnss_capabilities_t gnss_capab = {.gps_active = 0x0ffffffffULL,
                                  .gps_l2c = 0x0f7814bf5ULL,
                                  .gps_l5 = 0x0a78003a5ULL,

                                  .glo_active = 0x03ffffff,
                                  .glo_l2of = 0x03ffffff,
                                  .glo_l3 = 0x00080100,

                                  .sbas_active = 0x0007ffff,
                                  .sbas_l5 = 0x0005b8c8,

                                  .bds_active = 0x1fc00d3fffULL,
                                  .bds_d2nav = 0x000001001fULL,
                                  .bds_b2 = 0x0000003fffULL,

                                  .qzss_active = 0xf,

                                  .gal_active = 0x022ae2ddbULL,
                                  .gal_e5 = 0x022a62ddbULL,
                                  .gps_e6 = 0x022a62ddbULL};

/** \defgroup gnss_capabilities Constellation capabilities
 * Functions used in gnss capabilities
 * \{ */

/** Returns true if Glonass satellite is active.
 * \param mesid   satellite identifier
 */
bool glo_active(const me_gnss_signal_t mesid) {
  assert(IS_GLO(mesid));
  return (0 != (gnss_capab.glo_active & (1 << (mesid.sat - GLO_FIRST_PRN))));
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
