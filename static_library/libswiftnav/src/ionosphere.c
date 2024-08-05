/*
 * Copyright (C) 2015, 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <swiftnav/almanac.h>
#include <swiftnav/constants.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/logging.h>

/** \defgroup ionosphere Ionospheric models
 * Implemenations of ionospheric delay correction models.
 * \{ */

/** Calculate ionospheric delay using Klobuchar model.
 *
 * References:
 *   -# IS-GPS-200H, Section 20.3.3.5.2.5 and Figure 20-4
 *
 * \param t_gps GPS time at which to calculate the ionospheric delay
 * \param lat_u Latitude of the receiver [rad]
 * \param lon_u Longitude of the receiver [rad]
 * \param a Azimuth of the satellite, clockwise positive from North [rad]
 * \param e Elevation of the satellite [rad]
 * \param i Ionosphere parameters struct from GPS NAV data
 *
 * \return Ionospheric delay distance for GPS L1 frequency [m]
 */
double calc_ionosphere(const gps_time_t *t_gps,
                       double lat_u,
                       double lon_u,
                       double a,
                       double e,
                       const ionosphere_t *i) {
  /* Convert inputs from radians to semicircles */
  /* All calculations are in semicircles */
  lat_u = lat_u / M_PI;
  lon_u = lon_u / M_PI;
  /* a can remain in radians */
  e = e / M_PI;

  /* Calculate the earth-centered angle */
  double psi = 0.0137 / (e + 0.11) - 0.022;

  /* Compute the latitude of the Ionospheric Pierce Point */
  double lat_i = lat_u + psi * cos(a);
  if (lat_i > 0.416) {
    lat_i = 0.416;
  }
  if (lat_i < -0.416) {
    lat_i = -0.416;
  }

  /* Compute the longitude of the IPP */
  double lon_i = lon_u + (psi * sin(a)) / cos(lat_i * M_PI);

  /* Find the geomagnetic latitude of the IPP */
  double lat_m = lat_i + 0.064 * cos((lon_i - 1.617) * M_PI);

  /* Find the local time at the IPP */
  double t = 43200.0 * lon_i + t_gps->tow;
  t = fmod(t, DAY_SECS);
  if (t > DAY_SECS) {
    t -= DAY_SECS;
  }
  if (t < 0.0) {
    t += DAY_SECS;
  }

  /* Compute the amplitude of ionospheric delay */
  double amp = i->a0 + lat_m * (i->a1 + lat_m * (i->a2 + i->a3 * lat_m));
  if (amp < 0.0) {
    amp = 0.0;
  }

  /* Compute the period of ionospheric delay */
  double per = i->b0 + lat_m * (i->b1 + lat_m * (i->b2 + i->b3 * lat_m));
  if (per < 72000.0) {
    per = 72000.0;
  }

  /* Compute the phase of ionospheric delay */
  double x = 2.0 * M_PI * (t - 50400.0) / per;

  /* Compute the slant factor */
  double temp = 0.53 - e;
  double sf = 1.0 + 16.0 * temp * temp * temp;

  /* Compute the ionospheric time delay */
  double d_l1;
  if (fabs(x) >= 1.57) {
    d_l1 = sf * 5e-9;
  } else {
    double x_2 = x * x;
    d_l1 = sf * (5e-9 + amp * (1.0 - x_2 / 2.0 + x_2 * x_2 / 24.0));
  }

  d_l1 *= GPS_C;

  return d_l1;
}

/**
 * Decodes ionospheric parameters from GLS LNAV message subframe 4.
 *
 * The method decodes ionosphere data from GPS LNAV subframe 4 words 3-5.
 *
 * References:
 * -# IS-GPS-200H, Section 20.3.3.5.1.7
 *
 * \param[in]  words    Subframe 4 page 18.
 * \param[out] i        Destination object.
 *
 * \retval true  Ionosphere parameters have been decoded.
 * \retval false Decoding error.
 */
bool decode_iono_parameters(const u32 words[8], ionosphere_t *i) {
  bool retval = false;

  assert(NULL != words);
  assert(NULL != i);

  memset(i, 0, sizeof(*i));

  /* Word 3 bits 1-2: data ID */
  u8 data_id = words[3 - 3] >> (30 - 2) & 0x3;
  /* Word 3 bits 3-8: SV ID */
  u8 sv_id = words[3 - 3] >> (30 - 8) & 0x3F;

  if (GPS_LNAV_ALM_DATA_ID_BLOCK_II == data_id &&
      GPS_LNAV_ALM_SVID_IONO == sv_id) {
    /* Word 3 bits 9-16 */
    i->a0 = (s8)(words[3 - 3] >> (30 - 16) & 0xFF) * GPS_LNAV_IONO_SF_A0;
    /* Word 3 bits 17-24 */
    i->a1 = (s8)(words[3 - 3] >> (30 - 24) & 0xFF) * GPS_LNAV_IONO_SF_A1;
    /* Word 4 bits 1-8 */
    i->a2 = (s8)(words[4 - 3] >> (30 - 8) & 0xFF) * GPS_LNAV_IONO_SF_A2;
    /* Word 4 bits 9-16 */
    i->a3 = (s8)(words[4 - 3] >> (30 - 16) & 0xFF) * GPS_LNAV_IONO_SF_A3;
    /* Word 4 bits 17-24 */
    i->b0 = (s8)(words[4 - 3] >> (30 - 24) & 0xFF) * GPS_LNAV_IONO_SF_B0;
    /* Word 5 bits 1-8 */
    i->b1 = (s8)(words[5 - 3] >> (30 - 8) & 0xFF) * GPS_LNAV_IONO_SF_B1;
    /* Word 5 bits 9-16 */
    i->b2 = (s8)(words[5 - 3] >> (30 - 16) & 0xFF) * GPS_LNAV_IONO_SF_B2;
    /* Word 5 bits 17-24 */
    i->b3 = (s8)(words[5 - 3] >> (30 - 24) & 0xFF) * GPS_LNAV_IONO_SF_B3;
    retval = true;
  }

  return retval;
}

/**
 * Decodes Beidou D1 ionospheric parameters.
 * \param words subframes (FraID) 1.
 * \param iono ionospheric parameters.
 */
void decode_bds_d1_iono(const u32 words[10], ionosphere_t *iono) {
  const u32 *sf1_word = &words[0];
  s8 alpha[4];
  alpha[0] = (((sf1_word[4]) >> 16) & 0xff);
  alpha[1] = (((sf1_word[4]) >> 8) & 0xff);
  alpha[2] = (((sf1_word[5]) >> 22) & 0xff);
  alpha[3] = (((sf1_word[5]) >> 14) & 0xff);
  s8 beta[4];
  beta[0] = (((sf1_word[5]) >> 8) & 0x3f) << 2;
  beta[0] |= (((sf1_word[6]) >> 28) & 0x3);
  beta[1] = (((sf1_word[6]) >> 20) & 0xff);
  beta[2] = (((sf1_word[6]) >> 12) & 0xff);
  beta[3] = (((sf1_word[6]) >> 8) & 0xf) << 4;
  beta[3] |= (((sf1_word[7]) >> 26) & 0xf);

  iono->toa.wn = (((sf1_word[2]) >> 17) & 0x1fff) + BDS_WEEK_TO_GPS_WEEK;
  iono->a0 = (alpha[0] * C_1_2P30);
  iono->a1 = (alpha[1] * C_1_2P27);
  iono->a2 = (alpha[2] * C_1_2P24);
  iono->a3 = (alpha[3] * C_1_2P24);
  iono->b0 = (double)(beta[0] * C_2P11);
  iono->b1 = (double)(beta[1] * C_2P14);
  iono->b2 = (double)(beta[2] * C_2P16);
  iono->b3 = (double)(beta[3] * C_2P16);
}

/** \} */
