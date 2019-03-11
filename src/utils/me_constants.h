/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ME_CONSTANTS_H
#define ME_CONSTANTS_H

/** Max user velocity for visibility calculation (m/s) */
#define MAX_USER_VELOCITY_MPS 38.0f

/** Approximate average distance to the GPS satellites in m. */
#define GPS_NOMINAL_RANGE 22.980e6

/** GPS L2C CNAV message length [bits] */
#define GPS_CNAV_MSG_LENGTH 300

/** GPS L1 C/A bit length [ms] */
#define GPS_L1CA_BIT_LENGTH_MS 20

/** GPS L2C symbol length [ms] */
#define GPS_L2C_SYMBOL_LENGTH_MS 20

/** GPS L5Q symbol length [ms] */
#define GPS_L5I_SYMBOL_LENGTH_MS 10

/** GPS L5Q symbol length [ms] */
#define GPS_L5Q_SYMBOL_LENGTH_MS 20

/** GPS L1 C/A symbol length [ms] */
#define GPS_L1CA_SYMBOL_LENGTH_MS 20

/** GLO L1CA symbol length [ms] */
#define GLO_L1CA_SYMBOL_LENGTH_MS 10

/** GLO meander wipe-off requires this symbol alignment [ms] */
#define GLO_MEANDER_WIPEOFF_ALIGN_MS (2 * GLO_L1CA_SYMBOL_LENGTH_MS)

/** GLO string length [ms] */
#define GLO_STRING_LENGTH_MS 2000

/** SBAS L1 symbol length [ms] */
#define SBAS_L1CA_SYMBOL_LENGTH_MS 2

/** SBAS L1 message length [bits] */
#define SBAS_MSG_LENGTH 250

/** Beidou2 B11 D1 symbol length */
#define BDS2_B11_D1NAV_SYMBOL_LENGTH_MS 20

/** Beidou2 B11 D2 symbol length */
#define BDS2_B11_D2NAV_SYMBOL_LENGTH_MS 2

/** QZSS L2 CM PRN period in ms */
#define QZS_L2CM_PRN_PERIOD_MS 20

/** QZSS L2 CL PRN period in ms */
#define QZS_L2CL_PRN_PERIOD_MS 1500

/** QZSS L2C symbol length [ms] */
#define QZS_L2C_SYMBOL_LENGTH_MS 20

#endif /* ME_CONSTANTS_H */
