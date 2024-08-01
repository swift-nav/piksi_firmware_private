/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Code extracted from RTKLib, Copyright (C) 2007-2013 by T.TAKASU, All rights
 * reserved.
 */

#ifndef LIBSWIFTNAV_RTKLIB_COMMON_SATELLITE_SYSTEMS_H
#define LIBSWIFTNAV_RTKLIB_COMMON_SATELLITE_SYSTEMS_H

#define SYS_NONE 0x00 /* navigation system: none */
#define SYS_GPS 0x01  /* navigation system: GPS */
#define SYS_SBS 0x02  /* navigation system: SBAS */
#define SYS_GLO 0x04  /* navigation system: GLONASS */
#define SYS_GAL 0x08  /* navigation system: Galileo */
#define SYS_QZS 0x10  /* navigation system: QZSS */
#define SYS_CMP 0x20  /* navigation system: BeiDou */
#define SYS_ALL 0xFF  /* navigation system: all */

#define MINPRNGPS 1  /* min satellite PRN number of GPS */
#define MAXPRNGPS 32 /* max satellite PRN number of GPS */
#define NSATGPS (MAXPRNGPS - MINPRNGPS + 1) /* number of GPS satellites */

#define MINPRNGLO 1  /* min satellite slot number of GLONASS */
#define MAXPRNGLO 24 /* max satellite slot number of GLONASS */
#define NSATGLO (MAXPRNGLO - MINPRNGLO + 1) /* number of GLONASS satellites */

#define MINPRNGAL 1  /* min satellite PRN number of Galileo */
#define MAXPRNGAL 36 /* max satellite PRN number of Galileo */
#define NSATGAL (MAXPRNGAL - MINPRNGAL + 1) /* number of Galileo satellites */

#define MINPRNQZS 193   /* min satellite PRN number of QZSS */
#define MAXPRNQZS 199   /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 189 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS (MAXPRNQZS - MINPRNQZS + 1) /* number of QZSS satellites */

#define MINPRNCMP 1  /* min satellite sat number of BeiDou */
#define MAXPRNCMP 35 /* max satellite sat number of BeiDou */
#define NSATCMP (MAXPRNCMP - MINPRNCMP + 1) /* number of BeiDou satellites */

#define MINPRNSBS 120 /* min satellite PRN number of SBAS */
#define MAXPRNSBS 142 /* max satellite PRN number of SBAS */
#define NSATSBS (MAXPRNSBS - MINPRNSBS + 1) /* number of SBAS satellites */

#define MAXSAT (NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATSBS)

#endif  // LIBSWIFTNAV_RTKLIB_COMMON_SATELLITE_SYSTEMS_H
