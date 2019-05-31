/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "board/me_max_channels.h"

#ifndef SWIFTNAV_PLATFORM_SIGNAL_CONFIG_H
#define SWIFTNAV_PLATFORM_SIGNAL_CONFIG_H

/* Platform-specific code support */
#define CODE_GPS_L1CA_SUPPORT 1
#define CODE_GPS_L2C_SUPPORT 1
#define CODE_GPS_L5_SUPPORT 0
#define CODE_GPS_AUX_SUPPORT 0
#define CODE_GLO_L1OF_SUPPORT 1
#define CODE_GLO_L2OF_SUPPORT 1
#define CODE_SBAS_L1CA_SUPPORT 1
#define CODE_QZSS_L1CA_SUPPORT 1
#define CODE_QZSS_L2C_SUPPORT 1
#define CODE_QZSS_L5_SUPPORT 0
#define CODE_BDS2_B1_SUPPORT 1
#define CODE_BDS2_B2_SUPPORT 1
#define CODE_BDS3_B5_SUPPORT 0
#define CODE_GAL_E1_SUPPORT 1
#define CODE_GAL_E5_SUPPORT 0
#define CODE_GAL_E7_SUPPORT 1

#endif
