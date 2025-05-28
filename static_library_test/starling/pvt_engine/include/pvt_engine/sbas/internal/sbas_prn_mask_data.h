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

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_PRN_MASK_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_PRN_MASK_H

#include <swiftnav/gnss_time.h>

#include <pvt_common/containers/map.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/sbas/sbas.h>

namespace pvt_engine {

using PrnMaskSatSet =
    pvt_common::containers::Map<u8, SatIdentifier, cMaxSbasAugmentedSatellites>;

// Table A-4 PRN Mask Assignments
const u8 GPS_MAX = 37;
const u8 GLO_MAX = 61;
const u8 FUTURE_GNSS_MAX = 119;
const u8 SBAS_MAX = 138;
const u8 MASK_MAX = 210;

const u8 IODP_MAX = 3;
const u16 PRN_MASK_AGE_S = 600;

struct PrnMask {
  PrnMask() : timestamp(GPS_TIME_UNKNOWN), prn_mask() {}

  gps_time_t timestamp;
  PrnMaskSatSet prn_mask;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_PRN_MASK_H
