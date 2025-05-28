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

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_FAST_CORRECTIONS_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_FAST_CORRECTIONS_H

#include <swiftnav/gnss_time.h>

namespace pvt_engine {

const double cSbasFcScaling = 0.125;
const u8 cSbasFcNumElements = 13;
const u8 cSbasFcDataStartOffset = 4;
const u8 cSbasMixedFcNumElements = 6;
const u8 cSbasMixedFcDataStartOffset = 0;
const u8 cSbasFcPrcSize = 12;
const u8 cSbasFcUdreiSize = 4;

// special values of the UDREI field
const u8 cSbasUdreiNotMonitored = 14;
const u8 cSbasUdreiDoNotUse = 15;

// fast corrections to age out in 60 seconds by default
const double cSbasFastCorrectionTimeoutS = 60;

struct FastCorrection {
  FastCorrection()
      : time_of_applicability(GPS_TIME_UNKNOWN), correction_m(0.0) {}

  gps_time_t time_of_applicability;
  double correction_m;  // correction value, from -256.000 to 255.875 m
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_SBAS_FAST_CORRECTIONS_H
