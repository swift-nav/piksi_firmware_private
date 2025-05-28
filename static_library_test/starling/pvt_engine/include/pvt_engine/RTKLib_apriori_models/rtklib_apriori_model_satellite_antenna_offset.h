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
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_SATELLITE_ANTENNA_OFFSET_H
#define LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_SATELLITE_ANTENNA_OFFSET_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_antenna.h>
#include <pvt_engine/common_data.h>

namespace pvt_engine {

Eigen::Vector3d compute_sat_antenna_offset(const Eigen::Vector3d &ex,
                                           const Eigen::Vector3d &ey,
                                           const Eigen::Vector3d &ez,
                                           const pcv_t<PCSatellite> &pcv,
                                           const PCVFrequencyType &frequency);
double compute_sat_antenna_pcv(const Eigen::Vector3d &sat_pos,
                               const Eigen::Vector3d &rec_pos,
                               const pcv_t<PCSatellite> &pcv,
                               const PCVFrequencyType &frequency);
}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_SATELLITE_ANTENNA_OFFSET_H
