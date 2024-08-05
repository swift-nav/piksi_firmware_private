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

#ifndef LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_SOLID_EARTH_TIDES_H
#define LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_SOLID_EARTH_TIDES_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/common_data.h>

namespace pvt_engine {

class RtklibAprioriModelSolidEarthTides {
 public:
  explicit RtklibAprioriModelSolidEarthTides();

  void tide_disp(const gps_time_t &time, const Eigen::Vector3d &rec_pos,
                 const ERPHandler &erp, Eigen::Vector3d *displacement);

 private:
  CommonTides sunmoon_;
};

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_SOLID_EARTH_TIDES_H
