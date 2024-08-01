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

#ifndef LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_PHASE_WINDUP_H
#define LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_PHASE_WINDUP_H

#include <pvt_engine/common_data.h>

namespace pvt_engine {

class RtkLibAprioriModelPhaseWindup {
 public:
  RtkLibAprioriModelPhaseWindup() = default;

  void windupcorr(const Eigen::Vector3d &sat_pos,
                  const Eigen::Vector3d &rec_pos, const Eigen::Vector3d &exs,
                  const Eigen::Vector3d &eys, double *phw);
};

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_PHASE_WINDUP_H
