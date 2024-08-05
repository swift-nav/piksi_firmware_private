/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_ELEVATION_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_ELEVATION_H

#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/common_data.h>

namespace pvt_engine {

class AprioriModelElevation : public AprioriModelWithConfig {
 public:
  explicit AprioriModelElevation(const AprioriModelConfiguration &config);
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_ELEVATION_H
