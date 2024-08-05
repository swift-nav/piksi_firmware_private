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

#ifndef LIBSWIFTNAV_APRIORI_MODEL_GENERAL_RELATIVITY_H
#define LIBSWIFTNAV_APRIORI_MODEL_GENERAL_RELATIVITY_H

#include <pvt_engine/apriori_model_interface.h>

namespace pvt_engine {
class AprioriModelGeneralRelativity : public AprioriModelWithConfig {
 public:
  explicit AprioriModelGeneralRelativity(
      const AprioriModelConfiguration &config);
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_APRIORI_MODEL_GENERAL_RELATIVITY_H
