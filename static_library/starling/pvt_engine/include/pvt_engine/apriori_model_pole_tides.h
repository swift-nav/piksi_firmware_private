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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_POLE_TIDES_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_POLE_TIDES_H

#include <pvt_engine/apriori_model_erp_interface.h>

namespace pvt_engine {

class AprioriModelPoleTides : public AprioriModelErpInterface {
 public:
  explicit AprioriModelPoleTides(const CommonAprioriModelConfiguration &config);

  PRC initialize(const CommonAprioriModelConfiguration &config) override {
    return AprioriModelErpInterface::initialize(config);
  }

  bool update_config(const CommonAprioriModelConfiguration &config) override {
    return AprioriModelErpInterface::update_config(config);
  }

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_POLE_TIDES_H
