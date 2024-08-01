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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_INTERFACE_H

#include <pvt_engine/common_data.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

class AprioriModelInterface {
 public:
  virtual ~AprioriModelInterface() = default;

  virtual PRC initialize(const AprioriModelConfiguration &config) = 0;

  virtual bool update_config(const AprioriModelConfiguration &config) = 0;

  virtual PRC process(const optional<Eigen::Vector3d> &station_location,
                      InputObservationHandler *observation_handler) = 0;
};

class AprioriModelWithConfig : public AprioriModelInterface {
 public:
  explicit AprioriModelWithConfig(const AprioriModelConfiguration &config)
      : config_(config){};

  ~AprioriModelWithConfig() override = default;

  PRC initialize(const AprioriModelConfiguration &config) override {
    update_config(config);
    return RC_S_OK;
  }

  bool update_config(const AprioriModelConfiguration &config) override {
    config_ = config;
    return false;
  };

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override = 0;

 protected:
  AprioriModelConfiguration config_;
};

// This implements an instance of the apriori model interface that provides a
// default behavior for apriori models that have no config. Any priori model
// that doesn't require specific config can derive from this class and utilize
// the default behavior provided for update_config() and initialize()
class AprioriModelNoConfig : public AprioriModelInterface {
 public:
  ~AprioriModelNoConfig() override = default;
  PRC initialize(__attribute__((unused))
                 const AprioriModelConfiguration &config) override {
    return RC_S_OK;
  }
  bool update_config(__attribute__((unused))
                     const AprioriModelConfiguration &config) override {
    return false;
  }
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override = 0;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_INTERFACE_H
