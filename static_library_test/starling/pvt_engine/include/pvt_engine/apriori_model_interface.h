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
#include <swiftnav/macros.h>

namespace pvt_engine {

class AprioriModelInterface {
 public:
  virtual ~AprioriModelInterface() = default;

  virtual PRC process(const optional<Eigen::Vector3d> &station_location,
                      InputObservationHandler *observation_handler) = 0;
};

class AprioriModelUsingCommonConfig : public AprioriModelInterface {
 public:
  explicit AprioriModelUsingCommonConfig(){};

  ~AprioriModelUsingCommonConfig() override = default;

  virtual PRC initialize(
      const CommonAprioriModelConfiguration &config SWIFT_ATTR_UNUSED) {
    return RC_S_OK;
  }

  virtual bool update_config(
      const CommonAprioriModelConfiguration &config SWIFT_ATTR_UNUSED) {
    return false;
  };

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override = 0;
};

class AprioriModelUsingFilterConfig : public AprioriModelInterface {
 public:
  explicit AprioriModelUsingFilterConfig(){};

  ~AprioriModelUsingFilterConfig() override = default;

  virtual PRC initialize(
      const FilterManagerAprioriModelConfiguration &config SWIFT_ATTR_UNUSED) {
    return RC_S_OK;
  }

  virtual bool update_config(
      const FilterManagerAprioriModelConfiguration &config SWIFT_ATTR_UNUSED) {
    return false;
  };

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override = 0;
};

class AprioriModelNoConfig : public AprioriModelInterface {
 public:
  explicit AprioriModelNoConfig(){};

  ~AprioriModelNoConfig() override = default;

  virtual PRC initialize() { return RC_S_OK; }

  virtual bool update_config() { return false; };

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override = 0;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_INTERFACE_H
