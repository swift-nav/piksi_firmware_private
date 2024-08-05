/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PVT_ENGINE_PROTECTION_LEVEL_MANAGER_H
#define STARLING_PVT_ENGINE_PROTECTION_LEVEL_MANAGER_H

#include <pvt_engine/ambiguity_set.h>
#include <pvt_engine/araim/araim_observation_data.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/filter_manager.h>
#include <pvt_engine/observation_handler.h>

#include <pvt_engine/araim/fault_mode.h>

namespace pvt_engine {

class ProtectionLevelManager {
 public:
  explicit ProtectionLevelManager(const MasterConfiguration &master_config,
                                  const CommonData &common_data);
  ~ProtectionLevelManager();

  PRC initialize(const MasterConfiguration &master_config);

  bool update_config(const MasterConfiguration &master_config);

  PRC process(const FilterObservationHandler &obs_handler,
              const ambiguities::TransformedIntegerAmbiguities &ambiguities,
              const Eigen::Vector3d &ref_pos);

  PRC get_protection_level_result(FilterResult *result) const;

 private:
  araim::AraimObservationData preprocess_obs(
      const FilterObservationHandler &obs_handler,
      const ambiguities::TransformedIntegerAmbiguities &ambiguities,
      const Eigen::Vector3d &linearization_point);

  ProtectionLevelConfiguration config_;
  ObservationProcessModel obs_proc_model_;
  optional<ProtectionLevelResult> pl_result_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_PROTECTION_LEVEL_MANAGER_H
