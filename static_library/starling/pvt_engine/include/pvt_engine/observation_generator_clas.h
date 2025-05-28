/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_CLAS_H
#define STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_CLAS_H

#include <swiftnav/gnss_time.h>

#include <pvt_common/eigen_custom.h>
#include <pvt_common/optional.h>
#include <pvt_engine/apriori_model_handler.h>
#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/observation_generator.h>
#include <pvt_engine/satellite_states.h>

namespace pvt_engine {

class ObservationGeneratorClas final : public ObservationGenerator {
 public:
  explicit ObservationGeneratorClas(
      const ObservationGeneratorConfiguration &config,
      const SatelliteInformationHandler &sat_info);

  void update_atmospheric_corrections(const AtmosphericCorrectionsHandler &
                                          new_atmospheric_corrections) override;

  optional<obss_t> generate_virtual_base_obs(
      const Eigen::Vector3d &rover_location, const gps_time_t &gps_time,
      ObsGenFailureInfo *failure_info) override;

 private:
  std::pair<MeanAndVarianceMap, bool> compute_iono_delays(
      const Eigen::Vector3d &rover_ecef, const gps_time_t &gps_time,
      const InputObservationHandler &obs_handler) override;

  std::pair<MeanAndVarianceMap, bool> compute_tropo_delays(
      const Eigen::Vector3d &rover_ecef, const gps_time_t &gps_time,
      const InputObservationHandler &obs_handler) override;

  AtmosphericCorrectionsHandler atmospheric_corrections_handler_;
};

}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_OBSERVATION_GENERATOR_CLAS_H
