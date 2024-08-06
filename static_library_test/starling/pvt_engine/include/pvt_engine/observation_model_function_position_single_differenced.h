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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_SINGLE_DIFFERENCED_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_SINGLE_DIFFERENCED_H

#include <pvt_engine/line_of_sight.h>
#include <pvt_engine/observation_model_function_position.h>

namespace pvt_engine {

class ObservationModelFunctionPositionSingleDifferenced
    : public ObservationModelFunctionPosition {
 public:
  ObservationModelFunctionPositionSingleDifferenced();

  void set_data(const Observation &obs, const CommonData &common_data,
                const VectorMaxKinematicStatesd_t &linearization_point,
                const pvt_common::containers::StaticVector<
                    pvt_engine::StateAndModel, cMaxStateDim> &state_labels);

  double operator()(const VectorMaxStateDimd_t &state) const override;

 private:
  struct IntermediateData {
    const Eigen::Vector3d &linearization_position;
    const Eigen::Vector3d &ref_station_pos;
    const Eigen::Vector3d &linearization_baseline;
    const Eigen::Vector3d &average_velocity_estimate;
    const Eigen::Vector3d &instantaneous_velocity_estimate;
    const optional<Eigen::Vector3d> &ref_los;
    const optional<Eigen::Vector3d> &rover_los;
    const VectorMaxStateDimd_t &state;

    IntermediateData(const Eigen::Vector3d &lp, const Eigen::Vector3d &rsp,
                     const Eigen::Vector3d &lb,
                     const Eigen::Vector3d &avgvelest,
                     const Eigen::Vector3d &instvelest,
                     const optional<Eigen::Vector3d> &refl,
                     const optional<Eigen::Vector3d> &rovl,
                     const VectorMaxStateDimd_t &st)
        : linearization_position(lp),
          ref_station_pos(rsp),
          linearization_baseline(lb),
          average_velocity_estimate(avgvelest),
          instantaneous_velocity_estimate(instvelest),
          ref_los(refl),
          rover_los(rovl),
          state(st) {}
  };

  double observe_range_phase(const IntermediateData &point) const;
  double observe_meas_doppler(const IntermediateData &point) const;
  double observe_baseline_mag(const IntermediateData &point) const;

  Eigen::Vector3d current_base_position;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_MODEL_FUNCTION_POSITION_SINGLE_DIFFERENCED_H
