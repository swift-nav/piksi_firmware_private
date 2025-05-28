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

#ifndef LIBSWIFTNAV_PVT_ENGINE_VARIANCE_HANDLER_H
#define LIBSWIFTNAV_PVT_ENGINE_VARIANCE_HANDLER_H

#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>

#include <pvt_common/optional.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_handler.h>

namespace pvt_engine {

using weighting_func_t = double (*)(const Observation &,
                                    const Eigen::Vector3d &, const double);

double constant_weighting_func(const Observation &obs,
                               const Eigen::Vector3d &rover_pos, double alpha);
double elevation_weighting_func(const Observation &obs,
                                const Eigen::Vector3d &rover_pos, double alpha);

class VarianceHandler {
 public:
  explicit VarianceHandler(const VarianceHandlerConfiguration &config,
                           const ObsProcModelTypes &models);

  ~VarianceHandler() = default;

  PRC get_single_variance(
      const Eigen::Vector3d &station_position,
      const pvt_common::containers::MapElement<ObservationIdentifier,
                                               Observation> &obs,
      double *R,
      pvt_common::containers::Map<ObservationIdentifier, LabeledObservation,
                                  cMaxFilterObservations> *obs_insight) const;

  void set_weighting_model(WEIGHTING_MODEL weighting_model);

  PRC initialize(const VarianceHandlerConfiguration &config,
                 const ObsProcModelTypes &models);

  bool update_config(const VarianceHandlerConfiguration &config,
                     const ObsProcModelTypes &models);

 private:
  weighting_func_t weighting_func_;
  double lock_time_weighting(const Observation &obs) const;
  double elevation_weighting(const Observation &obs,
                             const Eigen::Vector3d &station_position) const;
  double constellation_type_weighting(const Observation &obs) const;
  double propagation_weighting(const Observation &obs) const;
  double cn0_weighting(const Observation &obs,
                       const bool &half_cycle_slip_ambiguity_resolved) const;
  optional<double> obs_type_weighting(const Observation &obs) const;
  double noise_multipath_weighting(const Observation &obs) const;
  double iono_residual_weighting(const ObservationIdentifier &obs_id,
                                 const Observation &obs,
                                 const double &elw) const;
  double tropo_residual_weighting(const ObservationIdentifier &obs_id,
                                  const Observation &obs,
                                  const double &elw) const;
  double orbit_clock_residual_variance_weighting(
      const ObservationIdentifier &obs_id, const Observation &obs) const;

  double iono_correction_variance(
      const pvt_engine::applied_model_bitfield &applied_model) const;
  double tropo_correction_variance(
      const pvt_engine::applied_model_bitfield &applied_model) const;
  double broadcast_orbit_clock_correction_variance(
      const Observation &obs) const;

  VarianceHandlerConfiguration config_;
  ObsProcModelTypes models_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_VARIANCE_HANDLER_H
