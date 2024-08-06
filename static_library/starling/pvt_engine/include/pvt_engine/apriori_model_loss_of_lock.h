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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_LOSS_OF_LOCK_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_LOSS_OF_LOCK_H

#include <pvt_engine/apriori_model_interface.h>
#include <starling/build/config.h>

namespace pvt_engine {

class AprioriModelLossOfLock : public AprioriModelNoConfig {
 public:
  AprioriModelLossOfLock();

  PRC initialize() override;

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;

 private:
  bool was_lock_lost(const optional<double> &delta_t,
                     const optional<double> &prev_lock_time,
                     const Observation &this_obs) const;

  void reinitialize_lock_times();

  void store_lock_times(const InputObservationHandler &observation_handler,
                        const gps_time_t &this_epoch);

  double calc_delta_time(const gps_time_t &this_epoch) const;

  optional<double> compute_effective_lock_duration(
      const gnss_signal_t &sid, OBSERVATION_TYPE obs_type) const;

  bool is_time_matched(
      const InputObservationHandler &observation_handler) const;

  double get_lock_time(const Observation &obs) const;

  optional<gps_time_t> last_epoch_time_;
  pvt_common::containers::Map<gnss_signal_t, double, cMaxInputObservations>
      phase_lock_times_;
  pvt_common::containers::Map<gnss_signal_t, double, cMaxInputObservations>
      code_lock_times_;
  pvt_common::containers::Map<gnss_signal_t, double, cMaxInputObservations>
      measured_doppler_lock_times_;
  pvt_common::containers::Map<gnss_signal_t, gps_time_t, cNumSignals>
      last_loss_of_lock_times_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_LOSS_OF_LOCK_H
