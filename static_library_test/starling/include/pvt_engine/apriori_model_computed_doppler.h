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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_COMPUTED_DOPPLER_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_COMPUTED_DOPPLER_H

#include <pvt_engine/apriori_model_interface.h>

namespace pvt_engine {

class AprioriModelComputedDoppler : public AprioriModelWithConfig {
 public:
  explicit AprioriModelComputedDoppler(const AprioriModelConfiguration &config);

  PRC initialize(const AprioriModelConfiguration &config) override;

  bool update_config(const AprioriModelConfiguration &config) override;

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;

 private:
  struct ComputedDopplerInfo {
    double previous_carrier_phase_;
    applied_model_bitfield applied_apriori_corrections_;

    ComputedDopplerInfo(
        const double previous_carrier_phase,
        const applied_model_bitfield applied_apriori_corrections)
        : previous_carrier_phase_(previous_carrier_phase),
          applied_apriori_corrections_(applied_apriori_corrections){};

    ComputedDopplerInfo()
        : previous_carrier_phase_(0.0), applied_apriori_corrections_(0){};
  };

  using ComputedDopplerInfoMap =
      pvt_common::containers::Map<gnss_signal_t, ComputedDopplerInfo,
                                  cMaxInputObservations>;

  ComputedDopplerInfoMap previous_computed_doppler_info_;
  optional<gps_time_t> previous_time_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_COMPUTED_DOPPLER_H
