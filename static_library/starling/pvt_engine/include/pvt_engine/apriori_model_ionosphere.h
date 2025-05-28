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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_IONOSPHERE_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_IONOSPHERE_H

#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/common_data.h>

namespace pvt_engine {

class AprioriModelIonosphere : public AprioriModelUsingFilterConfig {
 public:
  explicit AprioriModelIonosphere(
      const FilterManagerAprioriModelConfiguration &config,
      const CommonData &common_data);

  AprioriModelIonosphere &operator=(const AprioriModelIonosphere &other) {
    use_klobuchar_ = other.use_klobuchar_;
    // common_data_ = other.common_data_
    return *this;
  };

  PRC initialize(const FilterManagerAprioriModelConfiguration &config) final {
    PRC rc = RC_S_OK;
    if (use_klobuchar_ != config.use_klobuchar) {
      rc = RC_E_NOTOK;
    }
    use_klobuchar_ = config.use_klobuchar;
    return rc;
  };

  bool update_config(
      const FilterManagerAprioriModelConfiguration &config) final {
    bool changed = false;
    if (use_klobuchar_ != config.use_klobuchar) {
      changed = true;
    }
    use_klobuchar_ = config.use_klobuchar;
    return changed;
  };

  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;
  void correct_one_station(const gnss_signal_t &sid,
                           const Eigen::Vector3d &station_position_llh,
                           const Observation &obs,
                           const ionosphere_t &iono_params, double *correction);

 private:
  const CommonData &common_data_;
  bool use_klobuchar_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_IONOSPHERE_H
