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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_TROPOSPHERE_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_TROPOSPHERE_H

#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/troposphere_model_gpt2.h>
#include <pvt_engine/troposphere_model_modified_hopfield.h>
#include <pvt_engine/troposphere_model_unb3m.h>

namespace pvt_engine {

class AprioriModelTroposphere : public AprioriModelUsingCommonConfig {
 public:
  explicit AprioriModelTroposphere(
      const CommonAprioriModelConfiguration &config);

  AprioriModelTroposphere(const AprioriModelTroposphere &other);

  AprioriModelTroposphere &operator=(const AprioriModelTroposphere &other);

  PRC initialize(const CommonAprioriModelConfiguration &config) override;
  bool update_config(const CommonAprioriModelConfiguration &config) override;
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;
  PRC correct_one_station(const Eigen::Vector3d &station_position_ecef,
                          const Eigen::Vector3d &station_position_llh,
                          const Observation &obs, double *correction);

  void set_tropo_model(TROPO_MODEL tropo_model);

 private:
  TroposphereModelInterface *model_;

  TroposphereModelModifiedHopfield modified_hopfield_model_;
  TroposphereModelUNB3m unb3m_model_;
  TroposphereModelGPT2 gpt2_model_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_TROPOSPHERE_H
