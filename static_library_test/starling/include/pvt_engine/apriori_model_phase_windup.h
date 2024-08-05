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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_PHASE_WINDUP_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_PHASE_WINDUP_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_apriori_model_phase_windup.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/apriori_model_erp_interface.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/satellite_attitude.h>

namespace pvt_engine {

class AprioriModelPhaseWindup : public AprioriModelErpInterface,
                                RtkLibAprioriModelPhaseWindup {
 public:
  explicit AprioriModelPhaseWindup(const AprioriModelConfiguration &config);

  PRC initialize(const AprioriModelConfiguration &config) override;
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;

 private:
  void initialize_data();

  pvt_common::containers::Map<gnss_signal_t, double, NUM_SATS * MAX_FREQUENCY>
      phw_arr_;
  CommonTides sunmoon_;
  SatelliteAttitude satellite_attitude_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_PHASE_WINDUP_H
