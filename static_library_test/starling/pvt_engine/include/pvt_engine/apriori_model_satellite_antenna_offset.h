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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_SATELLITE_ANTENNA_OFFSET_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_SATELLITE_ANTENNA_OFFSET_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_apriori_model_satellite_antenna_offset.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_antenna.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/satellite_attitude.h>

namespace pvt_engine {

class AprioriModelSatelliteAntennaOffset
    : public AprioriModelUsingCommonConfig {
 public:
  explicit AprioriModelSatelliteAntennaOffset(
      const CommonAprioriModelConfiguration &config);
  PRC initialize(const CommonAprioriModelConfiguration &config) override;
  bool update_config(const CommonAprioriModelConfiguration &config) override;
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;

  void add_satellite_apc_data(const pvt_engine::SatellitePCVMap &satellite_apc);

 private:
  optional<Eigen::Vector3d> get_if_offset(
      const pvt_common::containers::MapElement<const ObservationIdentifier,
                                               Observation> &observation,
      const pcv_t<PCSatellite> &pcv, const SatelliteBodyVectors &sat_body_vec);

  optional<SatellitePCVMap> pcvs_;
  CommonTides sunmoon_;
  SatelliteAttitude satellite_attitude_;
  CommonNonOTLConfiguration config_;
  bool attempted_to_read_pcvs_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_SATELLITE_ANTENNA_OFFSET_H
