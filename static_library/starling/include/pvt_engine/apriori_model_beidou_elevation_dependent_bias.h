/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_BEIDOU_ELEVATION_DEPENDENT_BIAS_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_BEIDOU_ELEVATION_DEPENDENT_BIAS_H

#include <pvt_engine/apriori_model_interface.h>

namespace pvt_engine {

constexpr s32 ELEV_BINS = 10;

// Correction values obtained from L. Wanninger, S. Beer; BeiDou
// satellite-induced code pseudorange variations: diagnosis and therapy, GPS
// Solutions, September 2014
constexpr double ELEV_IGSO[MAX_FREQUENCY][ELEV_BINS] = {
    {-0.55, -0.40, -0.34, -0.23, -0.15, -0.04, 0.09, 0.19, 0.27, 0.35},
    {-0.71, -0.36, -0.33, -0.19, -0.14, -0.03, 0.08, 0.17, 0.24, 0.33},
    {-0.27, -0.23, -0.21, -0.15, -0.11, -0.04, 0.05, 0.14, 0.19, 0.32}};
constexpr double ELEV_MEO[MAX_FREQUENCY][ELEV_BINS] = {
    {-0.47, -0.38, -0.32, -0.23, -0.11, 0.06, 0.34, 0.69, 0.97, 1.05},
    {-0.40, -0.31, -0.26, -0.18, -0.06, 0.09, 0.28, 0.48, 0.64, 0.69},
    {-0.22, -0.15, -0.13, -0.10, -0.04, 0.05, 0.14, 0.27, 0.36, 0.47}};

class AprioriModelBeidouElevationDependentBias : public AprioriModelWithConfig {
 public:
  explicit AprioriModelBeidouElevationDependentBias(
      const AprioriModelConfiguration &config);
  PRC process(const optional<Eigen::Vector3d> &station_location,
              InputObservationHandler *observation_handler) override;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_BEIDOU_ELEVATION_DEPENDENT_BIAS_H
