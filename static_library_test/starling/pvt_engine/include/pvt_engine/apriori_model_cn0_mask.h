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

#ifndef LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_CN0_MASK_H
#define LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_CN0_MASK_H

#include <pvt_engine/apriori_model_interface.h>
#include <pvt_engine/common_data.h>
#include <swiftnav/macros.h>

namespace pvt_engine {

class AprioriModelCn0Mask : public AprioriModelUsingFilterConfig {
 public:
  explicit AprioriModelCn0Mask(
      const FilterManagerAprioriModelConfiguration &config);

  PRC initialize(const FilterManagerAprioriModelConfiguration &config) override;

  bool update_config(
      const FilterManagerAprioriModelConfiguration &config) override;

  PRC process(
      SWIFT_ATTR_UNUSED const optional<Eigen::Vector3d> &station_location,
      InputObservationHandler *observation_handler) override;

 private:
  pvt_common::containers::Map<FREQUENCY, u8, MAX_FREQUENCY>
      code_phase_cn0_mask_;
  pvt_common::containers::Map<FREQUENCY, u8, MAX_FREQUENCY>
      measured_doppler_cn0_mask_;
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_APRIORI_MODEL_CN0_MASK_H
