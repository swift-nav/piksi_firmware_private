/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef LIBSWIFTNAV_PVT_ENGINE_PREPROCESSING_INSIGHT_H
#define LIBSWIFTNAV_PVT_ENGINE_PREPROCESSING_INSIGHT_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation.h>

namespace pvt_engine {

class PreprocessingInsights {
 public:
  PreprocessingInsights() : sats_without_SSR_corrections_() {}

  const SatIdSet &get_sats_without_SSR_corrections() const {
    return sats_without_SSR_corrections_;
  }

  void set_obs_without_SSR_corrections(
      const InputObservationIdSet &obs_without_SSR_corrections) {
    sats_without_SSR_corrections_ = SatIdSet();
    for (const auto &ob_without_SSR_corrections : obs_without_SSR_corrections) {
      sats_without_SSR_corrections_.add(
          pvt_engine::SatIdentifier(ob_without_SSR_corrections.sid));
    }
  }

 private:
  SatIdSet sats_without_SSR_corrections_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_PREPROCESSING_INSIGHT_H
