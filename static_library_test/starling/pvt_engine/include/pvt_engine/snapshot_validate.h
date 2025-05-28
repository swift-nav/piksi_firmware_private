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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SNAPSHOT_VALIDATE_H
#define LIBSWIFTNAV_PVT_ENGINE_SNAPSHOT_VALIDATE_H

#include <pvt_engine/ambiguity_lambda_interface.h>
#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/degrees_of_freedom.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sum_squared_normalized_residuals.h>

namespace pvt_engine {

namespace ambiguities {

class SnapshotValidator {
 public:
  explicit SnapshotValidator(const LAMBDAValidationConfiguration &config,
                             const obs_filters::CodeSet codes_to_fix)
      : config_(config), codes_to_fix_(codes_to_fix) {
    assert(config_.num_candidates <= cMaxIntegerAmbiguitySets);
  }

  void update_config(const LAMBDAValidationConfiguration &validation_config);

  PRC validate_integers(
      const AmbiguitiesAndCovariances &float_ambs,
      const pvt_engine::DoF_container &DoF,
      const SumSquaredNormalizedResiduals &sum_squared_normalized_residuals,
      TransformedIntegerAmbiguities *validated,
      AmbiguitySearchResult *insight) const;

  PRC drop_n(const AmbiguitiesAndCovariances &float_ambs,
             const TransformedIntegerAmbiguities &full_results,
             const IndexSet &indices_to_drop,
             AmbiguitySearchResult *insight) const;

 private:
  PRC search_ambiguities_individually(
      const AmbiguitiesAndCovariances &float_ambs, const MatrixMaxAmbss32_t &T,
      TransformedIntegerAmbiguities *validated,
      AmbiguitySearchResult *insight) const;

  PRC search_ambiguities_together(
      const AmbiguitiesAndCovariances &float_ambs,
      const pvt_engine::DoF_container &DoF,
      const double &code_phase_sum_squared_normalized_residuals,
      const pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>
          &constels_to_fix,
      const MatrixMaxAmbss32_t &T, TransformedIntegerAmbiguities *validated,
      AmbiguitySearchResult *insight) const;

  PRC validate_drops(const AmbiguitiesAndCovariances &float_ambs,
                     const TransformedIntegerAmbiguities &full_results,
                     AmbiguitySearchResult *insight) const;

  PRC validate_r_ratio(const double &r_ratio, const optional<double> &DoF,
                       AmbiguitySearchResult *insight) const;

  LAMBDAValidationConfiguration config_;
  obs_filters::CodeSet codes_to_fix_;
};

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SNAPSHOT_VALIDATE_H
