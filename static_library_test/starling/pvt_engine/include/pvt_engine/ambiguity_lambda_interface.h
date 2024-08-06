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

#ifndef LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_LAMBDA_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_LAMBDA_INTERFACE_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/internal/ambiguity_lambda_interface.h>
#include <pvt_engine/internal/snapshot_validate.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sid_set.h>
#include <pvt_engine/utils.h>

namespace pvt_engine {

namespace ambiguities {

class SdiffsManager {
 public:
  explicit SdiffsManager(const AmbiguitiesAndCovariances &float_ambs,
                         const obs_filters::CodeSet &codes_to_fix)
      : float_ambs_(float_ambs), codes_to_fix_(codes_to_fix) {}
  PRC get_references(
      const LambdaSearchStrategy &search_strategy,
      const LambdaCombinationStrategy &combination_strategy,
      ReferenceIndexMap *references,
      pvt_common::containers::Map<constellation_t, code_t, CONSTELLATION_COUNT>
          *ref_codes) const;
  PRC get_sd_map_with_imposed_references(const ReferenceIndexMap &references,
                                         SDiffToDDiffMap_t *sd_map) const;

 private:
  PRC get_uncombined_references(ReferenceIndexMap *references) const;

  struct WLInfo {
    WLInfo()
        : sat(),
          non_ref_code(),
          ref_code(),
          non_ref_index(),
          ref_index(),
          differenced_ambiguity_value() {}
    WLInfo(const u16 &sat_, const code_t &non_ref_code_,
           const code_t &ref_code_, const s32 &non_ref_index_,
           const s32 &ref_index_, const double &differenced_ambiguity_value_)
        : sat(sat_),
          non_ref_code(non_ref_code_),
          ref_code(ref_code_),
          non_ref_index(non_ref_index_),
          ref_index(ref_index_),
          differenced_ambiguity_value(differenced_ambiguity_value_) {}

    u16 sat;
    code_t non_ref_code;
    code_t ref_code;
    s32 non_ref_index;
    s32 ref_index;
    double differenced_ambiguity_value;
  };

  optional<s32> get_reference(const gnss_signal_t &sid) const;

  void get_undifferenced_widelane_data(
      pvt_common::containers::StaticVector<WLInfo, cMaxAmbiguities> *undiff_WLs,
      pvt_common::containers::Set<code_t, CODE_COUNT> *codes,
      pvt_common::containers::Map<code_t, double, CODE_COUNT> *min_ref_vars)
      const;

  static optional<double> get_mean_squared_error(
      const SdiffsManager::WLInfo &reference_candidate,
      const pvt_common::containers::StaticVector<SdiffsManager::WLInfo,
                                                 cMaxAmbiguities> &undiff_WLs);

  PRC get_widelane_references_lowest_variance(
      ReferenceIndexMap *references,
      pvt_common::containers::Map<constellation_t, code_t, CONSTELLATION_COUNT>
          *ref_codes) const;

  PRC get_widelane_references_lowest_spread(
      ReferenceIndexMap *references,
      pvt_common::containers::Map<constellation_t, code_t, CONSTELLATION_COUNT>
          *ref_codes) const;

  void get_number_of_codes_per_sat(
      const constellation_t &constellation,
      pvt_common::containers::Map<
          SatIdentifier, pvt_common::containers::Set<code_t, MAX_FREQUENCY>,
          MAX_CHANNELS> *all_sat_codes) const;

  AmbiguitiesAndCovariances get_possible_ref_satellites_states(
      const pvt_common::containers::Map<
          SatIdentifier, pvt_common::containers::Set<code_t, MAX_FREQUENCY>,
          MAX_CHANNELS> &all_sat_codes,
      const s32 &num_codes_for_ref_sat) const;

  AmbiguitiesAndCovariances float_ambs_;
  obs_filters::CodeSet codes_to_fix_;
};

class AmbiguityLambdaInterface {  // NOLINT -- clang-tidy shits the bed here
 public:
  /*
   * @param float_ambs float ambiguity states, including sids, state estimates,
   *                   covariance
   * @param codes_to_fix Set of code_t in which integer ambiguity resolution is
   * allowed
   */
  explicit AmbiguityLambdaInterface(const obs_filters::CodeSet &codes_to_fix)
      : codes_to_fix_(codes_to_fix) {}

  PRC run_lambda(s32 num_candidates,
                 const AmbiguitiesAndCovariances &float_ambs,
                 const MatrixMaxAmbss32_t &T,
                 TransformedIntegerAmbiguities results[], double residuals[]);

 private:
  obs_filters::CodeSet codes_to_fix_;
};

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_LAMBDA_INTERFACE_H
