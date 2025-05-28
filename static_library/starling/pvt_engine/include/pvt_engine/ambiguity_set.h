/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_AMBIGUITY_SET_H
#define STARLING_AMBIGUITY_SET_H

#include <pvt_engine/cross_validate.h>
#include <pvt_engine/filter.h>
#include <pvt_engine/observation_handler.h>
#include <starling/build/config.h>

namespace pvt_engine {

constexpr s32 EPOCHS_TO_BUFFER = cMaxObsBufferSize;

struct CombinedAmbiguity {
  CombinedAmbiguity() : untransformed_ambiguity_sids_(), amb_(0.0), var_(0.0) {}
  CombinedAmbiguity(
      const pvt_engine::ambiguities::SidVector &untransformed_ambiguity_sids,
      double amb, double var)
      : untransformed_ambiguity_sids_(untransformed_ambiguity_sids),
        amb_(amb),
        var_(var) {}
  pvt_engine::ambiguities::SidVector untransformed_ambiguity_sids_;
  double amb_;
  double var_;
};

using CombinedAmbiguityVector =
    pvt_common::containers::StaticVector<CombinedAmbiguity,
                                         pvt_engine::cMaxAmbiguities>;

class AmbiguitySet {
 public:
  explicit AmbiguitySet(const AmbiguityManagerConfiguration &amb_set_config);
  AmbiguitySet(const AmbiguitySet &other);

  virtual ~AmbiguitySet() = default;

  AmbiguitySet &operator=(const AmbiguitySet &other);

  PRC initialize(const AmbiguityManagerConfiguration &amb_set_config);

  bool update_config(const AmbiguityManagerConfiguration &amb_set_config);

  // Get a set of validated integer ambiguities if they're available.
  // If they are, `validated_ambiguities` will contain them when
  // success is returned.  If no validated integers are available,
  // failure is returned.
  PRC get_ambiguities(
      ambiguities::TransformedIntegerAmbiguities *validated_ambiguities) const;

  // Make sure the ambiguity set only knows about signals
  // currently known to be valid.
  //
  // If we have lost lock on or entirely dropped a signal, it is
  // invalid and must be removed from the ambiguity history and
  // current validated set.  This uses the contents of the observation
  // handler along with the `loss_of_lock` flag for each observation
  // to invalidate the appropriate signals.
  virtual s32 refine_ambiguities(const FilterObservationHandler &obs_handler);

  // Remove the ambiguities corresponding to the given signal IDs.
  virtual s32 remove_ambiguities(const gps_time_t &time,
                                 const ambiguities::SidSet &ambs_to_drop);

  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
  compute_proportion_of_fixed_ambiguities(const Filter &filter) const;

  CombinedAmbiguityVector get_transformed_float_ambiguities(
      const ambiguities::AmbiguitiesAndCovariances &float_ambs) const;

  // Retrieve the most recent insight metadata.
  ambiguities::ValidationInsight get_insight() const { return insight_; }

  bool has_sid(const gnss_signal_t &sid) const {
    return validated_ambiguities_.get_untransformed_ambiguity_sids().contains(
        sid);
  }

 protected:
  ambiguities::SidSet get_continuous_lock_sids(
      const FilterObservationHandler &obs_handler);
  s32 internal_refine(const optional<gps_time_t> &epoch_time,
                      const ambiguities::SidSet &continuous_lock);

  void copy_validated_ambs_and_update_for_subsequent_LoLs(
      const AmbiguitySet &other);

  AmbiguityManagerConfiguration amb_set_config_;
  ambiguities::TransformedIntegerAmbiguities validated_ambiguities_;
  optional<gps_time_t> last_refined_epoch_;
  pvt_common::containers::Map<gnss_signal_t, gps_time_t,
                              cNumSat * MAX_FREQUENCY>
      last_reset_;
  pvt_common::containers::Map<gps_time_t, ambiguities::SidSet, EPOCHS_TO_BUFFER>
      continuous_lock_epochs_;
  ambiguities::ValidationInsight insight_;

 private:
  pvt_common::containers::Map<constellation_t, MatrixMaxAmbss32_t,
                              CONSTELLATION_COUNT>
  get_transformation_matrix_by_constellation(
      const ambiguities::AmbiguitiesAndCovariances &float_ambs) const;
};

}  // namespace pvt_engine

#endif  // STARLING_AMBIGUITY_SET_H
