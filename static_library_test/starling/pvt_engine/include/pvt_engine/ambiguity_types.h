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

#ifndef LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_TYPES_H
#define LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_TYPES_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/utils.h>
#include <swiftnav/signal.h>
#include <mapbox/variant.hpp>

namespace pvt_engine {

namespace ambiguities {

// Failure due to not enough signals before attempting LAMBDA
struct AmbiguitySearchFailedMinDDSignals {
  s32 min_signals;  // How many DD signals were needed
  s32 dd_signals;   // How many DD signals were available

  AmbiguitySearchFailedMinDDSignals(s32 min, s32 dd)
      : min_signals(min), dd_signals(dd) {}
  // This default constructor is here so that in the absence of any
  // other information, our validation result will accurately
  // represent that we don't have enough signals to even start..
  AmbiguitySearchFailedMinDDSignals() : min_signals(1), dd_signals(0) {}
};

// Failure due to the ratio test
struct AmbiguitySearchFailedRatio {
  double ratio_threshold;   // Ratio test threshold
  double ratio_test_value;  // Ratio test value

  AmbiguitySearchFailedRatio(double threshold, double fail)
      : ratio_threshold(threshold), ratio_test_value(fail) {}
};

// Failure due to the drop-signal test
struct AmbiguitySearchFailedDrop {
  s32 fixed_ambiguity_dropped_row;  // Which index was dropped when the
  // failure occurred. Note: since we drop
  // rows in the transformation matrix that
  // transforms from SD ambiguities to
  // whatever we search, the row may not
  // correspond to a sid (e.g. widelane ambs)
  s32 full_set_size;  // How many signals were in the full
  // set (before dropping)
  s32 match_size;  // How many signals matched between the
  // full set and the drop-1 set

  AmbiguitySearchFailedDrop(s32 index, s32 full, s32 match)
      : fixed_ambiguity_dropped_row(index),
        full_set_size(full),
        match_size(match) {}
};

// Successful ambiguity search results
struct AmbiguitySearchSuccess {
  s32 num_ambiguities;  // Number of ambiguities validated.
  pvt_common::containers::StaticVector<double, cMaxAmbiguities>
      ratio_test_values;

  explicit AmbiguitySearchSuccess(
      s32 num,
      pvt_common::containers::StaticVector<double, cMaxAmbiguities> &ratios)
      : num_ambiguities(num), ratio_test_values() {
    assert(num > 0);
    ratio_test_values = ratios;
  }
};

// Failure due to not enough ambiguities after matching across
// snapshots
struct AmbiguityCrossValidationFailedMinSnapshotAmbs {
  s32 min_ambs;       // How many snapshot ambs were needed
  s32 snapshot_ambs;  // How many snapshot ambs were available

  AmbiguityCrossValidationFailedMinSnapshotAmbs(s32 min, s32 snapshot)
      : min_ambs(min), snapshot_ambs(snapshot) {}

  // This default constructor is here so that in the absence of any
  // other information, our validation result will accurately
  // represent that we don't have enough signals to even start..
  AmbiguityCrossValidationFailedMinSnapshotAmbs()
      : min_ambs(1), snapshot_ambs(0) {}
};

// Failure when checking against the float filter
struct AmbiguityCrossValidationFailedFloatBaseline {
  double baseline_threshold;  // Max discrepancy between baseline estimates
  double distance;            // Distance between float and fixed baselines

  AmbiguityCrossValidationFailedFloatBaseline(double threshold, double dist)
      : baseline_threshold(threshold), distance(dist) {}
};

// Successful validation results
struct AmbiguityCrossValidationSuccess {
  s32 num_ambiguities;  // Number of ambiguities validated.
  double distance_between_float_and_fixed_baselines;

  explicit AmbiguityCrossValidationSuccess(s32 num, double dist)
      : num_ambiguities(num), distance_between_float_and_fixed_baselines(dist) {
    assert(num > 0);
  }
};

using AmbiguitySearchResult =
    mapbox::util::variant<AmbiguitySearchFailedMinDDSignals,
                          AmbiguitySearchFailedRatio, AmbiguitySearchFailedDrop,
                          AmbiguitySearchSuccess>;

using AmbiguityCrossValidationResult =
    mapbox::util::variant<AmbiguityCrossValidationFailedMinSnapshotAmbs,
                          AmbiguityCrossValidationFailedFloatBaseline,
                          AmbiguityCrossValidationSuccess>;

/**
 * Holds information about how integer validation went on a particular filter
 * update.
 */
struct ValidationInsight {
  AmbiguitySearchResult ambiguity_search_result;
  AmbiguityCrossValidationResult ambiguity_cross_validation_result;
  gps_time_t validation_time;
  ValidationInsight()
      : ambiguity_search_result(),
        ambiguity_cross_validation_result(),
        validation_time(GPS_TIME_UNKNOWN) {}
};

// Define the float and fixed ambiguity vectors as double and integer Eigen
// matrices of one column respectively.
using FloatAmbiguityVector =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxAmbiguities, 1>;
using FixedAmbiguityVector =
    Eigen::Matrix<s32, Eigen::Dynamic, 1, 0, cMaxAmbiguities, 1>;

// Base class for both float and fixed ambiguities - templated on the type to
// allow float ambiguities to be doubles and fixed to be integers
template <typename AmbiguityType>
class Ambiguities {
 public:
  // Define the generic ambiguity type
  using AmbiguityVector =
      Eigen::Matrix<AmbiguityType, Eigen::Dynamic, 1, 0, cMaxAmbiguities, 1>;

  Ambiguities();

  Ambiguities(const SidVector &ambiguity_labels,
              const AmbiguityVector &ambiguities);

  Ambiguities(const Ambiguities &other) = default;

  Ambiguities &operator=(const Ambiguities &rhs) = default;

  // The equality operator doesn't attempt to check if sets are equivalent up to
  // reordering, but simply checks to see if they are equal. This has CPU load
  // advantages.
  bool operator==(const Ambiguities &rhs) const;

  const SidVector &get_untransformed_ambiguity_sids() const;

  const AmbiguityVector &get_ambiguity_values() const;

  // The number of underlying ambiguities (fixed ambiguities will transform to a
  // smaller number, in general, because of differencing)
  s32 untransformed_size() const;

  // Get a set of the sids in the class
  SidSet get_untransformed_ambiguity_sidset() const;

  // Get the index of a particular sid
  optional<s32> index_of_sid(const gnss_signal_t &sid) const;

  // Get the state relating to a particular sid
  optional<AmbiguityType> ambiguity_estimate(const gnss_signal_t &sid) const;

  // Add an offset to the float ambiguities - used to adjust GLO ambs for the
  // code bias
  PRC add_offset_to_float_ambiguities(const gnss_signal_t &sid,
                                      const AmbiguityType &offset);

 protected:
  SidVector untransformed_ambiguity_sids_;
  AmbiguityVector ambiguities_values_;
};

template <typename AmbiguityType>
Ambiguities<AmbiguityType>::Ambiguities()
    : untransformed_ambiguity_sids_(), ambiguities_values_() {}

template <typename AmbiguityType>
Ambiguities<AmbiguityType>::Ambiguities(const SidVector &ambiguity_labels,
                                        const AmbiguityVector &ambiguities)
    : untransformed_ambiguity_sids_(ambiguity_labels),
      ambiguities_values_(ambiguities) {}

// The equality operator doesn't attempt to check if sets are equivalent up to
// reordering, but simply checks to see if they are equal. This has CPU load
// advantages.
template <typename AmbiguityType>
bool Ambiguities<AmbiguityType>::operator==(
    const Ambiguities<AmbiguityType> &rhs) const {
  return (untransformed_ambiguity_sids_ ==
          rhs.get_untransformed_ambiguity_sids())
             .all() &&
         ambiguities_values_.isApprox(rhs.get_ambiguity_values());
}

// Returns a SidSet of the ambiguity labels
template <typename AmbiguityType>
SidSet Ambiguities<AmbiguityType>::get_untransformed_ambiguity_sidset() const {
  return SidSet(untransformed_ambiguity_sids_.data(),
                untransformed_ambiguity_sids_.data() +
                    untransformed_ambiguity_sids_.size());
}

// Returns the index of a sid for use by the functions creating the
// transformation matrix
template <typename AmbiguityType>
optional<s32> Ambiguities<AmbiguityType>::index_of_sid(
    const gnss_signal_t &sid) const {
  const gnss_signal_t *begin = &untransformed_ambiguity_sids_[0];
  const gnss_signal_t *end =
      &untransformed_ambiguity_sids_[untransformed_ambiguity_sids_.size()];
  const auto found = std::find(begin, end, sid);
  if (found == end) {
    detailed_log_warn("%s", prc::message(RC_W_SIGNAL_NOT_PRESENT));
    return {};
  }
  return static_cast<s32>(found - begin);
}

// Returns the state of the sid
template <typename AmbiguityType>
optional<AmbiguityType> Ambiguities<AmbiguityType>::ambiguity_estimate(
    const gnss_signal_t &sid) const {
  optional<s32> index = index_of_sid(sid);
  if (index) {
    return ambiguities_values_[*index];
  }
  return {};
}

template <typename AmbiguityType>
const SidVector &Ambiguities<AmbiguityType>::get_untransformed_ambiguity_sids()
    const {
  return untransformed_ambiguity_sids_;
}

template <typename AmbiguityType>
const typename Ambiguities<AmbiguityType>::AmbiguityVector &
Ambiguities<AmbiguityType>::get_ambiguity_values() const {
  return ambiguities_values_;
}

// The number of underlying (original) ambiguities that relate to the
// ambiguities
template <typename AmbiguityType>
s32 Ambiguities<AmbiguityType>::untransformed_size() const {
  return untransformed_ambiguity_sids_.size();
}

// Adds an offset to an ambiguity state (used to apply the GLONASS phase biases)
template <typename AmbiguityType>
PRC Ambiguities<AmbiguityType>::add_offset_to_float_ambiguities(
    const gnss_signal_t &sid, const AmbiguityType &offset) {
  optional<s32> index = index_of_sid(sid);
  if (index) {
    ambiguities_values_[*index] += offset;
    return RC_S_OK;
  }

  detailed_log_warn("%s", prc::message(RC_W_SIGNAL_NOT_PRESENT));
  return RC_W_SIGNAL_NOT_PRESENT;
}

// Define these classes for simplicity
using FloatAmbiguityStates = Ambiguities<double>;
using IntegerAmbiguityStates = Ambiguities<s32>;

// Float ambiguities class used to extract states and variances from the filter
// and pass them around to the ambiguity fixing classes
class AmbiguitiesAndCovariances : public FloatAmbiguityStates {
 public:
  AmbiguitiesAndCovariances();

  AmbiguitiesAndCovariances(const SidVector &ambiguity_labels,
                            const FloatAmbiguityVector &ambiguities,
                            const MatrixMaxAmbsd_t &covariances);
  AmbiguitiesAndCovariances(const AmbiguitiesAndCovariances &other) = default;

  AmbiguitiesAndCovariances &operator=(const AmbiguitiesAndCovariances &rhs) =
      default;

  // The equality operator doesn't attempt to check if sets are equivalent up to
  // reordering or rotation, but simply checks to see if they are equal. This
  // has CPU load advantages.
  bool operator==(const AmbiguitiesAndCovariances &rhs) const;

  const MatrixMaxAmbsd_t &get_ambiguity_covariances() const;

  // Remove all but the desired sids from the class
  AmbiguitiesAndCovariances indexed(const SidSet &sids_to_keep) const;

  // Return the sid with the lowest variance
  optional<gnss_signal_t> get_lowest_variance_sid() const;

 protected:
  MatrixMaxAmbsd_t ambiguity_covariances_;

 private:
  // Return the index of the sid with the lowest variance
  optional<s32> get_lowest_variance_index() const;

  // Return the index of the sid with the highest variance
  optional<s32> get_highest_variance_index() const;
};

// Fixed ambiguity class to hold the original ambiguity labels, the
// transformation matrix from the original ambiguities to the fixed ones and the
// fixed ambiguities
class TransformedIntegerAmbiguities : public IntegerAmbiguityStates {
 public:
  TransformedIntegerAmbiguities();

  TransformedIntegerAmbiguities(
      const SidVector &ambiguity_labels,
      const FixedAmbiguityVector &ambiguities,
      const MatrixMaxAmbss32_t &transformation_matrix);

  TransformedIntegerAmbiguities(const TransformedIntegerAmbiguities &other) =
      default;

  TransformedIntegerAmbiguities &operator=(
      const TransformedIntegerAmbiguities &rhs) = default;

  // The equality operator doesn't attempt to check if sets are equivalent up to
  // reordering or rotation, but simply checks to see if they are equal. This
  // has CPU load advantages.
  bool operator==(const TransformedIntegerAmbiguities &rhs) const;

  const MatrixMaxAmbss32_t &get_transformation_matrix() const;

  // The number of transformed (i.e. fixed) ambiguities after the transformation
  // has been applied
  s32 transformed_size() const;

  // Remove all the sids that weren't observed, rotating to minimize the number
  // of dropped ambiguities
  void refine_untransformed_sids(const SidSet &observed);

  // Remove all the sids that are indicate, rotating to minimize the number of
  // dropped ambiguities
  void drop_untransformed_sids(const SidSet &to_remove);

  void drop_rows(const pvt_common::containers::Set<s32, cMaxAmbiguities>
                     &row_indices_to_drop);

  // Intersect with the other set, if only_equal_ambs is true, ambiguities whose
  // values don't match are dropped; otherwise all are keep
  void intersect(const TransformedIntegerAmbiguities &other,
                 bool only_equal_ambs);

  // Merge the other set into the current set
  void merge(const TransformedIntegerAmbiguities &other);

  // Get the code type sets that each transformed ambiguity corresponds to (e.g.
  // L1/L2 widelane ambiguities would each be the L1, L2 set)
  pvt_common::containers::StaticVector<obs_filters::CodeSet, cMaxAmbiguities>
  get_underlying_codes() const;

  // Transform the first and second sets to the same basis in preparation for
  // intersection.
  static void transform_to_intersection(
      TransformedIntegerAmbiguities *first,
      TransformedIntegerAmbiguities *second,
      MatrixMaxAmbss32_t *this_trans_to_intersection_ = nullptr);

  // Align the untransformed ambiguities of the two sets so the maths is
  // operating on the same basis
  static void align_untransformed_ambiguities(
      TransformedIntegerAmbiguities *first,
      TransformedIntegerAmbiguities *second);

  // Rotates other to include eveything in other that is not in the intersection
  // of it and this
  bool rotate_to_not_intersection(TransformedIntegerAmbiguities *other) const;

  // Drop labels that aren't referenced in the transformation matrix
  void remove_orphaned_labels();

  // Just add in the linearly independent rows of other
  void add_linearly_independent_ambs(
      const TransformedIntegerAmbiguities &other);

  // TODO(STAR-922): This reorder function was made public to simplfy the PL obs
  //  preprocessing. Is this alright to have as a public function??

  // Reorder the ambiguities for ease of manipulation. If keep_orphans is true,
  // no screening out of unreferenced ambiguities is conducted; otherwise, all
  // ambiguities not referenced in the transformation matrix are dropped
  void reorder(const SidVector &new_ambiguity_labels);

 private:
  // Add the labels in new_untransformed_sids into the class
  void append_missing_untransformed_sids(
      const ambiguities::SidSet &new_untransformed_sids);

  // Transform by the trans_to_intersection matrix. If keep_union is set, the
  // matrix is extended to a square matrix by addition of linearly independent
  // rows in order not to drop ambiguities
  void transform_by(const MatrixMaxAmbss32_t &trans_to_intersection);

  void perform_row_reduction_on_transformation();

  MatrixMaxAmbss32_t transformation_matrix_;
};

MatrixMaxAmbss32_t get_transformation_to_minimize_number_of_differing_rows(
    const FixedAmbiguityVector &first_ambiguity_values,
    const FixedAmbiguityVector &second_ambiguity_values,
    const MatrixMaxAmbss32_t &input_transformation_matrix);

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_TYPES_H
