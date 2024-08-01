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

#ifndef LIBSWIFTNAV_PVT_ENGINE_CROSS_VALIDATE_H
#define LIBSWIFTNAV_PVT_ENGINE_CROSS_VALIDATE_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sid_set.h>

namespace pvt_engine {

namespace ambiguities {

// This is the abstract base class for the float-baseline comparison
// functor needed to cross-validate integers across snapshots.
//
// The cross-validation algorithm needs to know, for possibly multiple
// sets of integers in a single run, how the baseline estimate
// resulting from a set of integers compares to that from the float
// filter.  We would be happy to accept a function pointer here.
// However:
//
//  - this computation has to combine some time-dependent state
//    (i.e. the current float filter state) with some functionality
//    (the baseline calculation), so we end up needing something like
//    a closure.
//
//  - lambdas that capture cannot become function pointers, and the
//    type of lambdas that do capture changes depending on what they
//    capture; however, here in the cross-validation algorithm, we
//    shouldn't have to know what is being captured.
//
//  - you can't pass pointers to member functions unless they're
//    static, which prevents something like a closure
//
//  - this "function handle" of whatever type has to be passed through
//    several layers of objects, so even though requiring a "helper"
//    argument like we do for some ambiguity structures is explicit
//    and easy to follow in direct usage, it's gross to add it to so
//    many layers.
//
// The solution is to define the IntegerPositionComparison interface
// and take any implementation as the argument to the cross-validation
// algorithm.
class IntegerPositionComparison {
 public:
  PRC operator()(const TransformedIntegerAmbiguities &ambs,
                 double *distance) const {
    return do_function(ambs, distance);
  };
  virtual ~IntegerPositionComparison() = default;

 private:
  virtual PRC do_function(const TransformedIntegerAmbiguities &ambs,
                          double *distance) const = 0;
};

struct CodeSetCount {
  CodeSetCount() : code_set_(), count_(0), conflicts_(0) {}
  CodeSetCount(const ambiguities::CodeSet &code_set, const s32 &count,
               const s32 &conflicts)
      : code_set_(code_set), count_(count), conflicts_(conflicts) {}

  ambiguities::CodeSet code_set_;
  s32 count_;
  s32 conflicts_;
};

// This device contains historical results from a snapshot filter and
// compares integers across time windows to determine whether they are
// trustworthy enough to be used in the fixed-integer RTK solution.
class CrossValidator {  // NOLINT -- clang-tidy shits the bed here
 public:
  explicit CrossValidator(const CrossValidationConfiguration &config)
      : config_(config), snapshot_index_(0), snapshots_() {
    assert(config_.n_snapshots <= cMaxSnapshots);
    assert(config_.n_snapshots > 0);
  }

  bool update_config(const CrossValidationConfiguration &config);

  // Cross-validate the snapshots contained in `this` object and
  // populate the output `validated` with a validated set of integers.
  // If validation fails for any reason, an error is returned.
  //
  // `compare_to_float_f` should be a functor that, given an
  // AmbiguityMap of integers, returns the 2-norm distance between the
  // baseline computed from those integers and the baseline estimated
  // as the float filter's state.  To ensure this has the right
  // interface, the functor should subclass
  // `IntegerPositionComparison`.  See the test suite for a simple
  // example.
  //
  // The validation process checks the previous ambiguity values given
  // in `prev_ambs` and attempts to combine them with the results of
  // the snapshot algorithm.  These ambiguity values must go through
  // the same time-dependent validation process as any new values from
  // the snapshots, even if no new ambiguity values are combined with
  // them.  This means that if `cross_validate` does not return
  // successfully, the previous ambiguities can no longer be
  // considered valid.
  PRC cross_validate(const TransformedIntegerAmbiguities &prev_ambs,
                     const IntegerPositionComparison &compare_to_float_f,
                     TransformedIntegerAmbiguities *validated,
                     ValidationResult *insight) const;

  // Add the validated integer output of a snapshot filter contained
  // in `ambs` to `this` object's snapshot state.  These integers will
  // be used for comparisons across independent time windows, which
  // help us determine which integers we trust enough to use for a
  // fixed solution.
  void add_snapshot_results(const TransformedIntegerAmbiguities &ambs);

  // Return the configuration of `this` object for examination.
  const CrossValidationConfiguration &get_config() const { return config_; }

  // Refine the snapshot history to contain only ambiguities corresponding to
  // signals contained in the index set `observed`.  Any ambiguities
  // present in snapshot history but not in `observed` will be removed on
  // return.  Any ambiguities present in `observed` but not contained in
  // the snapshot history set are ignored.
  void refine(const SidSet &observed);

  // Refine the snapshot history, removing all ambiguities
  // corresponding to signals contained in the index set `observed`.
  // Any ambiguities present in `to_drop` but not contained in the
  // snapshot history set are ignored.
  void drop(const SidSet &to_remove);

 private:
  PRC validate_number(const TransformedIntegerAmbiguities &ambs,
                      ValidationResult *insight) const;

  PRC validate_new_integers(const TransformedIntegerAmbiguities &ambs,
                            ValidationResult *insight) const;

  PRC validate_float_distance(
      const TransformedIntegerAmbiguities &ambs,
      const IntegerPositionComparison &compare_to_float_f,
      ValidationResult *insight) const;

  PRC validate_final_integers(
      const TransformedIntegerAmbiguities &ambs,
      const IntegerPositionComparison &compare_to_float_f,
      ValidationResult *insight) const;

  PRC validate_previous(const TransformedIntegerAmbiguities &prev_ambs,
                        const IntegerPositionComparison &compare_to_float_f,
                        TransformedIntegerAmbiguities *validated,
                        ValidationResult *insight) const;

  void merge_with_previous(const TransformedIntegerAmbiguities &previous,
                           TransformedIntegerAmbiguities *current) const;

  bool ok_to_merge(const CodeSetCount &code_type) const;

  CrossValidationConfiguration config_;
  s32 snapshot_index_;
  ambiguities::TransformedIntegerAmbiguities snapshots_[cMaxSnapshots];
};

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_CROSS_VALIDATE_H
