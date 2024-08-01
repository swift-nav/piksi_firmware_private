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

#ifndef LIBSWIFTNAV_PVT_ENGINE_LABELED_FLOAT_AMBS_H
#define LIBSWIFTNAV_PVT_ENGINE_LABELED_FLOAT_AMBS_H

#include <algorithm>
#include <cassert>
#include <cstdlib>

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/labeled_estimate_diagonal.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sid_set.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

namespace ambiguities {

// An LabeledFloatAmbs container holds corresponding sets of signal
// IDs, floating ambiguity estimates and estimate covariances.
class LabeledFloatAmbs {
 public:
  // Create an empty container.
  LabeledFloatAmbs() : labels_(), ambiguities_(), covariance_(){};

  // Create an `LabeledFloatAmbs` container from the corresponding
  // signal ID `labels`, ambiguity estimate vector `ambs` and estimate
  // covariance `covariance`.  Obviously the first element of `labels`
  // should correspond to the first estimate in `ambs`, and
  // `covariance` and `ambs` should have the same ordering.
  LabeledFloatAmbs(const gnss_signal_t labels[], VectorMaxStateDimd_t ambs,
                   MatrixMaxStateDimd_t covariance);

  bool operator==(const LabeledFloatAmbs &other) const {
    return size() == other.size() && labels_ == other.labels_ &&
           ambiguities_ == other.ambiguities_ &&
           covariance_ == other.covariance_;
  };

  bool operator!=(const LabeledFloatAmbs &other) const {
    return !(*this == other);
  }

  // Populate the given vector of `ambiguities` and corresponding
  // `labels` array and `covariance` matrix with `this` container's
  // estimates corresponding to the signal IDs contained in `sids`.
  // Any signal IDs contained in `indices` but not present in `this`
  // container are ignored.  If `ambiguities` is `nullptr`, an error
  // is returned; otherwise success is returned.
  //
  // `ambiguities` must be a valid pointer.  `covariance` and `labels`
  // are only populated if they are not `nullptr`s.
  PRC index_sids(const SidSet &indices, gnss_signal_t *labels,
                 VectorMaxStateDimd_t *ambiguities,
                 MatrixMaxStateDimd_t *covariance) const;

  // Return a new `LabeledFloatAmbs` object containing the estimates
  // whose signal IDs are present in `indices`.  Any signal IDs
  // contained in `indices` but not present in `this` container are
  // ignored.  The contents of the new object are ordered identically
  // to in `this` object, so e.g. searches for the same criterion are
  // stable.
  LabeledFloatAmbs indexed(const SidSet &indices) const;

  // Populate the given vector of `ambiguities` and corresponding
  // `labels` array and `covariance` matrix with `this` container's
  // estimates corresponding to the signal IDs contained in `sids`.
  // If `sids` contains any signals without corresponding estimates in
  // `this` container, an error is returned and `ambiguities,`
  // `labels` and `covariance` are unmodified.  Otherwise,
  // `ambiguities` contains as many estimates as `sids` contains
  // signal IDs, `labels` as many signal IDs as `sids` and
  // `covariance` is a square matrix of the corresponding size and
  // `RC_S_OK` is returned.
  //
  // `ambiguities` must be a valid pointer.  `covariance` and `labels`
  // are only populated if they are not `nullptr`s.
  PRC subset_sids(const SidSet &indices, gnss_signal_t *labels,
                  VectorMaxStateDimd_t *ambiguities,
                  MatrixMaxStateDimd_t *covariance) const;

  // Populate the given array of `labels`, vector of `ambiguities` and
  // corresponding `covariance` matrix with `this` container's
  // estimates.  These are only populated if they are not `nullptr`.
  // The caller is responsible for ensuring that there is enough room
  // for all the signal IDs in `labels`.
  void get_estimates(gnss_signal_t *labels, VectorMaxStateDimd_t *ambiguities,
                     MatrixMaxStateDimd_t *covariance) const;

  // Return the set of GNSS signal IDs for which `this` container
  // holds ambiguity estimates.
  //
  // Note that since a `SidSet` is returned, the ordering of labels
  // will change, and finding the offset into the returned `SidSet`
  // for a signal won't tell you anything about where to find the
  // ambiguity state estimate corresponding to it.
  SidSet get_indices() const { return labels_; };

  // Return the index corresponding to the given signal ID `sid` if
  // `this` container has data for it; otherwise, return an empty
  // object.
  optional<s32> index_of_sid(const gnss_signal_t &sid) const;

  // Return the signal ID corresponding to the given numerical `index`
  // if it's within range for `this` container; otherwise, return an
  // empty object.
  optional<gnss_signal_t> sid_of_index(s32 index) const;

  // Populate `*amb_est` with the ambiguity value corresponding to the
  // given numerical `index`.  If `index` is out of
  // bounds, return an error; otherwise return success.
  PRC amb_est_of_index(s32 index, double *amb_est) const;

  // Return the number of ambiguity estimates `this` container holds.
  s32 size() const { return labels_.size(); };

  // Iterators for LabeledFloatAmbs
  //
  // We only get one `begin()` and `end()` method for standard tools
  // to use for iteration (including range-based for loops,
  // `std::min_element`, `std::find_if` etc.), so we should choose the
  // most general thing over which to iterate that we can.
  // Unfortunately, this is not as simple as storing a pointer; we
  // have a covariance matrix, which has a different number of
  // elements to the other two components.  Furthermore, all the stuff
  // in this object is stored separately, and it's difficult to
  // imagine a sensible scheme for storing them in a combined (and
  // interleaved) way.
  //
  // We can solve the first problem by iterating only over the
  // diagonal of the covariance matrix.
  //
  // We can solve the second problem by providing a "fake" iterator
  // which returns a struct of type `LabeledEstimateDiagonal` when
  // dereferenced.  This struct contains a label, ambiguity state mean
  // and ambiguity state variance corresponding to the elements at the
  // current position of the iterator.
  //
  // Because it would be too much work to provide both const and
  // non-const versions of this iterator (the returned type would have
  // to be different in each case if we wanted modifications to affect
  // the stored data), and also because it doesn't make much sense to
  // update the ambiguity variance without updating the cross-terms of
  // the covariance matrix in the contexts where this is used, only a
  // const_iterator is provided.
  class DiagonalConstIterator {
   public:
    using difference_type = s32;
    using value_type = LabeledEstimateDiagonal;
    using reference = const LabeledEstimateDiagonal &;
    using pointer = const LabeledEstimateDiagonal *;
    using iterator_category = std::forward_iterator_tag;

    explicit DiagonalConstIterator(const LabeledFloatAmbs *mptr)
        : idx_(0), mptr_(mptr){};
    DiagonalConstIterator(const LabeledFloatAmbs *mptr, s32 idx)
        : idx_(idx), mptr_(mptr){};

    bool operator==(const DiagonalConstIterator &other) const {
      return other.mptr_ == mptr_ && other.idx_ == idx_;
    };

    bool operator!=(const DiagonalConstIterator &other) const {
      return !operator==(other);
    };

    DiagonalConstIterator &operator++() {
      idx_++;
      return *this;
    };
    DiagonalConstIterator &operator+=(s32 delta) {
      idx_ += delta;
      return *this;
    };
    DiagonalConstIterator &operator-=(s32 delta) {
      idx_ -= delta;
      return *this;
    };

    DiagonalConstIterator operator+(s32 delta) const {
      return DiagonalConstIterator(mptr_, idx_ + delta);
    };
    DiagonalConstIterator operator-(s32 delta) const {
      return DiagonalConstIterator(mptr_, idx_ - delta);
    };

    const LabeledEstimateDiagonal operator*() const {
      return mptr_->get_idx(idx_);
    };

   private:
    s32 idx_;
    const LabeledFloatAmbs *mptr_;
  };

  // Return a const_iterator pointing at the `LabeledEstimateDiagonal`
  // object corresponding to the first of `this` set's signal ID, mean
  // and variance elements.
  //
  // As with other ambiguity objects, we can only return
  // const_iterators, but the compiler will only form a range-based
  // loop if we provide `begin()` and `end()`, even if the loop
  // variable is const; we leave the user to deal with the error
  // message if a modification is attempted.
  DiagonalConstIterator begin() const { return DiagonalConstIterator(this); }

  // Return a const_iterator pointing past the last of `this` set's
  // signal ID, mean and variance elements.
  //
  // See `begin()` for a note about the constness of these iterators.
  DiagonalConstIterator end() const {
    return DiagonalConstIterator(this, size());
  }

  // Return a const_iterator pointing at the `LabeledEstimateDiagonal`
  // object corresponding to the first of `this` set's signal ID, mean
  // and variance elements.
  DiagonalConstIterator cbegin() const { return DiagonalConstIterator(this); }

  // Return a const_iterator pointing past the last of `this` set's
  // signal ID, mean and variance elements.
  DiagonalConstIterator cend() const {
    return DiagonalConstIterator(this, size());
  }

 private:
  // This constructor gets called for creating new
  // `LabeledFloatAmbs` when we know the ambiguity state is in
  // sorted order by label (e.g. as the output of `subset_sids`).
  // It's private because we don't trust anyone to check this
  // ordering; we only use it internally in `index_sids`.
  LabeledFloatAmbs(const SidSet &indices, VectorMaxStateDimd_t ambiguities,
                   MatrixMaxStateDimd_t covariance)
      : labels_(indices), ambiguities_(ambiguities), covariance_(covariance) {}

  // This is a helper function for `subset_sids()` and `index_sids()`.
  PRC index_sids(const SidSet &indices, gnss_signal_t *signals,
                 VectorMaxStateDimd_t *ambiguities,
                 MatrixMaxStateDimd_t *covariance, bool bail) const;

  // This is a helper function for implementing the iterators; it gets
  // a `LabeledEstimateDiagonal` that combines the elements at the
  // given index.
  LabeledEstimateDiagonal get_idx(s32 idx) const {
    const auto sid = labels_.lookup(idx);
    assert(sid);
    return LabeledEstimateDiagonal(*sid, ambiguities_(idx),
                                   covariance_(idx, idx));
  };

  // The following pieces of information are all stored in the same
  // order.  The order is however they were passed to the constructor.
  // When you index into this object using `index_sids()` or
  // `subset_sids()`, you get the results in the same order as the
  // signal IDs given to those functions.

  // The GNSS signal IDs for each estimate.
  SidSet labels_;
  // The float estimates of carrier-phase ambiguity
  VectorMaxAmbsd_t ambiguities_;
  // The covariance matrix describing the shape of the ambiguity estimates.
  MatrixMaxAmbsd_t covariance_;
};

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_LABELED_FLOAT_AMBS_H
