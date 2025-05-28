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

#ifndef LIBSWIFTNAV_PVT_ENGINE_TAGGED_FILTER_DATA_H
#define LIBSWIFTNAV_PVT_ENGINE_TAGGED_FILTER_DATA_H

#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>

#include <pvt_engine/observation_model_interface.h>
#include <pvt_engine/untagged_filter_data.h>

namespace pvt_engine {

class TaggedFilterData : public UntaggedFilterData {
 public:
  TaggedFilterData();

  // allow construction if we already have the data
  TaggedFilterData(
      const FilterObservationIdVector &obs_ids, const VectorMaxObsd_t &y,
      const MatrixMaxObsByStatesd_t &H,
      const optional<const pvt_common::containers::StaticVector<
          ObservationModelFunctionRow, cMaxFilterObservations> &> &h_func,
      const VectorMaxObsd_t &R,
      const pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
          &state_labels);
  TaggedFilterData &operator=(const TaggedFilterData &rhs) = default;
  TaggedFilterData(const TaggedFilterData &other) = default;
  TaggedFilterData &operator=(TaggedFilterData &&rhs) = default;  // NOLINT
  TaggedFilterData(TaggedFilterData &&other) = default;           // NOLINT
  ~TaggedFilterData() override;

  void clear() {
    obs_ids_.clear();
    state_labels_.clear();
    UntaggedFilterData::clear();
  }
  void resize(
      const s32 &rows, const s32 &cols,
      const pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
          &state_labels) {
    assert(state_labels.size() == cols);
    obs_ids_.resize(rows);
    state_labels_ = state_labels;
    UntaggedFilterData::resize(rows, cols);
  }
  void conservativeResize(
      const s32 &rows, const s32 &cols,
      const pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
          &state_labels) {
    (void)state_labels;

    assert(cols == state_labels_.size());
    assert(std::equal(state_labels_.begin(), state_labels_.end(),
                      state_labels.begin(), state_labels.end()));
    obs_ids_.conservativeResize(rows);
    state_labels_.conservativeResize(cols);
    UntaggedFilterData::conservativeResize(rows, cols);
  }

  // give an interface to enable constructing and appending a single
  // observation - adds on the obs_id and calls the base class
  // function
  void insert_observation(
      const ObservationIdentifier &obs_id, const double &y,
      const VectorMaxStateDimd_t &H_row,
      const optional<const ObservationModelFunctionRow &> &h_func,
      const double &R);

  PRC insert_observation(const ObservationIdentifier &obs_id,
                         const TaggedFilterData &from);

  // give an interface to remove an observation at a given index -
  // removes the obs_id and calls the base class function
  void remove_ob(const ObservationIdentifier &obs_id);

  const FilterObservationIdVector &get_ids() const { return obs_ids_; };

  const pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
      &get_state_labels() const {
    return state_labels_;
  }

  SingleObsUntaggedFilterData get_single_obs_by_id(
      const ObservationIdentifier &obs_id) const;

 private:
  // probably don't need const access to the members, all functions
  // requiring access should probably be in this class
  FilterObservationIdVector obs_ids_;
  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
      state_labels_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_TAGGED_FILTER_DATA_H
