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

#ifndef LIBSWIFTNAV_PVT_ENGINE_UNTAGGED_FILTER_DATA_H
#define LIBSWIFTNAV_PVT_ENGINE_UNTAGGED_FILTER_DATA_H

#include <pvt_engine/optional.h>

#include <pvt_engine/eigen_types.h>

#include <pvt_engine/filter_data_interface.h>

namespace pvt_engine {

class UntaggedFilterData : public FilterDataInterface {
 public:
  UntaggedFilterData();
  // allow construction if we already have the data
  UntaggedFilterData(
      const VectorMaxObsd_t &y, const MatrixMaxObsByStatesd_t &H,
      const optional<const pvt_common::containers::StaticVector<
          ObservationModelFunctionRow, cMaxFilterObservations> &> &h_func,
      const VectorMaxObsd_t &R);
  UntaggedFilterData &operator=(const UntaggedFilterData &rhs);
  UntaggedFilterData &operator=(UntaggedFilterData &&rhs) = default;  // NOLINT
  UntaggedFilterData(const UntaggedFilterData &other) = default;
  UntaggedFilterData(UntaggedFilterData &&other) = default;  // NOLINT
  ~UntaggedFilterData() override;

  const optional<const ObservationModelFunctionRow &> get_h_func(
      s32 idx) const override;
  void clear();
  void resize(const s32 &rows, const s32 &cols);
  void conservativeResize(const s32 &rows, const s32 &cols);

  const VectorMaxObsd_t &get_y() const override { return y_; }
  const MatrixMaxObsByStatesd_t &get_H() const override { return H_; }
  const VectorMaxObsd_t &get_R() const override { return R_; }

  // give const access to the members
  SingleObsUntaggedFilterData get_single_obs(const s32 &index) const override;
  s32 get_num_obs() const override { return curr_size; }

  // give an interface to enable constructing and appending a single
  // observation
  void insert_observation(
      const double &y, const VectorMaxStateDimd_t &H_row,
      const optional<const ObservationModelFunctionRow &> &h_func,
      const double &R);

  //   protected:
  // give an interface to remove an observation at a given index -
  // protected so the remove in the deriving class must be called
  // with the obs_id, rather than this
  void remove_ob(const s32 &idx);

 protected:
  s32 curr_size;

 private:
  VectorMaxObsd_t y_;
  MatrixMaxObsByStatesd_t H_;
  // This was turned into a pair for CPU load reasons - we can avoid calling the
  // ObservationModelFunctionRow constructor by simply switching the bool on and
  // off
  std::pair<bool, pvt_common::containers::StaticVector<
                      ObservationModelFunctionRow, cMaxFilterObservations>>
      h_func_;
  VectorMaxObsd_t R_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_UNTAGGED_FILTER_DATA_H
