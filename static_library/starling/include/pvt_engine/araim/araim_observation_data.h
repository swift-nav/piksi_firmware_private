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

#ifndef STARLING_PVT_ENGINE_ARAIM_OBSERVATION_DATA_H
#define STARLING_PVT_ENGINE_ARAIM_OBSERVATION_DATA_H

#include <pvt_common/containers/set.h>
#include <pvt_engine/ambiguity_set.h>
#include <pvt_engine/araim/araim_definitions.h>
#include <pvt_engine/integrity_support_message.h>
#include <pvt_engine/tagged_filter_data.h>

namespace pvt_engine {
namespace araim {

class AraimObservationData {
 public:
  AraimObservationData(
      const TaggedFilterData &raw_observation_model,
      const Eigen::Vector3d &linearization_point,
      const IntegritySupportMessage &ism,
      const ambiguities::TransformedIntegerAmbiguities &ambiguities);

  VectorMaxObsd_t get_y() const { return y_; }
  MatrixMaxObsByStatesd_t get_G() const { return G_; }
  MatrixMaxObsd_t get_C_int() const { return C_int_; }
  MatrixMaxObsd_t get_C_acc() const { return C_acc_; }
  VectorMaxObsd_t get_b_nom() const { return b_nom_; }
  Eigen::Vector3d get_linearization_point() const {
    return linearization_point_;
  }

  ambiguities::SidVector get_obs_sids() const;

  ObsIndexSubset sat_id_to_index(const SatIdentifier &sat_id) const;
  ObsIndexSubset constellation_to_index(
      const constellation_t &constellation) const;

  pvt_common::containers::StaticVector<s32, NUM_DIMENSIONS>
  position_state_indices() const;

 private:
  void transform_and_disambiguate();
  template <typename P>
  ObsIndexSubset traverse_ids_for_indices(P obs_id_matcher) const;

  VectorMaxObsd_t y_;
  MatrixMaxObsByStatesd_t G_;
  MatrixMaxObsd_t R_;
  MatrixMaxObsd_t C_int_;
  MatrixMaxObsd_t C_acc_;
  VectorMaxObsd_t b_nom_;
  Eigen::Vector3d linearization_point_;
  ambiguities::TransformedIntegerAmbiguities ambiguities_;

  FilterObservationIdVector uncombined_obs_ids_;
  pvt_common::containers::StaticVector<StateAndModel, cMaxStateDim>
      state_labels_;
};

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_PVT_ENGINE_ARAIM_OBSERVATION_DATA_H
