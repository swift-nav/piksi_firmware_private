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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_HANDLER_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_HANDLER_H

#include <pvt_engine/optional.h>
#include <array>
#include <cassert>

#include <swiftnav/gnss_time.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/signal.h>

#include <starling/observation.h>

#include <pvt_common/containers/map.h>
#include <pvt_common/containers/set.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/difference.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/finity.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_utils.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sid_set.h>

namespace pvt_engine {

template <s32 max_obs_num>
class ObservationHandler;
class ObservationIterator;
class ConstObservationIterator;

template <s32 max_obs_num>
class ObservationHandler {  // NOLINT -- clang-tidy shits the bed here
  friend class ObservationIterator;
  friend class ConstObservationIterator;

 public:
  ObservationHandler();

  ObservationHandler(const UndifferencedObservations &undifferenced_obs,
                     const bool is_time_matched);

  ObservationHandler(const ObservationHandler &other,
                     const bool is_time_matched);

  template <s32 other_max_obs_num>
  explicit ObservationHandler(
      const ObservationHandler<other_max_obs_num> &other);

  ObservationHandler(const ObservationHandler &rover_obs_handler,
                     const ObservationHandler &reference_obs_handler,
                     const EphemerisHandler &eph_handler);

  void initialize();

  void set_new_observations(const UndifferencedObservations &undifferenced_obs,
                            const bool is_time_matched = true);

  void update_sat_PVAs_from_reference(const ObservationHandler &sdiffs);

  bool contains_orion_corrections() const;

  optional<gps_time_t> get_epoch_time() const;

  optional<double> get_propagation_time() const;

  bool is_time_matched() const;

  PRC add_baseline_magnitude_observation(const double &obs_value);

  using Map = pvt_common::containers::Map<ObservationIdentifier, Observation,
                                          max_obs_num>;
  using ObservationWithIdentifier =
      pvt_common::containers::MapElement<ObservationIdentifier, Observation>;
  using Iterator =
      pvt_common::containers::MapElement<const ObservationIdentifier,
                                         Observation> *;
  using ConstIterator =
      const pvt_common::containers::MapElement<ObservationIdentifier,
                                               Observation> *;
  Iterator begin();
  Iterator end();
  ConstIterator begin() const;
  ConstIterator end() const;
  ConstIterator cbegin() const;
  ConstIterator cend() const;

  s32 size() const;
  s32 unique_sats_size() const;
  s32 unique_constellations_size() const;

  pvt_common::containers::Set<ObservationIdentifier, max_obs_num> get_signals()
      const;

  void remove_observation(const ObservationIdentifier &obs_id);
  void remove_observations(
      const pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
          &obs_ids);
  void add_observations(const Map &observations, const bool &is_time_matched);
  optional<const Observation &> lookup_observation(
      const pvt_engine::ObservationIdentifier &obs_id) const;

  ObservationHandler filtered(bool (*select_f)(
      const ObservationHandler::ObservationWithIdentifier &elem)) const &;
  ObservationHandler filtered(bool (*select_f)(
      const ObservationHandler::ObservationWithIdentifier &elem)) &&;

  template <typename C>
  ObservationHandler filtered(
      bool (*select_f)(const ObservationWithIdentifier &elem, C helper),
      C helper) const &;
  template <typename C>
  ObservationHandler filtered(
      bool (*select_f)(const ObservationWithIdentifier &elem, C helper),
      C helper) &&;

  template <typename C>
  pvt_common::containers::FilteredMapView<ObservationIdentifier, Observation, C,
                                          max_obs_num>
  filtered_view(bool (*select_f)(const pvt_common::containers::MapElement<
                                     ObservationIdentifier, Observation> &elem,
                                 const C &helper),
                const C &helper) const {
    return observations_.template filtered_view<C>(select_f, helper);
  }

  template <typename C>
  C traverse(C (*traverse_f)(const ObservationWithIdentifier &elem,
                             C accumulator),
             C initial_accumulator) const;

 private:
  optional<Observation> single_obs_of_sdiff(const gps_time_t &epoch_time,
                                            const sdiff_t &sdiff,
                                            OBSERVATION_TYPE obs_type,
                                            double propagation_time) const;

  bool single_obs_contains_orion_corrections(const Observation &obs) const;

  optional<Observation> single_obs_of_nav_meas(
      const gps_time_t &epoch_time, const navigation_measurement_t &nav_meas,
      OBSERVATION_TYPE obs_type) const;
  void reinitialize_observations();

  Map observations_;
  bool is_time_matched_;
  bool contains_orion_corrections_;
};

template <s32 max_obs_num>
pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
sidset_to_obs_id_set(const ambiguities::SidSet &sids);

template <s32 max_obs_num>
SatIdSet obs_ids_to_sats(
    const pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
        &obs_ids);

template <s32 max_obs_num>
s32 num_unique_sats(const pvt_common::containers::Set<ObservationIdentifier,
                                                      max_obs_num> &obs_ids);

template <s32 max_obs_num>
s32 num_unique_constellations(
    const pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
        &obs_ids);

template <s32 max_obs_num>
pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
sidset_to_obs_id_set(const ambiguities::SidSet &sids) {
  pvt_common::containers::Set<ObservationIdentifier, max_obs_num> ret;
  for (const auto &sid : sids) {
    ret.add(ObservationIdentifier(sid, CARRIER_PHASE));
  }
  return ret;
}

template <s32 max_obs_num>
SatIdSet obs_ids_to_sats(
    const pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
        &obs_ids) {
  SatIdSet sats;
  for (const auto &obs : obs_ids) {
    sats.add(SatIdentifier(obs.sid));
  }
  return sats;
}

template <s32 max_obs_num>
s32 num_unique_sats(const pvt_common::containers::Set<ObservationIdentifier,
                                                      max_obs_num> &obs_ids) {
  return obs_ids_to_sats(obs_ids).size();
}

template <s32 max_obs_num>
s32 num_unique_constellations(
    const pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
        &obs_ids) {
  obs_filters::ConstellationSet const_set;
  for (const auto &obs : obs_ids) {
    const_set.add(sid_to_constellation(obs.sid));
  }
  return const_set.size();
}

template <s32 max_obs_num>
template <typename C>
ObservationHandler<max_obs_num> ObservationHandler<max_obs_num>::filtered(
    bool (*select_f)(
        const ObservationHandler<max_obs_num>::ObservationWithIdentifier &elem,
        C helper),
    C helper) const & {
  ObservationHandler<max_obs_num> other(*this);
  other.observations_.template filter<C>(select_f, helper);
  return other;
}

template <s32 max_obs_num>
template <typename C>
ObservationHandler<max_obs_num> ObservationHandler<max_obs_num>::filtered(
    bool (*select_f)(
        const ObservationHandler<max_obs_num>::ObservationWithIdentifier &elem,
        C helper),
    C helper) && {
  observations_.template filter<C>(select_f, helper);
  return std::move(*this);
}

template <s32 max_obs_num>
template <typename C>
C ObservationHandler<max_obs_num>::traverse(
    C (*traverse_f)(
        const ObservationHandler<max_obs_num>::ObservationWithIdentifier &elem,
        C accumulator),
    C initial_accumulator) const {
  return observations_.traverse(traverse_f, initial_accumulator);
}

using FilterObservationHandler = ObservationHandler<cMaxFilterObservations>;
using InputObservationHandler = ObservationHandler<cMaxInputObservations>;

// We don't want this to be templated because then it could take input with too
// many signals for the size of the output. It can only accept a
// FilterObservationIdSet.
ambiguities::SidSet sidset_of_obs_id_set(const FilterObservationIdSet &indices);

namespace obs_filters {

bool match_type(const FilterObservationHandler::ObservationWithIdentifier &elem,
                OBSERVATION_TYPE type);

bool exclude_type(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    OBSERVATION_TYPE type);

bool match_type_const_ref(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const OBSERVATION_TYPE &type);

bool exclude_type_const_ref(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const OBSERVATION_TYPE &type);

bool match_types(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const ObservationTypeSet &types);
bool exclude_types(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const ObservationTypeSet &types);

bool match_type(const ObservationIdentifier &obs_id, OBSERVATION_TYPE type);

bool exclude_type(const ObservationIdentifier &obs_id, OBSERVATION_TYPE type);

bool match_types(const ObservationIdentifier &obs_id,
                 const ObservationTypeSet &types);

bool exclude_types(const ObservationIdentifier &obs_id,
                   const ObservationTypeSet &types);

bool select_lol(const FilterObservationHandler::ObservationWithIdentifier &elem,
                const optional<gps_time_t> &last_processed_epoch);

bool ignore_lol(const FilterObservationHandler::ObservationWithIdentifier &elem,
                const optional<gps_time_t> &last_processed_epoch);
bool match_frequency(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    FREQUENCY frequency);

bool exclude_frequency(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    FREQUENCY frequency);

bool match_frequencies(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const FrequencySet &frequencies);

bool exclude_frequencies(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const FrequencySet &frequencies);

bool match_frequency(const ObservationIdentifier &obs_id, FREQUENCY frequency);

bool exclude_frequency(const ObservationIdentifier &obs_id,
                       FREQUENCY frequency);

bool match_frequencies(const ObservationIdentifier &obs_id,
                       const FrequencySet &frequencies);

bool exclude_frequencies(const ObservationIdentifier &obs_id,
                         const FrequencySet &frequencies);

bool match_constellation(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    constellation_t constellation);

bool exclude_constellation(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    constellation_t constellation);

bool match_constellations(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const ConstellationSet &constellations);

bool exclude_constellations(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const ConstellationSet &constellations);

bool match_constellation(const ObservationIdentifier &obs_id,
                         constellation_t constellation);

bool exclude_constellation(const ObservationIdentifier &obs_id,
                           constellation_t constellation);

bool match_constellations(const ObservationIdentifier &obs_id,
                          const ConstellationSet &constellations);

bool exclude_constellations(const ObservationIdentifier &obs_id,
                            const ConstellationSet &constellations);

bool match_code(const FilterObservationHandler::ObservationWithIdentifier &elem,
                code_t code);

bool exclude_code(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    code_t code);

bool match_codes(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const ObservationCodeSet &codes);
bool exclude_codes(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const ObservationCodeSet &codes);

bool match_code(const ObservationIdentifier &obs_id, code_t code);

bool exclude_code(const ObservationIdentifier &obs_id, code_t code);

bool match_codes(const ObservationIdentifier &obs_id,
                 const ObservationCodeSet &codes);

bool exclude_codes(const ObservationIdentifier &obs_id,
                   const ObservationCodeSet &codes);

bool match_SatId(
    const FilterObservationHandler::ObservationWithIdentifier &elem,
    const SatIdSet &sat_num);

}  // namespace obs_filters

template <s32 max_obs_num>
ObservationHandler<max_obs_num>::ObservationHandler()
    : observations_(),
      is_time_matched_(true),
      contains_orion_corrections_(false) {}

template <s32 max_obs_num>
ObservationHandler<max_obs_num>::ObservationHandler(
    const UndifferencedObservations &undifferenced_obs,
    const bool is_time_matched)
    : observations_(),
      is_time_matched_(is_time_matched),
      contains_orion_corrections_(false) {
  set_new_observations(undifferenced_obs, is_time_matched);
}

template <s32 max_obs_num>
ObservationHandler<max_obs_num>::ObservationHandler(
    const ObservationHandler<max_obs_num> &other, const bool is_time_matched)
    : observations_(other.observations_),
      is_time_matched_(is_time_matched),
      contains_orion_corrections_(other.contains_orion_corrections()) {}

template <s32 max_obs_num>
template <s32 other_max_obs_num>
ObservationHandler<max_obs_num>::ObservationHandler(
    const ObservationHandler<other_max_obs_num> &other)
    : observations_(other.begin(), other.end()),
      is_time_matched_(other.is_time_matched()),
      contains_orion_corrections_(other.contains_orion_corrections()) {
  // Note: if other contains too many observations, an assert will be triggered
  // within the assignment of observations_.
}

template <s32 max_obs_num>
ObservationHandler<max_obs_num>::ObservationHandler(
    const ObservationHandler<max_obs_num> &rover_obs_handler,
    const ObservationHandler<max_obs_num> &reference_obs_handler,
    const EphemerisHandler &eph_handler)
    : observations_(),
      is_time_matched_(rover_obs_handler.is_time_matched_ &&
                       reference_obs_handler.is_time_matched_),
      contains_orion_corrections_(
          reference_obs_handler.contains_orion_corrections()) {
  observations_.template intersect_on<const EphemerisHandler &>(
      difference::single, rover_obs_handler.observations_,
      reference_obs_handler.observations_, eph_handler);
}

template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::initialize() {
  reinitialize_observations();
}

/**
 * reinitialise the internal state of the observation handler which will need
 * to happen at each epoch before observations are loaded in.
 */
template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::reinitialize_observations() {
  observations_ = pvt_common::containers::Map<ObservationIdentifier,
                                              Observation, max_obs_num>();
}

optional<double> accept_observation(const navigation_measurement_t &nav_meas,
                                    const OBSERVATION_TYPE type);

template <s32 max_obs_num>
bool ObservationHandler<max_obs_num>::single_obs_contains_orion_corrections(
    const pvt_engine::Observation &obs) const {
  return (obs.iono_std.has_value() && obs.tropo_std.has_value() &&
          obs.range_std.has_value());
}

template <s32 max_obs_num>
optional<Observation> ObservationHandler<max_obs_num>::single_obs_of_nav_meas(
    const gps_time_t &epoch_time, const navigation_measurement_t &nav_meas,
    OBSERVATION_TYPE obs_type) const {
  const auto valid_obs_value = accept_observation(nav_meas, obs_type);
  if (!valid_obs_value) {
    return {};
  }

  Observation output;
  output.obs_id.obs_type = obs_type;
  output.obs_id.sid = nav_meas.sid;
  output.epoch_time = epoch_time;
  output.time_of_transmission = nav_meas.tot;
  output.measurement = *valid_obs_value;
  if (obs_utils::is_flag_set(nav_meas.flags, NAV_MEAS_FLAG_CN0_VALID)) {
    output.cn0 = nav_meas.cn0;
  }
  output.lock_time = nav_meas.lock_time;
  output.sat_pva =
      SatPVA(nav_meas.sat_pos, nav_meas.sat_vel, nav_meas.sat_acc,
             nav_meas.sat_clock_err * GPS_C,
             nav_meas.sat_clock_err_rate * GPS_C, nav_meas.IODE, nav_meas.IODC);
  output.last_loss_of_lock = {};

  // Increment the epoch counter
  const auto previous_obs = observations_[output.obs_id];
  if (previous_obs) {
    output.epoch_count = previous_obs->epoch_count + 1;
  } else {
    output.epoch_count = 1;
  }

  return output;
}

template <s32 max_obs_num>
PRC ObservationHandler<max_obs_num>::add_baseline_magnitude_observation(
    const double &obs_value) {
  // Note: We do not need to worry about changes to other ObservationHandler
  // member variables here, since baseline magnitude observations are synthetic
  // pseudo-observations which do not affect this object's state outside of
  // the map of observations
  ObservationIdentifier obs_id;
  obs_id.obs_type = BASELINE_MAGNITUDE;
  obs_id.sid.code = CODE_INVALID;
  obs_id.sid.sat = 0;
  Observation obs;
  obs.measurement = obs_value;
  obs.obs_id = obs_id;

  if (observations_.insert_missing(obs_id, obs).has_value()) {
    return RC_S_OK;
  }

  assert(obs.obs_id.obs_type == obs_id.obs_type);
  assert(obs_id.obs_type != INVALID_OBSERVATION_TYPE &&
         obs_id.obs_type != MAX_OBSERVATION_TYPE);

  return RC_E_NOTOK;
}

template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::remove_observation(
    const ObservationIdentifier &obs_id) {
  observations_.drop(obs_id);
}

template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::remove_observations(
    const pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
        &obs_ids) {
  observations_.drop(obs_ids);
}

template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::add_observations(
    const ObservationHandler::Map &observations, const bool &is_time_matched) {
  // Verify that the incoming observations' properties match those of the
  // existing observations
  bool new_obs_contain_orion_corrections = (observations.size() > 0);
  u8 num_obs_with_orion_corrections = 0;
  for (const auto &obs : observations) {
    if (!single_obs_contains_orion_corrections(obs.value)) {
      new_obs_contain_orion_corrections = false;
    } else {
      num_obs_with_orion_corrections++;
    }
  }
  assert(num_obs_with_orion_corrections == 0 ||
         num_obs_with_orion_corrections == observations.size());
  assert(contains_orion_corrections_ == new_obs_contain_orion_corrections);
  assert(is_time_matched_ == is_time_matched);

  observations_.extend(observations);
}

template <s32 max_obs_num>
optional<const Observation &>
ObservationHandler<max_obs_num>::lookup_observation(
    const pvt_engine::ObservationIdentifier &obs_id) const {
  return observations_.lookup(obs_id);
}

template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::set_new_observations(
    const UndifferencedObservations &undifferenced_obs,
    const bool is_time_matched) {
  is_time_matched_ = is_time_matched;

  contains_orion_corrections_ = (undifferenced_obs.num_obs_ > 0);
  u8 num_obs_with_orion_corrections = 0;

  // Store the new observations.
  observations_ = Map();

  for (u8 i = 0; i < undifferenced_obs.num_obs_; i++) {
    for (s32 j = 0; j < MAX_OBSERVATION_TYPE; ++j) {
      OBSERVATION_TYPE obs_type = static_cast<OBSERVATION_TYPE>(j);
      // We should never be trying to put duplicate observations into
      // the observation handler in the same call to this function.
      const ObservationIdentifier obs_id(undifferenced_obs.nav_meas_[i].sid,
                                         obs_type);
      if (observations_.contains(obs_id)) {
        log_error_sid(undifferenced_obs.nav_meas_[i].sid,
                      "Duplicate observation");
        continue;
      }

      auto single_obs =
          single_obs_of_nav_meas(undifferenced_obs.obs_time_,
                                 undifferenced_obs.nav_meas_[i], obs_type);
      if (single_obs) {
        assert(sid_valid(obs_id.sid));
        assert(single_obs->obs_id == obs_id);
        if (undifferenced_obs.obs_std_[obs_id.sid]) {
          single_obs->filling_observations_variance(
              *undifferenced_obs.obs_std_[obs_id.sid]);

          assert(single_obs_contains_orion_corrections(*single_obs));
          num_obs_with_orion_corrections++;
        } else {
          contains_orion_corrections_ = false;
        }
        observations_.insert(obs_id, *single_obs);
      }
    }
  }

  assert(num_obs_with_orion_corrections == 0 ||
         num_obs_with_orion_corrections == observations_.size());
}

template <s32 max_obs_num>
void ObservationHandler<max_obs_num>::update_sat_PVAs_from_reference(
    const ObservationHandler &sdiffs) {
  for (const auto &sdiff : sdiffs) {
    optional<Observation &> undiff = observations_[sdiff.key];
    assert(undiff.has_value());
    if (fabs(gpsdifftime(&undiff->time_of_transmission,
                         &sdiff.value.ref_time_of_transmission)) <
        FLOAT_EQUALITY_EPS) {
      if (undiff->sat_pva.get_iodc() != sdiff.value.ref_sat_pva.get_iodc() ||
          undiff->sat_pva.get_iode() != sdiff.value.ref_sat_pva.get_iode()) {
        undiff->set_rover_sat_from_ref_sat(sdiff.value);
      }
    }
  }
}

template <s32 max_obs_num>
ObservationHandler<max_obs_num> ObservationHandler<max_obs_num>::filtered(
    bool (*select_f)(const ObservationHandler::ObservationWithIdentifier &elem))
    const & {
  ObservationHandler other(*this);
  other.observations_.filter(select_f);
  return other;
}

template <s32 max_obs_num>
ObservationHandler<max_obs_num> ObservationHandler<max_obs_num>::filtered(bool (
    *select_f)(const ObservationHandler::ObservationWithIdentifier &elem)) && {
  observations_.filter(select_f);
  return std::move(*this);
}

template <s32 max_obs_num>
bool ObservationHandler<max_obs_num>::contains_orion_corrections() const {
  return contains_orion_corrections_;
}

template <s32 max_obs_num>
optional<gps_time_t> ObservationHandler<max_obs_num>::get_epoch_time() const {
  optional<gps_time_t> epoch_time;
  for (const auto &observation :
       observations_.template filtered_view<obs_filters::ObservationTypeSet>(
           obs_filters::match_types,
           {PSEUDORANGE, CARRIER_PHASE, MEASURED_DOPPLER, COMPUTED_DOPPLER})) {
    if (!epoch_time) {
      epoch_time = observation.value.epoch_time;
    } else {
      // Differences in transmission time can cause the epoch to differ by more
      // than FLOAT_QUALITY_EPS
      assert(fabs(gpsdifftime(&*epoch_time, &observation.value.epoch_time)) <
             1e-10);
    }
  }
  return epoch_time;
}

template <s32 max_obs_num>
optional<double> ObservationHandler<max_obs_num>::get_propagation_time() const {
  optional<double> propagation_time;
  for (const auto &observation :
       observations_.template filtered_view<obs_filters::ObservationTypeSet>(
           obs_filters::match_types,
           {PSEUDORANGE, CARRIER_PHASE, MEASURED_DOPPLER, COMPUTED_DOPPLER})) {
    if (!propagation_time) {
      propagation_time = observation.value.propagation_time;
    } else {
      assert(double_approx_eq(*propagation_time,
                              observation.value.propagation_time));
    }
  }
  return propagation_time;
}

template <s32 max_obs_num>
bool ObservationHandler<max_obs_num>::is_time_matched() const {
  return is_time_matched_;
}

/**
 * Returns a iterator pointing at the start of the set of observations.
 * @return the iterator
 */
template <s32 max_obs_num>
typename ObservationHandler<max_obs_num>::Iterator
ObservationHandler<max_obs_num>::begin() {
  return observations_.begin();
}

/**
 * Returns a iterator pointing at one position pas the end of the set of
 * observations.
 * @return the iterator
 */
template <s32 max_obs_num>
typename ObservationHandler<max_obs_num>::Iterator
ObservationHandler<max_obs_num>::end() {
  return observations_.end();
}

/**
 * Returns the count of the number of observations.
 * @return the count
 */
template <s32 max_obs_num>
s32 ObservationHandler<max_obs_num>::size() const {
  return observations_.size();
}

template <s32 max_obs_num>
pvt_common::containers::Set<ObservationIdentifier, max_obs_num>
ObservationHandler<max_obs_num>::get_signals() const {
  return observations_.keys();
}

inline ambiguities::SidSet sidset_of_obs_id_set(
    const FilterObservationIdSet &indices) {
  const auto sid_of_obs_id = [](const ObservationIdentifier &obs_id) {
    return obs_id.sid;
  };
  return *indices.template transformed<gnss_signal_t>(sid_of_obs_id)
              .template shrunk<cMaxAmbiguities>();
}

/**
 * Returns the the number of unique satellites in the observations.
 * @return the number of unique satellites
 */
template <s32 max_obs_num>
s32 ObservationHandler<max_obs_num>::unique_sats_size() const {
  return num_unique_sats(
      filtered<const obs_filters::ObservationTypeSet &>(
          obs_filters::match_types, {PSEUDORANGE, CARRIER_PHASE})
          .get_signals());
}

template <s32 max_obs_num>
s32 ObservationHandler<max_obs_num>::unique_constellations_size() const {
  return num_unique_constellations(
      filtered<const obs_filters::ObservationTypeSet &>(
          obs_filters::match_types, {PSEUDORANGE, CARRIER_PHASE})
          .get_signals());
}

/**
 * Returns a const iterator pointing at the start of the set of
 * observations.
 * @return the iterator
 */
template <s32 max_obs_num>
typename ObservationHandler<max_obs_num>::ConstIterator
ObservationHandler<max_obs_num>::cbegin() const {
  return observations_.cbegin();
}

/**
 * Returns a const iterator pointing at one position past the end of the set of
 * observations.
 * @return the iterator
 */
template <s32 max_obs_num>
typename ObservationHandler<max_obs_num>::ConstIterator
ObservationHandler<max_obs_num>::cend() const {
  return observations_.cend();
}

/**
 * Returns a const iterator pointing at the start of the set of
 * observations.
 * @return the iterator
 */
template <s32 max_obs_num>
typename ObservationHandler<max_obs_num>::ConstIterator
ObservationHandler<max_obs_num>::begin() const {
  return observations_.begin();
}

/**
 * Returns a const iterator pointing at one position past the end of the set of
 * observations.
 * @return the iterator
 */
template <s32 max_obs_num>
typename ObservationHandler<max_obs_num>::ConstIterator
ObservationHandler<max_obs_num>::end() const {
  return observations_.end();
}

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_HANDLER_H
