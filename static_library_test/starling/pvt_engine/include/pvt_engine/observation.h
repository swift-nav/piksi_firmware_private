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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_H

#include <inttypes.h>

#include <array>

#include <swiftnav/gnss_time.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/signal.h>
#include <swiftnav/single_epoch_solver.h>

#include <pvt_common/containers/optional_tuple.h>
#include <pvt_common/optional.h>
#include <pvt_engine/azimuth_elevation.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/satellite_pva.h>

namespace pvt_engine {

static constexpr double IONO_STD_MULTIPLIER = 5e-3;
static constexpr double TROPO_STD_MULTIPLIER = 5e-3;
static constexpr double RANGE_STD_MULTIPLIER = 5e-3;

struct ObservationIdentifier {
  ObservationIdentifier()
      : sid{0, CODE_INVALID}, obs_type(INVALID_OBSERVATION_TYPE) {}
  ObservationIdentifier(gnss_signal_t s, OBSERVATION_TYPE ot)
      : sid(s), obs_type(ot) {}

  gnss_signal_t sid;
  OBSERVATION_TYPE obs_type;

  bool operator==(const ObservationIdentifier &other) const {
    return (obs_type == other.obs_type && sid == other.sid);
  }

  bool operator!=(const ObservationIdentifier &other) const {
    return !(*this == other);
  }

  bool operator<(const ObservationIdentifier &other) const {
    if (other.sid == sid) {
      return obs_type < other.obs_type;
    }

    return sid < other.sid;
  }
};

using ObsId2DoubleMap =
    pvt_common::containers::Map<ObservationIdentifier, double,
                                cMaxInputObservations>;

struct Observation {
  ObservationIdentifier obs_id;
  // Time of reception after approximate correction for receiver clock by the ME
  // (obs are also adjusted so that the apparent receiver clock error is the
  // same in this as in the obs)
  gps_time_t epoch_time;
  // Time of transmission as determined by subtracting the time of flight
  // (pseudorange/speed of light) from the time of reception. Still contains
  // residual satellite clock error
  gps_time_t time_of_transmission;
  gps_time_t ref_time_of_transmission;
  s32 epoch_count;
  applied_model_bitfield applied_apriori_corrections;
  double measurement;

  double lock_time;
  double propagation_time;
  double effective_lock_duration;

  bool half_cycle_slip_ambiguity_resolved;

 private:
  enum OptionalIndex {
    SatPva,
    RefSatPva,
    Cn0,
    LastLossOfLock,
    Flags,
    Dt,
    YawAngleDegrees,
    IonoStd,
    TropoStd,
    RangeStd,
    SatAzEl
  };

  pvt_common::containers::OptionalTuple<SatPVA, SatPVA, double, gps_time_t, u8,
                                        double, double, double, double, double,
                                        AzimuthElevation>
      optional_tuple_;

 public:
  Observation()
      : obs_id(),
        epoch_time(),
        time_of_transmission(),
        ref_time_of_transmission(),
        epoch_count(1),
        applied_apriori_corrections(0),
        measurement(0.0),
        lock_time(0.0),
        propagation_time(0),
        effective_lock_duration(0.0),
        half_cycle_slip_ambiguity_resolved(false),
        optional_tuple_() {}

  auto sat_pva() { return optional_tuple_.get<SatPva>(); }

  auto sat_pva() const { return optional_tuple_.get<SatPva>(); }

  auto ref_sat_pva() { return optional_tuple_.get<RefSatPva>(); }

  auto ref_sat_pva() const { return optional_tuple_.get<RefSatPva>(); }

  auto cn0() { return optional_tuple_.get<Cn0>(); }

  auto cn0() const { return optional_tuple_.get<Cn0>(); }

  auto last_loss_of_lock() { return optional_tuple_.get<LastLossOfLock>(); }

  auto last_loss_of_lock() const {
    return optional_tuple_.get<LastLossOfLock>();
  }

  auto dt() { return optional_tuple_.get<Dt>(); }

  auto dt() const { return optional_tuple_.get<Dt>(); }

  auto yaw_angle_degrees() { return optional_tuple_.get<YawAngleDegrees>(); }

  auto yaw_angle_degrees() const {
    return optional_tuple_.get<YawAngleDegrees>();
  }

  auto iono_std() { return optional_tuple_.get<IonoStd>(); }

  auto iono_std() const { return optional_tuple_.get<IonoStd>(); }

  auto tropo_std() { return optional_tuple_.get<TropoStd>(); }

  auto tropo_std() const { return optional_tuple_.get<TropoStd>(); }

  auto range_std() { return optional_tuple_.get<RangeStd>(); }

  auto range_std() const { return optional_tuple_.get<RangeStd>(); }

  auto flags() { return optional_tuple_.get<Flags>(); }

  auto flags() const { return optional_tuple_.get<Flags>(); }

  auto sat_az_el() { return optional_tuple_.get<SatAzEl>(); }

  auto sat_az_el() const { return optional_tuple_.get<SatAzEl>(); }

  void set_rover_sat_from_ref_sat(const Observation &other) {
    sat_pva() = other.ref_sat_pva().std_optional();
  }

  void set_rover_sat(const Observation &other) { sat_pva() = other.sat_pva(); }

  void set_ref_sat_from_rov_sat(const Observation &other) {
    ref_sat_pva() = other.sat_pva().std_optional();
  }

  void filling_observations_variance(const measurement_std_t &obs_std) {
    iono_std() = obs_std.iono_std;
    tropo_std() = obs_std.tropo_std;
    range_std() = obs_std.range_std;
    flags() = obs_std.flags;
  }
};

struct lock_timer {
  lock_timer() : obs_id(), lock_time(0.0) {}

  ObservationIdentifier obs_id;
  double lock_time;
};

struct LabeledObservation {
  Eigen::Vector3d sat_unit_vector;
  double measurement;
  double sat_elevation;
  double measurement_cn0;
  bool half_cycle_slip_ambiguity;
  s32 epoch_count;

  LabeledObservation()
      : sat_unit_vector{0, 0, 0},
        measurement(),
        sat_elevation(-100),
        measurement_cn0(0),
        half_cycle_slip_ambiguity(false),
        epoch_count(0) {}
  LabeledObservation(Eigen::Vector3d unit_vector, double meas, double elevation,
                     double cn0, bool half_cycle_slip_flag)
      : sat_unit_vector(std::move(unit_vector)),
        measurement(meas),
        sat_elevation(elevation),
        measurement_cn0(cn0),
        half_cycle_slip_ambiguity(half_cycle_slip_flag),
        epoch_count(0) {}
};

using FilterObservationIdSet =
    pvt_common::containers::Set<ObservationIdentifier, cMaxFilterObservations>;

using InputObservationIdSet =
    pvt_common::containers::Set<ObservationIdentifier, cMaxInputObservations>;

using FilterObservationIdVector =
    pvt_common::containers::StaticVector<ObservationIdentifier,
                                         cMaxFilterObservations>;

namespace obs_filters {

using ObservationTypeSet =
    pvt_common::containers::Set<OBSERVATION_TYPE,
                                static_cast<s32>(MAX_OBSERVATION_TYPE)>;

using FrequencySet =
    pvt_common::containers::Set<FREQUENCY, static_cast<s32>(MAX_FREQUENCY)>;

using ConstellationSet =
    pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>;

using CodeSet =
    pvt_common::containers::Set<code_t, static_cast<s32>(CODE_COUNT)>;

}  // namespace obs_filters

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_H
