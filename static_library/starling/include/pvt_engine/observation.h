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
#include <swiftnav/signal.h>

#include <pvt_common/containers/set.h>
#include <pvt_engine/azimuth_elevation.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/frequency_manager.h>
#include <pvt_engine/optional.h>
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
  applied_model_bitfield applied_apriori_corrections;
  double measurement;
  // If we have undifferenced data, the reference sat storage is unused
  SatPVA sat_pva;
  SatPVA ref_sat_pva;

  optional<double> cn0;
  double lock_time;
  double propagation_time;
  s32 epoch_count;
  double effective_lock_duration;
  optional<gps_time_t> last_loss_of_lock;
  optional<double> dt;
  optional<double> yaw_angle_degrees;
  // Variances of observations
  optional<double> iono_std;
  optional<double> tropo_std;
  optional<double> range_std;
  optional<u8> flags;
  optional<AzimuthElevation> sat_az_el;

  Observation()
      : obs_id(),
        epoch_time(),
        time_of_transmission(),
        ref_time_of_transmission(),
        applied_apriori_corrections(0),
        measurement(0.0),
        sat_pva(),
        ref_sat_pva(),
        cn0(),
        lock_time(0.0),
        propagation_time(0),
        epoch_count(1),
        effective_lock_duration(0.0),
        last_loss_of_lock(),
        dt(),
        yaw_angle_degrees(),
        iono_std(),
        tropo_std(),
        range_std(),
        flags(),
        sat_az_el() {}

  void set_rover_sat_from_ref_sat(const Observation &other) {
    sat_pva = other.ref_sat_pva;
  }

  void set_rover_sat(const Observation &other) { sat_pva = other.sat_pva; }

  void set_ref_sat_from_rov_sat(const Observation &other) {
    ref_sat_pva = other.sat_pva;
  }

  void filling_observations_variance(const measurement_std_t &obs_std) {
    iono_std = obs_std.iono_std;
    tropo_std = obs_std.tropo_std;
    range_std = obs_std.range_std;
    flags = obs_std.flags;
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
  s32 epoch_count;

  LabeledObservation()
      : sat_unit_vector{0, 0, 0},
        measurement(),
        sat_elevation(-100),
        epoch_count(0) {}
  LabeledObservation(Eigen::Vector3d unit_vector, double meas, double elevation)
      : sat_unit_vector(std::move(unit_vector)),
        measurement(meas),
        sat_elevation(elevation),
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

using ObservationCodeSet =
    pvt_common::containers::Set<code_t, static_cast<s32>(CODE_COUNT)>;

}  // namespace obs_filters

struct UndifferencedObservations {
  gps_time_t obs_time_;
  u8 num_obs_;
  navigation_measurement_t nav_meas_[cMaxInputTrackingChannels];
  pvt_common::containers::Map<gnss_signal_t, measurement_std_t,
                              pvt_engine::cMaxInputTrackingChannels>
      obs_std_;

  UndifferencedObservations()
      : obs_time_(GPS_TIME_UNKNOWN), num_obs_(0), nav_meas_(), obs_std_() {}

  UndifferencedObservations(
      const gps_time_t &obs_time, const u8 num_obs,
      const navigation_measurement_t nav_meas[],
      const pvt_engine::obs_filters::ConstellationSet &supported_constellations,
      const pvt_engine::obs_filters::FrequencySet &supported_frequencies)
      : obs_time_(obs_time), num_obs_(0), nav_meas_(), obs_std_() {
    const pvt_engine::FrequencyManager &frequency_manager =
        pvt_engine::FrequencyManager::instance();
    FREQUENCY freq;
    for (s32 index = 0; index < num_obs; ++index) {
      if (!sid_valid(nav_meas[index].sid)) {
        detailed_log_error("index %" PRId32 " of %" PRIu8 " sid S%" PRId16
                           " C%" PRId32 " is invalid",
                           static_cast<s32>(index), static_cast<u8>(num_obs),
                           static_cast<s16>(nav_meas[index].sid.sat),
                           static_cast<s32>(nav_meas[index].sid.code));
        assert(0);
      }
      frequency_manager.get_frequency_enum(nav_meas[index].sid, &freq);
      if (supported_constellations.contains(
              sid_to_constellation(nav_meas[index].sid)) &&
          supported_frequencies.contains(freq)) {
        nav_meas_[num_obs_++] = nav_meas[index];
      }
    }
  }

  UndifferencedObservations(
      const gps_time_t &obs_time, const u8 num_obs,
      const navigation_measurement_t nav_meas[],
      const pvt_common::containers::Map<gnss_signal_t, measurement_std_t,
                                        pvt_engine::cMaxInputTrackingChannels>
          obs_std,
      const pvt_engine::obs_filters::ConstellationSet &supported_constellations,
      const pvt_engine::obs_filters::FrequencySet &supported_frequencies)
      : UndifferencedObservations(obs_time, num_obs, nav_meas,
                                  supported_constellations,
                                  supported_frequencies) {
    obs_std_ = obs_std;
  }

  UndifferencedObservations(
      const gps_time_t &obs_time, const u8 num_obs,
      const navigation_measurement_t nav_meas[],
      const pvt_engine::obs_filters::ObservationCodeSet &supported_codes)
      : obs_time_(obs_time), num_obs_(0), nav_meas_(), obs_std_() {
    for (s32 index = 0; index < num_obs; ++index) {
      if (supported_codes.contains(nav_meas[index].sid.code)) {
        nav_meas_[num_obs_++] = nav_meas[index];
      }
    }
  }

  void clear() {
    obs_time_ = GPS_TIME_UNKNOWN;
    num_obs_ = 0;
  }
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_H
