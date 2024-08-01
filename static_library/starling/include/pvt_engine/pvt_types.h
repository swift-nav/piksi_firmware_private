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

#ifndef LIBSWIFTNAV_PVT_ENGINE_PVT_TYPES_H
#define LIBSWIFTNAV_PVT_ENGINE_PVT_TYPES_H

#include <bitset>

#include <pvt_common/containers/map.h>
#include <pvt_common/containers/static_vector.h>
#include <pvt_engine/eigen_types.h>
#include <starling/observation.h>

namespace pvt_engine {

/** /defgroup PositioningTypes
 * Definitions of all types used throughout pvt_engine
 */

#define TIME_MATCH_THRESHOLD 2e-2

constexpr s32 INVALID_INDEX = -1;

enum POSITION_STATE_LABELS {
  POSITION_X = 0,
  POSITION_Y,
  POSITION_Z,
  VELOCITY_X,
  VELOCITY_Y,
  VELOCITY_Z,
  ACCEL_X,
  ACCEL_Y,
  ACCEL_Z,
  MAX_POSITION_STATE = ACCEL_Z + 1
};

// The size of this bitfield must be larger than the number of corrections that
// can be applied below.
using applied_model_bitfield = std::bitset<32>;

namespace correction_marker {
enum CORRECTION_MARKER_FLAG {
  KLOBUCHAR_IONO = 0,
  TROPO,
  OCEAN_TIDE_LOADING,
  PHASE_WINDUP,
  POLE_TIDE,
  RCV_ANT,
  SAT_ANT,
  SOLID_EARTH_TIDE,
  GENERAL_RELATIVITY,
  SBAS_RANGE,
  SBAS_RANGE_RATE,
  SBAS_IONO,
  DIFFERENTIAL,
  SSR_RANGE,
  BDS_CODE_BIAS
};
}  // namespace correction_marker

enum IONO_CORRECTION_TYPE {
  NO_APRIORI_IONO = 0,
  DIFFERENTIAL_IONO,
  ESTIMATED_IONO,
  KLOBUCHAR,
  SBAS_IONO,
  NUM_IONO_CORRECTION_TYPE
};

enum TROPO_CORRECTION_TYPE {
  NO_APRIORI_TROPO = 0,
  DIFFERENTIAL_TROPO,
  ESTIMATED_TROPO,
  APRIORI_ZENITH_TROPO,
  NUM_TROPO_CORRECTION_TYPE
};

enum ORBIT_CLOCK_CORRECTION_TYPE {
  GPS_BROADCAST = 0,
  GLO_BROADCAST,
  BDS2_BROADCAST,
  GAL_BROADCAST,
  DIFFERENTIAL_ORBIT_CLOCK,
  SSR_ORBIT_CLOCK,
  SBAS_ORBIT_CLOCK,
  NUM_ORBIT_CLOCK_CORRECTION_TYPE
};

enum MODEL_TYPE {
  POSITION_MODEL_SINGLE_DIFFERENCED = 0,
  POSITION_MODEL_UNDIFFERENCED,
  DECOUPLED_CLOCK_MODEL,
  SINGLE_CLOCK_MODEL,
  SINGLE_CLOCK_PER_CONSTELLATION_MODEL,
  HARDWARE_BIASES_MODEL,
  IONOSPHERE_MODEL,
  TROPOSPHERE_MODEL,
  PHASE_WINDUP_MODEL,
  AMB_MODEL,
  MAX_MODEL_NUM
};

using ObsProcModelTypes =
    pvt_common::containers::StaticVector<MODEL_TYPE, MAX_MODEL_NUM>;

enum APRIORI_MODEL_TYPE {
  INVALID_APRIORI_MODEL = -1,
  ELEVATION_APRIORI_MODEL,
  CN0_MASK_APRIORI_MODEL,
  IONOSPHERE_APRIORI_MODEL,
  TROPOSPHERE_APRIORI_MODEL,
  LOSS_OF_LOCK_APRIORI_MODEL,
  SOLID_EARTH_TIDES_APRIORI_MODEL,
  OCEAN_TIDE_LOADING_APRIORI_MODEL,
  POLE_TIDES_APRIORI_MODEL,
  PHASE_WINDUP_APRIORI_MODEL,
  RECEIVER_ANTENNA_OFFSET_APRIORI_MODEL,
  SATELLITE_ANTENNA_OFFSET_APRIORI_MODEL,
  MIN_MAX_SATS_APRIORI_MODEL,
  COMPUTED_DOPPLER_APRIORI_MODEL,
  GENERAL_RELATIVITY_APRIORI_MODEL,
  SBAS_APRIORI_MODEL,
  BEIDOU_CODE_BIAS_APRIORI_MODEL,
  MAX_APRIORI_MODEL_NUM
};

using AprioriModelTypes =
    pvt_common::containers::StaticVector<APRIORI_MODEL_TYPE,
                                         MAX_APRIORI_MODEL_NUM>;

enum OBSERVATION_TYPE {
  INVALID_OBSERVATION_TYPE = -1,
  PSEUDORANGE = 0,
  CARRIER_PHASE,
  MEASURED_DOPPLER,
  COMPUTED_DOPPLER,
  BASELINE_MAGNITUDE,
  MAX_OBSERVATION_TYPE
};

inline std::string observation_type_to_string(const OBSERVATION_TYPE &type) {
  switch (type) {
    case PSEUDORANGE:
      return "pseudorange";
    case CARRIER_PHASE:
      return "carrier phase";
    case MEASURED_DOPPLER:
      return "measured doppler";
    case COMPUTED_DOPPLER:
      return "computed doppler";
    case BASELINE_MAGNITUDE:
      return "baseline magnitude";
    case INVALID_OBSERVATION_TYPE:
    case MAX_OBSERVATION_TYPE:
    default:
      return "<invalid observation type>";
  }
}

inline OBSERVATION_TYPE string_to_observation_type(
    const std::string &state_label) {
  if (state_label.find("pseudorange") == std::string::npos) {
    return PSEUDORANGE;
  }
  if (state_label.find("carrier phase") == std::string::npos) {
    return CARRIER_PHASE;
  }
  if (state_label.find("measured doppler") == std::string::npos) {
    return MEASURED_DOPPLER;
  }
  if (state_label.find("computed doppler") == std::string::npos) {
    return COMPUTED_DOPPLER;
  }
  if (state_label.find("baseline magnitude") == std::string::npos) {
    return BASELINE_MAGNITUDE;
  }
  assert(false);
}

enum FREQUENCY {
  INVALID_FREQUENCY = -1,
  L1_FREQUENCY = 0,
  L2_FREQUENCY,
  L5_FREQUENCY,
  MAX_FREQUENCY
};
static const FREQUENCY frequencies[MAX_FREQUENCY] = {L1_FREQUENCY, L2_FREQUENCY,
                                                     L5_FREQUENCY};

std::string frequency_to_string(const FREQUENCY &freq);
FREQUENCY string_to_frequency(const std::string &freq_str);
bool is_valid_constellation_frequency_pair(
    const constellation_t &constellation,
    const pvt_engine::FREQUENCY &frequency);
code_t constellation_frequency_to_code(const constellation_t &constellation,
                                       const pvt_engine::FREQUENCY &frequency);

enum ESTIMATOR_TYPE {
  NO_FILTER = -1,
  KALMAN_FILTER,
  UD_FILTER,
  MAX_ESTIMATOR_TYPE
};

enum OBSERVATION_MODEL_POSITION_MODE {
  INVALID_POSITION_MODE = -1,
  POSITION = 0,
  POSITION_VELOCITY,
  POSITION_VELOCITY_ACCELERATION,
  MAX_POSITION_MODE = POSITION_VELOCITY_ACCELERATION + 1
};

enum OBSERVATION_PROCESS_NOISE_BASELINE {
  POSITION_INDEX = 0,
  VELOCITY_INDEX,
  ACCELERATION_INDEX
};

enum REFERENCE_STATION_POSITION_SOURCE {
  INVALID = -1,
  KNOWN_SURVEYED = 0,
  SPP
};

static inline bool validate_known_position_source(
    REFERENCE_STATION_POSITION_SOURCE kps) {
  return kps >= KNOWN_SURVEYED && kps <= SPP;
}

struct DOPS {
  // We currently don't really need GDOP or TDOP as we aren't interested in our
  // time accuracy, they are also harder to calculate due to our current filter
  // structure. We should also add HDOP and VDOP once they are calculated
  double GDOP;
  double PDOP;
  double HDOP;
  double VDOP;
  double TDOP;
  double ADOP_from_filter;

  DOPS()
      : GDOP(0.0),
        PDOP(0.0),
        HDOP(0.0),
        VDOP(0.0),
        TDOP(0.0),
        ADOP_from_filter(0.0){};
};

enum WEIGHTING_MODEL { CONSTANT, ELEVATION };

enum TROPO_MODEL { UNB3M, MODIFIED_HOPFIELD, GPT2 };

enum PROCESSING_MODE { TIME_MATCHED, LOW_LATENCY };

enum CORRECTION_TYPE { BROADCAST, SSR };

enum FIXING_TYPE { FIXING_ALL_SIGNALS, FIXING_WIDELANE_ONLY };

// Correction source, currently used to determine the amount of process noise to
// be added to the ambiguities in order to compensate for changes in the
// orbit/clock correction errors. NON_DIFFERENTIAL is the PPP scenario, in which
// case we currently don't add any noise because it's close to being static and
// these types of errors aren't a problem in that scenario; SINGLE_BASE has no
// such errors, so we need not add any process noise and ORION is the case in
// which we need to add some noise.
enum CORRECTION_SOURCE {
  NON_DIFFERENTIAL,
  SINGLE_BASE,
  ORION,
  MAX_CORRECTION_SOURCES
};

// Target Integrity Risk is used to flag protection levels according to
// their associated probability of fault occurrence per unit of time.
enum class TIR : u8 {
  SAFE_STATE = 0,
  LEVEL_1,
  LEVEL_2,
  LEVEL_3,
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_PVT_TYPES_H
