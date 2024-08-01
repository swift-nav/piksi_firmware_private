/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_ARAIM_FAILURE_EVENT_H
#define STARLING_ARAIM_FAILURE_EVENT_H

#include <pvt_common/containers/set.h>
#include <pvt_engine/araim/araim_definitions.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/sat_identifier.h>

namespace pvt_engine {
namespace araim {

// ----------------------------------------------------------------------------
// About:
//
// A failure event could be any event which would inhibit localization of a
// receiver using signals from the GNSS space segment. For ARAIM purposes,
// we divide failure events up into independent satellite- and
// constellation-level failures, each with an assigned prior probability. In
// theory we could be even more granular (e.g. code-level), but this is not
// presently considered by the ARAIM algorithm.
//  Examples: - Satellite GPS-01 failure
//            - GAL constellation failure
// ----------------------------------------------------------------------------

enum class FailureType {
  SATELLITE_FAILURE = 0,
  CONSTELLATION_FAILURE,
  INVALID_FAILURE_TYPE
};

// TODO(STAR-925) Implement a robust manner of handling FailureEvent id members
struct FailureEvent {
  FailureType type;
  double P_event;
  union {
    SatIdentifier satellite_id;
    constellation_t constellation_id;
  };

  FailureEvent()
      : type(FailureType::INVALID_FAILURE_TYPE), P_event(0.0), satellite_id() {}

  FailureEvent(const SatIdentifier &sat_id, const double &probability)
      : type(FailureType::SATELLITE_FAILURE),
        P_event(probability),
        satellite_id(sat_id) {}

  FailureEvent(const constellation_t constellation, const double &probability)
      : type(FailureType::CONSTELLATION_FAILURE),
        P_event(probability),
        constellation_id(constellation) {}

  bool operator==(const FailureEvent &other) const {
    if (type == other.type &&
        std::fabs(P_event - other.P_event) < FLOAT_EQUALITY_EPS) {
      switch (type) {
        case FailureType::SATELLITE_FAILURE:
          return (satellite_id == other.satellite_id);
        case FailureType::CONSTELLATION_FAILURE:
          return (constellation_id == other.constellation_id);
        case FailureType::INVALID_FAILURE_TYPE:
        default:
          return true;
      }
    }

    return false;
  }

  bool operator!=(const FailureEvent &other) const { return !(*this == other); }

  bool operator<(const FailureEvent &other) const {
    if (other.type == type) {
      switch (type) {
        case FailureType::SATELLITE_FAILURE:
          if (satellite_id == other.satellite_id) {
            return P_event < other.P_event;
          }
          return satellite_id < other.satellite_id;
        case FailureType::CONSTELLATION_FAILURE:
          if (constellation_id == other.constellation_id) {
            return P_event < other.P_event;
          }
          return constellation_id < other.constellation_id;
        case FailureType::INVALID_FAILURE_TYPE:
        default:
          return P_event < other.P_event;
      }
    }

    return type < other.type;
  }
};

// A large vector to store a list of FailureEvents
//  Ex. List of FailureEvents associated with all the satelites in view
using FailureEventVector =
    pvt_common::containers::StaticVector<FailureEvent, cMaxFailureEvents>;
// A small set to store combinations of FailureEvents
//  Ex. Subset of FailureEvents that define a FaultMode
using FailureEventSubset =
    pvt_common::containers::Set<FailureEvent,
                                cMaxAllowableSimultaneousFailures>;

double sum_probability_of_events(const FailureEventVector &events);
double concurrent_probability_of_events(const FailureEventVector &events);
double complement_probability_of_events(const FailureEventVector &events);

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_ARAIM_FAILURE_EVENT_H
