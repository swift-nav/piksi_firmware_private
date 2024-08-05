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

#ifndef STARLING_ARAIM_HELPERS_H
#define STARLING_ARAIM_HELPERS_H

#include <pvt_engine/araim/araim_observation_data.h>
#include <pvt_engine/araim/failure_event.h>
#include <pvt_engine/araim/fault_mode.h>
#include <pvt_engine/integrity_support_message.h>
#include <pvt_engine/sat_identifier.h>

namespace pvt_engine {
namespace araim {

// ----------------------------------------------------------------------------
// About:
//
// The following helper functions are employed by the core ARAIM algorithm.
// These methods do not need to be exposed to the end user of the algorithm,
// so they are relegated to helper status.
//
// All methods are derived from the EU-US ARAIM Technical Subgroup Milestone
// 3 Report (heretofore referred to as the "ARAIM report"). Specific
// citations will be listed where appropriate, including section names,
// equation numbers, and/or page numbers.
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Generalized helper functions
// ----------------------------------------------------------------------------

s32 factorial(const s32 &n);

s32 binomial_coefficient(const s32 &n, const s32 &k);

// The Q function computes the tail probability of a standard Normal
// distribution given a number of Normal standard deviations as input
double Q(const double &sigma);

// The inverse of the Q function returns the number of Normal standard
// deviations associated with a given tail probability
double Q_inv(const double &tail_probability);

pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>
get_unique_constellations(const SatIdSet &sat_ids);

bool match_sat_ids(
    const pvt_common::containers::MapElement<SatIdentifier, double> &elem,
    const SatIdSet &sat_ids);

bool match_constellations(
    const pvt_common::containers::MapElement<constellation_t, double> &elem,
    pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>
        constellations);

// ----------------------------------------------------------------------------
// ARAIM-specific functionality
// ----------------------------------------------------------------------------

FaultModeVector create_fault_modes_of_size_n(
    const FailureEventVector &failure_events, const AraimObservationData &obs,
    const s32 &n);

double probability_of_no_failure(const FailureEventVector &failure_events);

double probability_of_simultaneous_failure_events(
    const FailureEventSubset &failure_events, const double &P_no_failure);

ObsIndexSubset map_failure_events_to_obs_indices(
    const FailureEventSubset &failure_events, const AraimObservationData &obs);

MatrixMaxObsByStatesd_t remove_unobserved_states_from_model(
    const MatrixMaxObsByStatesd_t &model_matrix,
    const ObsIndexSubset &excluded_obs_indices);

Eigen::Vector3d compute_solution_separation_threshold(
    const Eigen::Vector3d &sigma_ss, const double &N_fault_modes,
    const Eigen::Vector3d &P_FA);

Eigen::Vector3d initial_pl_upper_bounds(
    const FaultModeVector &fault_modes,
    const Eigen::Vector3d &allocated_integrity_risk);

Eigen::Vector3d initial_pl_lower_bounds(
    const FaultModeVector &fault_modes,
    const Eigen::Vector3d &allocated_integrity_risk);

double compute_fault_mode_integrity_risk_along_axis(const s32 &q,
                                                    const FaultMode &fault_mode,
                                                    const double &PL);

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_ARAIM_HELPERS_H
