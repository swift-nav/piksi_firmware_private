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

#ifndef STARLING_ARAIM_H
#define STARLING_ARAIM_H

#include <pvt_engine/araim/araim_observation_data.h>
#include <pvt_engine/araim/fault_mode.h>
#include <pvt_engine/integrity_support_message.h>

namespace pvt_engine {
namespace araim {

// ----------------------------------------------------------------------------
// About:
//
// The following functions make up the core implementation of the ARAIM user
// algorithm. All methods are derived from the EU-US ARAIM Technical
// Subgroup Milestone 3 Report (heretofore referred to as the "ARAIM
// report"). Specific citations will be listed where appropriate, including
// section names, equation numbers, and/or page numbers.
// ----------------------------------------------------------------------------

FailureEventVector list_failure_events(const SatIdSet &sats_in_view,
                                       const IntegritySupportMessage &ism);

s32 compute_max_simultaneous_faults(const FailureEventVector &failure_events,
                                    const double &P_thres);

FaultModeVector define_fault_modes(const AraimObservationData &obs,
                                   const FailureEventVector &failure_events,
                                   const s32 &N_fault_max);

FaultModeVector filter_out_unobservable_fault_modes(
    const AraimObservationData &obs, FaultModeVector *fault_modes);

void evaluate_fault_mode_solution(const AraimObservationData &obs,
                                  FaultMode *fault_mode);

void map_all_solutions_to_common_ned_frame(const Eigen::Vector3d &x_ref_ecef,
                                           FaultModeVector *fault_modes);

void evaluate_fault_mode_separation(const FaultMode &all_in_view,
                                    const MatrixMaxObsd_t &C_acc,
                                    const s32 &N_fault_modes,
                                    const Eigen::Vector3d &P_FA,
                                    FaultMode *fault_mode);

void evaluate_fault_mode_position_bias(const VectorMaxObsd_t &b_nom,
                                       FaultMode *fault_mode);

FaultModeVector filter_out_solution_separation_failures(
    FaultModeVector *fault_modes);

double probability_of_unmonitored_fault_modes(
    const FailureEventVector &failure_events, const s32 &N_fault_max);

double probability_of_unobservable_fault_modes(
    const FaultModeVector &unobservable_fault_modes);

Eigen::Vector3d allocated_integrity_risk(const Eigen::Vector3d &PHMI,
                                         const double &P_unmonitored);

optional<Eigen::Vector3d> evaluate_protection_levels(
    const FaultModeVector &fault_modes,
    const Eigen::Vector3d &allocated_integrity_risk, const double &pl_tolerance,
    const s32 &N_iter_max);

}  // namespace araim
}  // namespace pvt_engine

#endif  // STARLING_ARAIM_H
