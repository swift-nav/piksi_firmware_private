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

#ifndef LIBSWIFTNAV_PVT_ENGINE_BASELINE_H
#define LIBSWIFTNAV_PVT_ENGINE_BASELINE_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/filter.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

namespace position {

PRC fixed_position_solution(
    const VectorMaxStateDimd_t &resolved_float_states,
    const VectorMaxStateDimd_t &unresolved_float_states,
    const ambiguities::TransformedIntegerAmbiguities &fixed_ambs,
    const ambiguities::AmbiguityIndexMap &float_amb_indices,
    const MatrixMaxStateDimd_t &Qrr, const MatrixMaxStateDimd_t &Qur,
    const MatrixMaxStateDimd_t &Quu,
    VectorMaxStateDimd_t *unresolved_fixed_states,
    MatrixMaxStateDimd_t *unresolved_states_covariance);

void conditional_solution(const VectorMaxStateDimd_t &unresolved_float_states,
                          const MatrixMaxStateDimd_t &Qrr,
                          const MatrixMaxStateDimd_t &Qur,
                          const MatrixMaxStateDimd_t &Quu,
                          const MatrixMaxStateDimd_t &T,
                          const VectorMaxStateDimd_t &T_times_fixed_minus_float,
                          VectorMaxStateDimd_t *unresolved_fixed_states,
                          MatrixMaxStateDimd_t *unresolved_states_covariance);

PRC get_float_position(const Filter &float_filter,
                       VectorMaxBaselineStatesd_t *float_position,
                       MatrixMaxBaselineStatesd_t *float_position_covariance);

PRC get_fixed_position(
    const Filter &float_filter, const FloatSolutionTypeToUse &float_type_to_use,
    const optional<glo_biases_t> &glo_biases,
    const ambiguities::TransformedIntegerAmbiguities &validated_ambiguities,
    FilterState *position);

VectorMaxStateDimd_t convert_to_baseline(
    const bool &is_relative_position, const CommonData &common_data,
    const VectorMaxStateDimd_t &state_estimate);

}  // namespace position

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_BASELINE_H
