
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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OUTLIER_UTILS_H
#define LIBSWIFTNAV_PVT_ENGINE_OUTLIER_UTILS_H

#include <pvt_engine/optional.h>

#include <swiftnav/common.h>

#include <pvt_engine/common_data.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

optional<double> compute_chi_square_statistic(
    const VectorMaxObsd_t &residual, const MatrixMaxObsd_t &covariance);

bool chi_squared_threshold_test(const double chi_statistic,
                                double degrees_of_freedom,
                                const double false_positive_threshold);

bool f_threshold_test(const double f_stat, const double DoF);

double inverse_cumulative_chi_squared(const double degrees_of_freedom,
                                      const double false_positive_threshold);

optional<bool> consistency_check(const VectorMaxObsd_t &residuals,
                                 const MatrixMaxObsd_t &residual_cov_matrix,
                                 const s32 number_of_unknowns,
                                 const double false_positive_threshold);

};  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_CYCLE_SLIP_MANAGER_H
