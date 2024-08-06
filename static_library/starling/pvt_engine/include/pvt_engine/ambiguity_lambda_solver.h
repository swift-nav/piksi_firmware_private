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

#ifndef LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_LAMBDA_SOLVER_H
#define LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_LAMBDA_SOLVER_H

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

namespace ambiguities {

PRC lambda_solver(const VectorMaxAmbsd_t &float_params,
                  const MatrixMaxAmbsd_t &float_covariance, int num_candidates,
                  MatrixMaxAmbiguitySetDimd_t *lambda_solutions,
                  VectorMaxAmbiguitySetDimd_t *lambda_residuals);

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_AMBIGUITY_LAMBDA_SOLVER_H
