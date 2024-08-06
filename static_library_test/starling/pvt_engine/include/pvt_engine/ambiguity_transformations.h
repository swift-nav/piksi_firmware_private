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

#ifndef STARLING_AMBIGUITY_TRANSFORMATIONS_H
#define STARLING_AMBIGUITY_TRANSFORMATIONS_H

#include <pvt_engine/ambiguity_lambda_interface.h>
#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

namespace ambiguities {

PRC construct_transformation_matrix(
    const AmbiguitiesAndCovariances &float_ambs,
    const obs_filters::CodeSet &codes_to_fix,
    const LambdaSearchStrategy &search_strategy,
    const LambdaCombinationStrategy &combination_strategy,
    MatrixMaxAmbss32_t *T);

MatrixMaxAmbss32_t construct_widelane_transformation_matrix(
    const AmbiguitiesAndCovariances &float_ambs,
    const pvt_common::containers::Map<constellation_t, code_t,
                                      CONSTELLATION_COUNT> &ref_codes,
    const ReferenceIndexMap &references, const MatrixMaxAmbss32_t &T_sd);

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // STARLING_AMBIGUITY_TRANSFORMATIONS_H
