/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_OTL_PARAMS_H
#define LIBSWIFTNAV_PVT_ENGINE_OTL_PARAMS_H

#include <pvt_common/containers/static_vector.h>
#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/eigen_types.h>

namespace pvt_engine {

using OtlParamsVec =
    pvt_common::containers::StaticVector<double, NUM_TIDE_LOADING_PARAMS>;

struct OtlParams {
  OtlParamsVec params;
  bool is_full() const;

  OtlParams();
  OtlParams(std::initializer_list<double> vals);
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OTL_PARAMS_H
