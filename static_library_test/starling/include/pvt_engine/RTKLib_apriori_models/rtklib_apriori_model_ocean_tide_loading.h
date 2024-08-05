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

#ifndef LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_OCEAN_TIDE_LOADING_H
#define LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_OCEAN_TIDE_LOADING_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_common_tides.h>
#include <pvt_engine/common_data.h>
#include <pvt_engine/ocean_tide_loading_model.h>

namespace pvt_engine {

void otl_disp(const gps_time_t &time, const Eigen::Vector3d &rec_pos,
              const double *odisp, Eigen::Vector3d *displacement);
u16 readblq(const char *file, const char *sta, double *odisp);

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_RTK_LIB_APRIORI_MODEL_OCEAN_TIDE_LOADING_H
