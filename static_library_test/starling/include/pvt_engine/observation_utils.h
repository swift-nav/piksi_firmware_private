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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_UTILS_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_UTILS_H

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/propagate.h>

namespace pvt_engine {

namespace obs_utils {

bool is_flag_set(const nav_meas_flags_t &obs_flags,
                 const nav_meas_flags_t &flag);

bool any_have_nav_meas_flag(const UndifferencedObservations &observations,
                            const nav_meas_flags_t &flag);

void add_measured_doppler(const Eigen::Vector3d &station_position,
                          UndifferencedObservations *observations);

}  // namespace obs_utils

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_UTILS_H
