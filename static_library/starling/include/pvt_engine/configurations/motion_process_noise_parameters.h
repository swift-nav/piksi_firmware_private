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

#ifndef STARLING_MOTION_PROCESS_NOISE_PARAMETERS_H
#define STARLING_MOTION_PROCESS_NOISE_PARAMETERS_H

#include <starling/process_noise_motion.h>

#ifdef __cplusplus
#include <pvt_engine/configuration.h>
#else
typedef struct ProcessModelConfiguration ProcessModelConfiguration;
#endif  // __cplusplus

#ifdef __cplusplus
namespace pvt_engine {
extern "C" {
#endif  // __cplusplus

// High Dynamics values retained as previous version of variance values
static const double process_noise_motion_north_east_default[] = {25.0, 1600.0,
                                                                 1600.0};
static const double process_noise_motion_down_default[] = {25.0, 1600.0,
                                                           1600.0};
// High Horizontal Dynamics values arrived at after taking mean of optimum tuned
// variances
// from 6 drive scenarios
static const double process_noise_motion_north_east_automotive[] = {25.0, 1.805,
                                                                    25.0};
static const double process_noise_motion_down_automotive[] = {25.0, 0.0086,
                                                              25.0};
// Low Dynamics values defaulted to High Horizontal Dynamics velocity variance
// to reflect low
// variation
static const double process_noise_motion_north_east_static[] = {0.0086, 0.0086,
                                                                0.0086};
static const double process_noise_motion_down_static[] = {0.0086, 0.0086,
                                                          0.0086};
void set_process_noise_values(
    const PROCESS_NOISE_MOTION_TYPE process_noise_motion_type,
    ProcessModelConfiguration *process_model_config);
#ifdef __cplusplus
}  // extern "C"
}  // namespace pvt_engine
#endif

#endif  // STARLING_MOTION_PROCESS_NOISE_PARAMETERS_H
