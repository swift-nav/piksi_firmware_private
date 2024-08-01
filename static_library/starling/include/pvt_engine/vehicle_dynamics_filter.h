/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_VEHICLE_DYNAMICS_FILTER
#define LIBSWIFTNAV_PVT_ENGINE_VEHICLE_DYNAMICS_FILTER

/**
 * Vehicle Dynamics Filter
 * =======================
 *
 * June 2018
 *
 * The Swift PVT engine is tuned to work in a wide variety of scenarios.
 * While the GNSS filters themselves do have mechanisms for incorporating
 * vehicle dynamics, the larger Starling system will always suffer from
 * discontinuities as it switches filter modes. For certain situations,
 * this behavior is undesirable, so we also provide an additional filter
 * which is capable of introducing certain vehicle dynamics on top of
 * an arbitrary PVT solution sequence.
 *
 * The main benefit to having this separate filter is that it separates
 * the concern of vehicle dynamics from our primary GNSS filters. We can
 * also hopefully focus our efforts on the few cases required for customers.
 *
 * Ultimately, as we explore the possiblity of tightly coupling inertial
 * measurements (and other) into our GNSS filters, this extra filter
 * will become obsolete. It is merely intended to meet an immediate
 * customer need for the interim.
 *
 * OPERATION:
 * The vehicle dynamics filter presents a standard object-oriented interface.
 * An instance of the filter takes in a trajectory of PVT results and attempts
 * to impose on it a specific set of dynamic constraints.
 *
 * The interface is written in "C", as it is intended for immediate use
 * on Piksi Multi. The implementation leverages our existing IP to the maximum
 * extent possible.
 */

#include <pvt_engine/firmware_binding.h>

#ifdef __cplusplus
namespace pvt_engine {
extern "C" {
#endif

typedef struct VehicleDynamicsFilter VehicleDynamicsFilter;

typedef enum VehicleDynamicsFilterType {
  /* Input is passed to output unfiltered. */
  DYNAMICS_NONE,
  /* Assume dynamics of tractor performing agriculture tasks. */
  DYNAMICS_TRACTOR,
  /* Assume dynamics of a standard quad/hex rotor UAV. */
  DYNAMICS_DRONE,
  /* Assume dynamics of an FTL voyager outfitted with warp drive technology. */
  DYNAMICS_INTERGALACTIC_VOYAGER,
  /* Assume dynamics of a Sedan with multiple receivers. */
  DYNAMICS_SEDAN_MULTIPLE_RECEIVERS,
} VehicleDynamicsFilterType;

/* Instantiate a new vehicle dynamics filter object. The parameter
 * determines which filter implementation is used. */
VehicleDynamicsFilter *vehicle_dynamics_filter_create(
    VehicleDynamicsFilterType dynamics_type);

/* Cleanup an instantiated vehicle dynamics filter object. */
void vehicle_dynamics_filter_destroy(VehicleDynamicsFilter *self);

/* Query the type of a filter object. */
VehicleDynamicsFilterType vehicle_dynamics_filter_get_type(
    const VehicleDynamicsFilter *self);

/* Reset the filter. The actual internal initial condition is
 * implementation-dependent, and will vary according to the
 * dynamics. */
void vehicle_dynamics_filter_reset(VehicleDynamicsFilter *self);

/* Process a PVT solution and return the resulting filtered solution. */
void vehicle_dynamics_filter_process(VehicleDynamicsFilter *self,
                                     const pvt_engine_result_t *input_result,
                                     pvt_engine_result_t *output_result);

/**
 * Everything beyond this point is for configuration and tuning purposes.
 */
typedef enum VehicleDynamicsFilterParamType {
  /* Time constant of low-pass filter used to "smooth" the output.
   * units: s */
  VEHICLE_DYNAMICS_LOWPASS_TIME_CONSTANT_S,
  /* Maximum magnitude of realistic linear acceleration for the vehicle.
   * units: m/s^2 */
  VEHICLE_DYNAMICS_MAX_LINEAR_ACCELERATION_MS2,
  /* Relative weighting of max vertical to horizontal acceleration.
   * units: [none] */
  VEHICLE_DYNAMICS_RELATIVE_VERTICAL_ACCELERATION,
  /* Scaling applied to secondary measurements when primary is unavailable */
  VEHICLE_DYNAMICS_BIAS_VARIANCE_SCALING,
  /* Scaling applied to bias process noise for each time/process update */
  VEHICLE_DYNAMICS_BIAS_PROCESS_NOISE_SCALING,
} VehicleDynamicsFilterParamType;

struct VehicleDynamicsFilterParams {
  /* Time-constant on the lowpass filter of the measurement covariance
   * matrices [s]. */
  double covariance_filter_time_constant;
  /* Prior on how much the vehicle can acceleration in each direction */
  double prior_acceleration_variance_horiz;
  double prior_acceleration_variance_vert;
  /* Prior on the position bias states [m]^2 */
  double bias_process_noise_scaling;
  /* Prior used to "guess" velocity in the absence of valid observations,
   * (m/s)^2. */
  double prior_velocity_variance;
  /* Maximum permissible update interval in seconds. */
  double max_update_interval;
  /* Variance associated with position bias states [m]^2 */
  double bias_measurement_scaling;
};

/* Set a specific parameter in the filter. Some parameter types apply
 * only to certain filter types.
 * TODO(kevin) sort this mechanism out. */
void vehicle_dynamics_filter_set_param(VehicleDynamicsFilter *self,
                                       VehicleDynamicsFilterParamType param,
                                       double value);

#ifdef __cplusplus
}
}  // namespace pvt_engine
#endif

#endif
