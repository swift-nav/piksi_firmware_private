/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef PVT_DRIVER_SOLUTIONS_H
#define PVT_DRIVER_SOLUTIONS_H

#include <pvt_common/observations.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/single_epoch_solver.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Various solution output modes supported by the Starling Engine.
 *
 * LOW_LATENCY:  Output a solution immediately for every rover observation.
 * TIME_MATCHED: Output a solution whenever a base observation is received.
 * NO_DGNSS:     Output single-point low-latency solution.
 */
typedef enum {
  PVT_DRIVER_SOLN_MODE_LOW_LATENCY,
  PVT_DRIVER_SOLN_MODE_TIME_MATCHED,
  PVT_DRIVER_SOLN_MODE_NO_DGNSS
} pvt_driver_solution_mode_t;

/**
 * Target integrity risk levels
 */
#define PVT_DRIVER_TIR_SAFE_STATE 0
#define PVT_DRIVER_TIR_LEVEL_1 1
#define PVT_DRIVER_TIR_LEVEL_2 2
#define PVT_DRIVER_TIR_LEVEL_3 3
#define PVT_DRIVER_TIR_LEVEL_MASK 0x07
#define PVT_DRIVER_TIR_LEVEL_OFFSET 0

typedef struct {
  uint8_t tir_level : 3;
} pvt_driver_protection_level_flags_t;

typedef struct {
  bool valid;
  double protection_levels_ned[3];
  double pos_ecef[3];
  pvt_driver_protection_level_flags_t flags;
} pvt_driver_protection_levels_t;

/**
 * Filter result data type returned by various API functions.
 */
typedef struct {
  dops_t dops;
  pvt_engine_result_t result;
  pvt_driver_protection_levels_t pl;
} pvt_driver_filter_solution_t;

/**
 * Contains function pointers for handling PVT engine solutions.
 *
 * Either of the function pointers can be set to NULL if that particular
 * solution is not of interest.
 */
typedef struct {
  /**
   * User handling of a low-latency solution.
   *
   * At every processing epoch, possible solutions include both
   * an SPP solution, and an RTK solution. Either one may be invalid,
   * (indicated by NULL pointer), although existence of an RTK
   * solution implies existence of an SPP solution.
   *
   * NOTE: The pointers are only valid within the enclosing scope.
   *       Any copies of the data must be deep copies.
   */
  void (*handle_low_latency)(const pvt_driver_filter_solution_t *spp_solution,
                             const pvt_driver_filter_solution_t *rtk_solution,
                             const gps_time_t *solution_epoch_time, void *ctx);

  /**
   * User handling of a time-matched solution.
   *
   * The solution pointer may optionally be NULL if there was no
   * valid solution for this epoch of processing. The observation
   * pointers are expected to always be valid.
   *
   * NOTE: The pointers are only valid within the enclosing scope.
   *       Any copies of the data must be deep copies.
   */
  void (*handle_time_matched)(const pvt_driver_filter_solution_t *solution,
                              const obs_core_t *obs_base,
                              const pvt_engine_result_t *rover_soln,
                              const double *spp_ecef, void *ctx);

  /**
   * The context information to pass into the two functions when called.
   */
  void *ctx;
} pvt_driver_solution_callback_t;

#ifdef __cplusplus
}
#endif

#endif  // PVT_DRIVER_SOLUTIONS_H
