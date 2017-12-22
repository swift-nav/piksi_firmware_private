/*
 * Copyright (C) 2010 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *          Matt Peddie <peddie@alum.mit.edu>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_PVT_H
#define SWIFTNAV_PVT_H

#include <libswiftnav/common.h>
#include <libswiftnav/dops.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/nav_meas.h>
#include <libswiftnav/troposphere.h>
#include "utils/sid_set.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define PVT_MAX_ITERATIONS 10

#define MIN_SATS_FOR_PVT 5

extern const char *pvt_err_msg[7];

#define PVT_CONVERGED_NO_RAIM 2
#define PVT_CONVERGED_RAIM_REPAIR 1
#define PVT_CONVERGED_RAIM_OK 0
#define PVT_PDOP_TOO_HIGH -1
#define PVT_BAD_ALTITUDE -2
#define PVT_VELOCITY_LOCKOUT -3
#define PVT_RAIM_REPAIR_FAILED -4
#define PVT_RAIM_REPAIR_IMPOSSIBLE -5
#define PVT_UNCONVERGED -6
#define PVT_INSUFFICENT_MEAS -7

/* coefficient for elevation term (estimated from Piksi v3 data) */
#define ELEVATION_NOISE_COEFFICIENT 0.5

/* Measurement noise model parameters */
/* TODO: tuning */
#define GPS_CODE_CN0_COEFFICIENT 25000.0
#define GPS_DOPPLER_CN0_COEFFICIENT 10000.0
#define GPS_CARRIER_CN0_COEFFICIENT 0.9
#define GPS_PSEUDORANGE_VARIANCE 2.25
/* TODO: initial GLO parameters just the GPS ones multiplied roughly by 4 */
#define GLO_CODE_CN0_COEFFICIENT 100000.0
#define GLO_DOPPLER_CN0_COEFFICIENT 40000.0
#define GLO_CARRIER_CN0_COEFFICIENT 3.0
#define GLO_PSEUDORANGE_VARIANCE 25.0

#define NO_PLL_MULTIPLIER 16.0
#define NO_HALF_CYCLE_MULTIPLIER 9.0
#define LOCK_TIME_THRESHOLD_S 0.5
#define SHORT_LOCK_TIME_MULTIPLIER 4.0
#define TRACK_TIME_THRESHOLD_S 4.0
#define SHORT_TRACK_TIME_MULTIPLIER 4.0
#define NOMINAL_CLOCK_VARIANCE 0.002 /* cycles^2 / s */

/* RAIM parameters */
/* Maximum number of signals to exclude. (If problems are found with multiple
 * signals, then there is some more profound problem and RAIM is unlikely to
 * help.) */
#define RAIM_MAX_EXCLUSIONS 2
/* RAIM metric threshold (unitless), typical range 1 - 10.
 * This is in units of noise sigmas. Too small values lead to unnecessary
 * exclusions, while larger values let larger outliers through. */
#define RAIM_METRIC_THRESHOLD 2.5
/* Dimension of state vector, currently 4 (position + clock bias)*/
#define RAIM_N_STATE 4

typedef struct __attribute__((packed)) {
  /*
   * Be careful of stuct packing to avoid (very mild) slowness,
   * try to keep all the types aligned i.e. put the 64bit
   * things together at the top, then the 32bit ones etc.
   */
  /** Receiver position latitude [deg], longitude [deg], altitude [m] */
  double pos_llh[3];
  /** Receiver position ECEF XYZ [m] */
  double pos_ecef[3];
  /** Receiver velocity in NED [m/s] */
  double vel_ned[3];
  /** Receiver velocity in ECEF XYZ [m/s] */
  double vel_ecef[3];

  /* This is the row-first upper diagonal matrix of error covariances
   * in x, y, z (all receiver clock covariance terms are ignored).  So
   * it goes like so:
   *
   *    0  1  2
   *    _  3  4
   *    _  _  5
   *
   *    Index 6 is the GDOP.
   */
  double err_cov[7];

  /* Upper diagonal of the covariances of the velocity solution, similarly
   * as above, but without the DOP element.
   */
  double vel_cov[7];

  double clock_offset;
  double clock_offset_var;
  double clock_drift;
  double clock_drift_var;

  /* GPS time */
  gps_time_t time;

  /* 0 = invalid, 1 = code phase */
  u8 valid;
  /* 0 = invalid, 1 = doppler */
  u8 velocity_valid;
  /* Number of satellites used in the soluton. */
  u8 n_sats_used;
  /* Number of signals used in the soluton. */
  u8 n_sigs_used;
} gnss_solution;

s8 calc_PVT(const u8 n_used,
            const navigation_measurement_t nav_meas[],
            bool disable_raim,
            bool disable_velocity,
            gnss_solution *soln,
            dops_t *dops,
            gnss_sid_set_t *raim_removed_sids);

void calc_iono_tropo(u8 n_ready_tdcp,
                     navigation_measurement_t *nav_meas_tdcp,
                     const double *pos_ecef,
                     const double *pos_llh,
                     const ionosphere_t *iono_params);

void calc_measurement_noises(const navigation_measurement_t *nav_meas,
                             double *p_pseudorange_var,
                             double *p_carrier_phase_var,
                             double *p_measured_doppler_var,
                             double *p_computed_doppler_var);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_PVT_H */
