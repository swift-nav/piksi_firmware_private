/*
 * Copyright (C) 2010-2017 Swift Navigation Inc.
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

#include "pvt.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_meas.h>

/** Velocity solver
 *
 * Return  0 for success
 *        -1 for failure
 */
static s8 vel_solve(double rx_vel[],
                    const u8 n_used,
                    const navigation_measurement_t *nav_meas[n_used],
                    const double G[n_used][4],
                    double V[4][4],
                    double omp_doppler[n_used]) {
  /* Velocity Solution
   *
   * G matrix already exists from the position
   * solution loop through valid measurements.  Here we form satellite
   * velocity and pseudorange rate vectors -- it's the same
   * prediction-error least-squares thing, but we do only one step.
   *
   * Output the covariance matrix V of the velocity-drift solution
  */

  double pdot_pred;
  double w[n_used];

  for (u8 j = 0; j < n_used; j++) {
    /* Calculate predicted pseudorange rates from the satellite velocity
     * and the geometry matrix G which contains normalized line-of-sight
     * vectors to the satellites.
     */
    pdot_pred = -vector_dot(3, G[j], nav_meas[j]->sat_vel);

    /* Select the doppler value to use, try calculated doppler first, otherwise
     * use measured doppler. */
    double doppler = 0.0;
    double doppler_var = 0.0;
    double meas_doppler_var = 0.0;
    double comp_doppler_var = 0.0;

    calc_measurement_noises(
        nav_meas[j], NULL, NULL, &meas_doppler_var, &comp_doppler_var);

    double wavelength = sid_to_lambda(nav_meas[j]->sid);

    /* Pick either the computed or measured Doppler */
    if (0 != (nav_meas[j]->flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID)) {
      doppler = nav_meas[j]->computed_doppler;
      doppler_var = comp_doppler_var;
      log_debug_sid(nav_meas[j]->sid,
                    "comp doppler %.1f, std %g, meas doppler %.1f, std %g",
                    doppler * wavelength,
                    sqrt(doppler_var) * wavelength,
                    nav_meas[j]->measured_doppler * wavelength,
                    sqrt(meas_doppler_var) * wavelength);

    } else if (0 != (nav_meas[j]->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
      doppler = nav_meas[j]->measured_doppler;
      doppler_var = meas_doppler_var;
    } else {
      /* Some signal lacks valid Doppler, do not compute velocity.
       * (Currently either all signals have Doppler or none do, in case of
       * base station measurements) */
      rx_vel[0] = 0.0;
      rx_vel[1] = 0.0;
      rx_vel[2] = 0.0;
      rx_vel[3] = 0.0;
      return -1;
    }

    /* convert Doppler variance (Hz^2) into (m/s)^2 */
    doppler_var *= wavelength * wavelength;

    /* weighting is the inverse of variance, if defined */
    if (doppler_var > 0) {
      w[j] = 1.0 / doppler_var;
    } else {
      w[j] = 1.0;
    }

    /* The residual is due to the user's motion. */
    omp_doppler[j] = -doppler * wavelength - pdot_pred;
  }

  /* Solve the velocity update and its covariance matrix */
  s8 ret = matrix_wlsq_solve(n_used,
                             4,
                             (double *)G,
                             (double *)omp_doppler,
                             (double *)w,
                             (double *)rx_vel,
                             (double *)V);

  /* Subtract the solved receiver velocity the output omp_doppler */
  for (u8 j = 0; j < n_used; j++) {
    omp_doppler[j] -= vector_dot(3, G[j], rx_vel);
  }

  return ret;
}

static void compute_dops(const double H[4][4],
                         const double pos_ecef[3],
                         dops_t *dops) {
  /* PDOP is the norm of the position elements of tr(H) */
  double pdop_sq = H[0][0] + H[1][1] + H[2][2];
  dops->pdop = sqrt(pdop_sq);

  /* TDOP is like PDOP but for the time state. */
  dops->tdop = sqrt(H[3][3]);

  /* Calculate the GDOP -- ||tr(H)|| = sqrt(PDOP^2 + TDOP^2) */
  dops->gdop = sqrt(pdop_sq + H[3][3]);

  /* HDOP and VDOP are Horizontal and Vertical.  We could rotate H
   * into NED frame and then take the separate components, but a more
   * computationally efficient approach is to find the vector in the
   * ECEF frame that represents the Down unit vector, and project it
   * through H.  That gives us VDOP^2, then we find HDOP from the
   * relation PDOP^2 = HDOP^2 + VDOP^2. */
  double M[3][3];
  ecef2ned_matrix(pos_ecef, M);
  double down_ecef[4] = {M[2][0], M[2][1], M[2][2], 0};
  double tmp[3];
  matrix_multiply(3, 4, 1, (double *)H, down_ecef, tmp);
  double vdop_sq = vector_dot(3, down_ecef, tmp);
  dops->vdop = sqrt(vdop_sq);
  dops->hdop = sqrt(pdop_sq - vdop_sq);
}

/** Estimate measurement noises from elevation, cn0 and tracking flags.
 *
 * \param nav_meas               Measurement structure
 * \param p_pseudorange_var      Pointer for pseudorange variance [m^2]
 * \param p_carrier_phase_var    Pointer for carrier phase variance [cycles^2]
 * \param p_measured_doppler_var Pointer for Doppler variance [Hz^2]
 * \param p_computed_doppler_var Pointer for TDCP Doppler variance [Hz^2]
 */
void calc_measurement_noises(const navigation_measurement_t *nav_meas,
                             double *p_pseudorange_var,
                             double *p_carrier_phase_var,
                             double *p_measured_doppler_var,
                             double *p_computed_doppler_var) {
  double cn0_term = pow(10.0, -nav_meas->cn0 / 10.0);

  /* divide by sin-el, protect from division by zero */
  double el_term =
      ELEVATION_NOISE_COEFFICIENT / MAX(sin(nav_meas->elevation * D2R), 1e-3);

  double pseudorange_var = 0.0;
  double measured_doppler_var = 0.0;
  double computed_doppler_var = 0.0;
  double carrier_phase_var = 0.0;

  switch (nav_meas->sid.code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L1P:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2P:
    case CODE_GPS_L2CL:
      /* for pseudorange noise, use the larger of CN0 and elevation model */
      pseudorange_var = MAX(GPS_CODE_CN0_COEFFICIENT * cn0_term,
                            GPS_PSEUDORANGE_VARIANCE + el_term * el_term);
      /* Doppler and carrier phase variances are less tuned because they are
       * not used in the solution yet */
      measured_doppler_var = GPS_DOPPLER_CN0_COEFFICIENT * cn0_term;
      carrier_phase_var = GPS_CARRIER_CN0_COEFFICIENT * cn0_term;
      break;

    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
      /* for pseudorange noise, use the larger of CN0 and elevation model */
      pseudorange_var = MAX(GLO_CODE_CN0_COEFFICIENT * cn0_term,
                            GLO_PSEUDORANGE_VARIANCE + el_term * el_term);
      /* Doppler and carrier phase variances are less tuned because they are
       * not used in the solution yet */
      measured_doppler_var = GLO_DOPPLER_CN0_COEFFICIENT * cn0_term;
      carrier_phase_var = GLO_CARRIER_CN0_COEFFICIENT * cn0_term;
      break;

    case CODE_INVALID:
    case CODE_COUNT:
    case CODE_SBAS_L1CA:
    case CODE_GPS_L2CX:
    case CODE_GPS_L5I:
    case CODE_GPS_L5Q:
    case CODE_GPS_L5X:
    case CODE_BDS2_B11:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E1C:
    case CODE_GAL_E1X:
    case CODE_GAL_E6B:
    case CODE_GAL_E6C:
    case CODE_GAL_E6X:
    case CODE_GAL_E7I:
    case CODE_GAL_E7Q:
    case CODE_GAL_E7X:
    case CODE_GAL_E8:
    case CODE_GAL_E5I:
    case CODE_GAL_E5Q:
    case CODE_GAL_E5X:
    case CODE_QZS_L1CA:
    case CODE_QZS_L2CM:
    case CODE_QZS_L2CL:
    case CODE_QZS_L2CX:
    case CODE_QZS_L5I:
    case CODE_QZS_L5Q:
    case CODE_QZS_L5X:
    default:
      log_error_sid(nav_meas->sid, "Unsupported code");
  }

  /* Penalize the measurements that do not have all tracking flags set */

  /* Lower code/doppler accuracy if phase not locked */
  if (0 == (nav_meas->flags & NAV_MEAS_FLAG_PHASE_VALID)) {
    pseudorange_var *= NO_PLL_MULTIPLIER;
    measured_doppler_var *= NO_PLL_MULTIPLIER;
    /* no carrier phase measurement if there is no lock */
    carrier_phase_var = 0;
  }
  /* Lower code/doppler accuracy if signal has just been (re)acquired */
  if (TRACK_TIME_THRESHOLD_S > nav_meas->time_in_track) {
    /* coef works out to MULTIPLIER when time_in_track == 0
     *               and 1.0        when time_in_track == THRESHOLD
     * and interpolates linearly in between */
    double coef = SHORT_TRACK_TIME_MULTIPLIER -
                  (SHORT_TRACK_TIME_MULTIPLIER - 1) * nav_meas->time_in_track /
                      TRACK_TIME_THRESHOLD_S;
    pseudorange_var *= coef;
    measured_doppler_var *= coef;
  }
  /* Lower phase measurement accuracy if PLL lock time is short */
  if (LOCK_TIME_THRESHOLD_S > nav_meas->lock_time) {
    /* coef works out to MULTIPLIER when lock_time == 0
     *               and 1.0        when lock_time == THRESHOLD
     * and interpolates linearly in between */
    double coef = SHORT_LOCK_TIME_MULTIPLIER -
                  (SHORT_LOCK_TIME_MULTIPLIER - 1) * nav_meas->lock_time /
                      LOCK_TIME_THRESHOLD_S;
    carrier_phase_var *= coef;
  }
  /* Lower phase accuracy if half-cycle not resolved  */
  if (0 == (nav_meas->flags & NAV_MEAS_FLAG_HALF_CYCLE_KNOWN)) {
    carrier_phase_var *= NO_HALF_CYCLE_MULTIPLIER;
  }

  if (0 != (nav_meas->flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID)) {
    assert(nav_meas->computed_doppler_dt > 0);
    /* Estimate the variance of the computed Doppler measurement, assuming
     * the previous carrier phase measurement had the same variance as the
     * current one, and that their noises are independent. */

    /* The total variance consists of the variances of the two carrier phase
     * measurements summed and divided by time difference squared, and the
     * noise term from clock variance. (the latter term is somewhat ad hoc,
     * but it seems to capture well what happens with longer dt)
     */
    double dt_squared =
        nav_meas->computed_doppler_dt * nav_meas->computed_doppler_dt;
    computed_doppler_var = 2 * carrier_phase_var / dt_squared +
                           dt_squared * NOMINAL_CLOCK_VARIANCE;
  }

  /* Assign to outputs (where given) */
  if (NULL != p_pseudorange_var) {
    *p_pseudorange_var = pseudorange_var;
  }
  if (NULL != p_measured_doppler_var) {
    *p_measured_doppler_var = measured_doppler_var;
  }
  if (NULL != p_carrier_phase_var) {
    *p_carrier_phase_var = carrier_phase_var;
  }
  if (NULL != p_computed_doppler_var) {
    *p_computed_doppler_var = computed_doppler_var;
  }
}

/** Compute the distance and line-of-sight vector between the satellite and
 * assumed receiver position, taking into account the rotation of Earth
 * during time-of-flight.
 *
 * \param rx_state  Assumed state vector
 * \param sat_pos   Satellite position vector
 * \param[out] los  Line-of-sight unit vector
 *
 * \return distance in meters
 */
static double compute_predicted_range(const double rx_state[3],
                                      const double sat_pos[3],
                                      double los[3]) {
  double tempv[3];
  double xk_new[3];

  /* Magnitude of range vector converted into an approximate time in secs. */
  vector_subtract(3, rx_state, sat_pos, tempv);
  double tau = vector_norm(3, tempv) / GPS_C;

  /* Rotation of Earth during time of flight in radians. */
  double wEtau = GPS_OMEGAE_DOT * tau;

  /* Apply linearised rotation about Z-axis which will adjust for the
   * satellite's position at time t-tau. Note the rotation is through
   * -wEtau because it is the ECEF frame that is rotating with the Earth and
   * hence in the ECEF frame free falling bodies appear to rotate in the
   * opposite direction.
   *
   * Making a small angle approximation here leads to less than 1mm error in
   * the satellite position. */
  xk_new[0] = sat_pos[0] + wEtau * sat_pos[1];
  xk_new[1] = sat_pos[1] - wEtau * sat_pos[0];
  xk_new[2] = sat_pos[2];

  /* Line of sight vector. */
  vector_subtract(3, xk_new, rx_state, los);

  /* Length of the line of sight is the predicted range */
  return vector_norm(3, los);
}

/** One step of PVT iteration
 *
 * Return
 *      1 for converged solution
 *      0 for successful but not yet converged step
 *     -1 for failure
 *
 * This function is the key to GPS solution, so it's commented
 * liberally.  It does a single step of a multi-dimensional
 * Newton-Raphson solution for the variables X, Y, Z (in ECEF) plus
 * the clock offset for each receiver used to make pseudorange
 * measurements.  The steps involved are roughly the following:
 *
 *     1. Account for the Earth's rotation during transmission
 *
 *     2. Estimate the ECEF position for each satellite measured using
 *     the downloaded ephemeris
 *
 *     3. Compute the Jacobian of pseudorange versus estimated state.
 *     There's no explicit differentiation; it's done symbolically
 *     first and just coded as a "line of sight" vector.
 *
 *     4. Use weighted linear least squares subroutine to solve a vector
 *     of corrections to our state estimate. We apply
 *     these to our current estimate and recurse to the next step.
 *
 *     5. If our corrections are very small, we've arrived at a good
 *     enough solution.  Solve for the receiver's velocity (with
 *     vel_solve) and do some bookkeeping to pass the solution back
 *     out.
 */
static int pvt_solve(double rx_state[],
                     const u8 n_used,
                     bool disable_velocity,
                     const navigation_measurement_t *nav_meas[n_used],
                     double omp[n_used],
                     double omp_doppler[n_used],
                     double H[4][4],
                     double V[4][4],
                     double V_vel[4][4]) {
  /* G is a geometry matrix tells us how our pseudoranges relate to
   * our state estimates -- it's the Jacobian of d(p_i)/d(x_j) where
   * x_j are x, y, z, Δt. */
  double G[n_used][4];

  /* diagonal elements of the weighting matrix */
  double w[n_used];

  double los[3];
  double tempd;
  double correction[4];

  for (u8 j = 0; j < 4; j++) {
    correction[j] = 0.0;
  }

  for (u8 j = 0; j < n_used; j++) {
    /* Predicted range from satellite position and estimated Rx position. */
    double p_pred =
        compute_predicted_range(rx_state, nav_meas[j]->sat_pos, los);

    /* omp means "observed minus predicted" range -- this is E, the
     * prediction error vector (or innovation vector in Kalman/LS
     * filtering terms).
     */
    omp[j] = nav_meas[j]->pseudorange - p_pred;

    double pseudorange_var = 0.0;
    calc_measurement_noises(nav_meas[j], &pseudorange_var, NULL, NULL, NULL);

    /* Construct the weight matrix. Ideally it would have the inverses of
     * individual measurement variances on the diagonal
     */
    if (0 != pseudorange_var) {
      w[j] = 1.0 / pseudorange_var;
    } else {
      w[j] = 1.0;
    }

    /* Construct a geometry matrix.  Each row (satellite) is
     * independently normalized into a unit vector. */
    for (u8 i = 0; i < 3; i++) {
      G[j][i] = -los[i] / p_pred;
    }

    /* Projection of clock bias into each pseudorange is 1. */
    G[j][3] = 1;

  } /* End of channel loop. */

  /* Solve for position corrections using batch least-squares.  When
   * all-at-once least-squares estimation for a nonlinear problem is
   * mixed with numerical iteration (not time-series recursion, but
   * iteration on a single set of measurements), it's basically
   * Newton's method.  There's a reasonably clear explanation of this
   * in Wikipedia's article on GPS.
   */

  /* Solve the state update and its covariance matrix */
  int ret = matrix_wlsq_solve(n_used,
                              4,
                              (double *)G,
                              (double *)omp,
                              (double *)w,
                              (double *)correction,
                              (double *)V);
  if (ret < 0) {
    log_warn("Under-determined system, n_used = %u", n_used);
    return -1;
  }

  /* Increment ecef estimate by the new corrections */
  for (u8 i = 0; i < 3; i++) {
    rx_state[i] += correction[i];
  }

  /* Set the Δt estimates according to this solution */
  for (u8 i = 3; i < 4; i++) {
    rx_state[i] = correction[i];
  }

  /* Look at the magnitude of the correction to see if
   * the solution has converged yet.
   */
  tempd = vector_norm(3, correction);
  if (tempd > 0.001) {
    /* The solution has not converged, return 0 to
     * indicate that we should continue iterating.
     */
    return 0;
  }

  /* The solution has converged! */

  if (disable_velocity) {
    /* No velocity solution */
    rx_state[4] = 0.0;
    rx_state[5] = 0.0;
    rx_state[6] = 0.0;
    rx_state[7] = 0.0;
  } else {
    /* Perform the velocity solution. */
    vel_solve(&rx_state[4],
              n_used,
              nav_meas,
              (const double(*)[4])G,
              V_vel,
              omp_doppler);
  }

  /* Prepare a separate un-weighted geometry matrix for DOP computations in H */
  /* H is the inverted square of the Jacobian matrix; it tells us the shape of
     our error (or, if you prefer, the direction in which we need to
     move to get a better solution) in terms of the receiver state. */
  if (matrix_wlsq_solve(
          n_used, 4, (double *)G, (double *)omp, NULL, NULL, (double *)H) < 0) {
    log_warn("Under-determined system computing DOP, n_used = %u", n_used);
    return -1;
  }

  /* success */
  return 1;
}

static s8 filter_solution(gnss_solution *soln, dops_t *dops) {
  if (dops->gdop > 20.0) {
    /* GDOP is too high to yield a good solution. */
    return PVT_PDOP_TOO_HIGH;
  }

  if (soln->pos_llh[2] < -1e3 || soln->pos_llh[2] > 1e6) {
    /* Altitude is unreasonable. */
    return PVT_BAD_ALTITUDE;
  }

  /* NOTE: The following condition is required to comply with US export
   * regulations. It must not be removed. Any modification to this condition
   * is strictly not approved by Swift Navigation Inc. */

  if (vector_norm(3, soln->vel_ecef) >= 0.514444444 * 1000) {
    /* Velocity is greater than 1000kts. */
    return PVT_VELOCITY_LOCKOUT;
  }

  return 0;
}

/** Checks pvt_iter weighted residuals.
 *
 * \param n_used   length of omp
 * \param disable_velocity
 * \param omp      residual vector calculated by pvt_solve
 * \param omp_doppler  Doppler residual vector calculated by pvt_solve
 * \param nav_meas array of navigation measurements
 * \param rx_state reference to pvt solver state allocated in calc_PVT
 * \param metric   If not null, used to output double value of RAIM metric
 *
 * \return true if metric < scaled RAIM metric threshold
 */
static bool residual_test(u8 n_used,
                          bool disable_velocity,
                          const double omp[n_used],
                          const double omp_doppler[n_used],
                          const navigation_measurement_t *nav_meas[n_used],
                          const double rx_state[],
                          double *p_metric) {
  if (rx_state[0] == 0.0 && rx_state[1] == 0.0 && rx_state[2] == 0.0) {
    /* State un-initialized */
    return false;
  }

  u8 n_meas;
  u8 raim_n_state;

  if (disable_velocity) {
    /* One measurement per signal */
    n_meas = n_used;
    raim_n_state = RAIM_N_STATE;
  } else {
    /* Compute residual from both pseudoranges and Dopplers */
    n_meas = 2 * n_used;
    raim_n_state = 2 * RAIM_N_STATE;
  }

  double residual[n_meas];
  double pr_var;
  double dop_var;

  /* Add clock offset to observed-minus-predicted calculated by last
   * iteration of pvt_solve, and normalize by measurement variance. */
  for (int i = 0; i < n_used; i++) {
    residual[i] = omp[i] - rx_state[3];
    if (!disable_velocity) {
      residual[n_used + i] = omp_doppler[i] - rx_state[7];
    }
    calc_measurement_noises(nav_meas[i], &pr_var, NULL, &dop_var, NULL);
    if (pr_var != 0) {
      residual[i] /= sqrt(pr_var);
    }
    if (!disable_velocity && (dop_var != 0)) {
      residual[n_used + i] /= sqrt(dop_var);
    }
  }
  double metric = vector_norm(n_meas, residual) / sqrt(n_meas - raim_n_state);
  if (p_metric) {
    *p_metric = metric;
  }
  double scaled_threshold =
      RAIM_METRIC_THRESHOLD * sqrt((double)n_meas / (n_meas - raim_n_state));

  return metric < scaled_threshold;
}

/** Iterates pvt_solve until it converges or PVT_MAX_ITERATIONS is reached.
 *
 * \return
 *   - `0`: solution converged
 *   - `-1`: solution failed to converge
 *
 *  Results stored in rx_state, omp, H, V, V_vel
 */
static s8 pvt_iter(double rx_state[],
                   const u8 n_used,
                   bool disable_velocity,
                   const navigation_measurement_t *nav_meas[n_used],
                   double omp[n_used],
                   double omp_doppler[n_used],
                   double H[4][4],
                   double V[4][4],
                   double V_vel[4][4]) {
  /* Reset state to zero */
  for (u8 i = 0; i < 8; i++) {
    rx_state[i] = 0;
  }

  u8 iters;
  int ret;
  /* Newton-Raphson iteration. */
  for (iters = 0; iters < PVT_MAX_ITERATIONS; iters++) {
    ret = pvt_solve(rx_state,
                    n_used,
                    disable_velocity,
                    nav_meas,
                    omp,
                    omp_doppler,
                    H,
                    V,
                    V_vel);
    /* break loop if solution converged or failed */
    if (ret != 0) {
      break;
    }
  }

  if (iters >= PVT_MAX_ITERATIONS || ret < 0) {
    return -1;
  }

  return 0;
}

/** Run pvt_iter with a subset of measurements and run a residual test for the
 * result.
 *
 * \return
 *   - true: solution converged and residual test passed
 *   - false: either solution failed to converge or residual rest failed
 *
 *  Results stored in rx_state, omp, H, V, metric
 */
static bool pvt_iter_masked(double rx_state[],
                            const u8 n_meas,
                            bool disable_velocity,
                            const navigation_measurement_t *nav_meas[n_meas],
                            const gnss_sid_set_t *removed_sids,
                            double H[4][4],
                            double V[4][4],
                            double V_vel[4][4],
                            double *metric) {
  const navigation_measurement_t *nav_meas_subset[n_meas];
  gnss_sid_set_t used_sids;
  sid_set_init(&used_sids);

  u8 n_used = 0;
  /* form pointers array to the remaining measurements */
  for (u8 i = 0; i < n_meas; i++) {
    if (!sid_set_contains(removed_sids, nav_meas[i]->sid)) {
      nav_meas_subset[n_used] = nav_meas[i];
      sid_set_add(&used_sids, nav_meas[i]->sid);
      n_used++;
    }
  }

  /* check that there are still enough satellites RAIM */
  if (sid_set_get_sat_count(&used_sids) < 4) {
    log_info("RAIM failed, not enough satellites remaining");
    return false;
  }

  double omp[n_used];
  double omp_doppler[n_used];
  if (pvt_iter(rx_state,
               n_used,
               disable_velocity,
               nav_meas_subset,
               omp,
               omp_doppler,
               H,
               V,
               V_vel) < 0) {
    /* solution failed */
    return false;
  }

  /* return the result of the residual test */
  return residual_test(n_used,
                       disable_velocity,
                       omp,
                       omp_doppler,
                       nav_meas_subset,
                       rx_state,
                       metric);
}

/* PVT solution with only GPS measurements */
static u8 pvt_solve_gps_only(const u8 n_meas,
                             const navigation_measurement_t *nav_meas[n_meas],
                             double rx_state[],
                             bool disable_velocity,
                             double H[4][4],
                             double V[4][4],
                             double V_vel[4][4],
                             double original_metric,
                             gnss_sid_set_t *removed_sids) {
  double new_metric = original_metric;

  sid_set_init(removed_sids);
  for (s8 i = 0; i < n_meas; i++) {
    if (!IS_GPS(nav_meas[i]->sid)) {
      sid_set_add(removed_sids, nav_meas[i]->sid);
    }
  }
  u8 n_used = n_meas - sid_set_get_sig_count(removed_sids);

  if (n_used <= RAIM_N_STATE) {
    /* not enough measurements for constellation RAIM */
    log_info(
        "RAIM failed: %d measurements not enough for constellation RAIM, "
        "metric %.1g",
        n_used,
        original_metric);
    return PVT_RAIM_REPAIR_IMPOSSIBLE;
  } else if (pvt_iter_masked(rx_state,
                             n_meas,
                             disable_velocity,
                             nav_meas,
                             removed_sids,
                             H,
                             V,
                             V_vel,
                             &new_metric)) {
    /* success */
    log_info("RAIM excluded all non-GPS measurements (%" PRIu32
             " out of %d), metric %.1g "
             "-> %.1f",
             sid_set_get_sig_count(removed_sids),
             n_meas,
             original_metric,
             new_metric);
    return PVT_CONVERGED_RAIM_REPAIR;
  } else {
    /* no solution found */
    if (sid_set_get_sig_count(removed_sids) > 0) {
      log_info("RAIM failed: tried excluding %" PRIu32
               " measurement(s) out of %d, "
               "metric %"
               ".1g -> %.1f",
               sid_set_get_sig_count(removed_sids),
               n_meas,
               original_metric,
               new_metric);
    } else {
      log_info(
          "RAIM failed: all exclusion candidates out of %d measurements "
          "failed, "
          "metric %.1g",
          n_meas,
          original_metric);
    }
    return PVT_RAIM_REPAIR_FAILED;
  }
}

/** See pvt_solve_raim() for parameter meanings.
 *
 * \return
 *   - `1`: repaired solution, using fewer observations
 *          returns sids of removed measurements if removed_sids ptr is passed
 *
 *   - `-1`: no reasonable solution possible
 */
static s8 pvt_repair(double rx_state[],
                     const u8 n_used,
                     bool disable_velocity,
                     const navigation_measurement_t *nav_meas[n_used],
                     const double omp[n_used],
                     const double omp_doppler[n_used],
                     double H[4][4],
                     double V[4][4],
                     double V_vel[4][4],
                     gnss_sid_set_t *removed_sids) {
  /* If removed_sids is null, point it to a local variable */
  gnss_sid_set_t local_removed_sids;
  if (!removed_sids) {
    removed_sids = &local_removed_sids;
  }
  sid_set_init(removed_sids);

  u8 n_removed = sid_set_get_sig_count(removed_sids);

  /* Compute the residuals and metric for the original set of measurements */

  double metric[n_used];
  s8 bad_sat = -1;

  double original_metric;
  residual_test(n_used,
                disable_velocity,
                omp,
                omp_doppler,
                nav_meas,
                rx_state,
                &original_metric);

  /* Each iteration of this loop excludes one signal and either returns with
   * a successful solution, or eventually returns a failure when no more
   * signals can be removed. */
  while (n_removed < RAIM_MAX_EXCLUSIONS &&
         (n_used - n_removed - 1) > RAIM_N_STATE) {
    bool successful_exclusion_found = false;
    double best_metric = INFINITY;
    double residual = 0.0;
    double vel_residual = 0.0;

    /* loop through the signals and remove each in turn */
    for (s8 i = 0; i < n_used; i++) {
      metric[i] = INFINITY;
      if (sid_set_contains(removed_sids, nav_meas[i]->sid)) {
        /* this signal is already removed */
        continue;
      }
      /* try removing this signal */
      sid_set_add(removed_sids, nav_meas[i]->sid);
      /* compute solution and perform residual test with a subset of signals */
      bool solution_flag = pvt_iter_masked(rx_state,
                                           n_used,
                                           disable_velocity,
                                           nav_meas,
                                           removed_sids,
                                           H,
                                           V,
                                           V_vel,
                                           &metric[i]);
      if (solution_flag) {
        /* at least one exclusion is successful */
        successful_exclusion_found = true;
        log_debug_sid(nav_meas[i]->sid,
                      "RAIM exclusion successful, metric %.2g",
                      metric[i]);
      } else {
        log_debug_sid(nav_meas[i]->sid,
                      "RAIM failed to exclude measurement, metric %.2g",
                      metric[i]);
      }
      /* Store the metric and satellite index if it is best so far on this
       * round.
       * Note that if no successful solution has been found yet, we store any
       * exclusion that improves the current metric, successful or not (this
       * will be used later when trying multiple exclusions).
       * If a valid exclusion is already found, then store only those exclusions
       * that improve the metric and also result in a valid solution.
       * */
      if ((metric[i] < best_metric) &&
          (!successful_exclusion_found || solution_flag)) {
        bad_sat = i;
        best_metric = metric[i];

        /* Compute the residual of the removed signal against the repaired
         * position for logging */
        double los[3];
        double p_pred =
            compute_predicted_range(rx_state, nav_meas[bad_sat]->sat_pos, los);

        residual = nav_meas[bad_sat]->pseudorange - p_pred - rx_state[3];

        if (!disable_velocity) {
          double pdot_pred =
              vector_dot(3, los, nav_meas[bad_sat]->sat_vel) / p_pred;

          vel_residual = -nav_meas[bad_sat]->measured_doppler *
                             sid_to_lambda(nav_meas[bad_sat]->sid) -
                         pdot_pred - rx_state[7];
        }
      }
      sid_set_remove(removed_sids, nav_meas[i]->sid);
    }

    if (bad_sat < 0) {
      /* None of the exclusion attempts returned a solution, break from the
       * loop and try the gps-only solution */
      log_debug("RAIM failed: all exclusion candidates failed");
      break;
    }

    /* keep the best found removal from this round */
    sid_set_add(removed_sids, nav_meas[bad_sat]->sid);
    if (disable_velocity) {
      log_info_sid(
          nav_meas[bad_sat]->sid, "RAIM exclusion, residual %.0f m", residual);
    } else {
      log_info_sid(nav_meas[bad_sat]->sid,
                   "RAIM exclusion, residuals %.0f m, %.0f m/s",
                   residual,
                   vel_residual);
    }
    if (successful_exclusion_found) {
      /* Successful exclusion found. Recalculate that solution. */
      bool flag = pvt_iter_masked(rx_state,
                                  n_used,
                                  disable_velocity,
                                  nav_meas,
                                  removed_sids,
                                  H,
                                  V,
                                  V_vel,
                                  &metric[bad_sat]);
      assert(flag == true);
      log_info("RAIM excluded %" PRIu32
               " measurement(s) out of %d, metric %.1g -> %.1f",
               sid_set_get_sig_count(removed_sids),
               n_used,
               original_metric,
               metric[bad_sat]);
      return PVT_CONVERGED_RAIM_REPAIR;
    }

    n_removed = sid_set_get_sig_count(removed_sids);

    log_debug_sid(
        nav_meas[bad_sat]->sid,
        "RAIM no single exclusion found looking for more, metric: %.2g",
        best_metric);
  }

  /* Loop exhausted, cannot remove any more measurements. As a last-ditch
   * effort, try removing all but GPS signals */
  return pvt_solve_gps_only(n_used,
                            nav_meas,
                            rx_state,
                            disable_velocity,
                            H,
                            V,
                            V_vel,
                            original_metric,
                            removed_sids);
}

/** Calculate pvt solution, perform RAIM check, attempt to repair if needed.
 *
 * See calc_PVT for parameter meanings.
 * \param rx_state reference to pvt solver state allocated in calc_PVT
 * \param n_used number of measurements
 * \param nav_meas array of measurements
 * \param disable_raim passing True will omit RAIM check/repair functionality
 * \param disable_velocity passing True will skip velocity solution
 * \param H see pvt_solve
 * \param V see pvt_solve
 * \param removed_sids if not null and repair occurs, returns dropped sids
 * \param metric if not null, return the value of the RAIM metric
 *
 * \return Non-negative values indicate success; see below
 *         For negative values, refer to pvt_err_msg().
 * Return values:
 *    `2`: solution ok, but raim check was not used
 *        (exactly 4 measurements, or explicitly disabled)
 *
 *    `1`: repaired solution, using fewer observations
 *        returns sid set of removed measurements if removed_sid ptr is passed
 *
 *    `0`: initial solution ok
 *
 *   - `-4`: repair failed
 *   - `-5`: not enough satellites to attempt repair
 *   - `-6`: pvt_iter didn't converge
 *
 *  Results stored in rx_state, H, V, V_vel, removed_sid, residual
 */
static s8 pvt_solve_raim(double rx_state[],
                         const u8 n_used,
                         const navigation_measurement_t nav_meas[n_used],
                         bool disable_raim,
                         bool disable_velocity,
                         double H[4][4],
                         double V[4][4],
                         double V_vel[4][4],
                         gnss_sid_set_t *removed_sids,
                         double *metric) {
  double omp[n_used];
  double omp_doppler[n_used];

  assert(n_used <= MAX_CHANNELS);

  const navigation_measurement_t *nav_meas_ptrs[n_used];
  for (s8 i = 0; i < n_used; i++) {
    nav_meas_ptrs[i] = &nav_meas[i];
  }

  s8 flag = pvt_iter(rx_state,
                     n_used,
                     disable_velocity,
                     nav_meas_ptrs,
                     omp,
                     omp_doppler,
                     H,
                     V,
                     V_vel);

  if (flag == -1) {
    /* Iteration didn't converge. Attempt to repair although giving up would
     * be easier on CPU. */
    /* return PVT_UNCONVERGED; */
  }

  bool solution_ok = (flag >= 0);

  if (solution_ok && (disable_raim || RAIM_N_STATE == n_used)) {
    /* RAIM is disabled, or there are too few measurements to test residuals */
    return PVT_CONVERGED_NO_RAIM;
  }

  bool residual_ok = residual_test(n_used,
                                   disable_velocity,
                                   omp,
                                   omp_doppler,
                                   nav_meas_ptrs,
                                   rx_state,
                                   metric);

  if (solution_ok && residual_ok) {
    return PVT_CONVERGED_RAIM_OK;
  }

  if (!solution_ok && n_used <= RAIM_N_STATE + 1) {
    /* Solution failed, but there are not enough measurements to repair it.
     * At least 2 more measurements than states are needed, so that one
     * measurement can be removed and there still remains an over-determined
     * measurement set for residual checking.
     */
    return PVT_RAIM_REPAIR_IMPOSSIBLE;
  }

  /* Otherwise, try RAIM repair */
  return pvt_repair(rx_state,
                    n_used,
                    disable_velocity,
                    nav_meas_ptrs,
                    omp,
                    omp_doppler,
                    H,
                    V,
                    V_vel,
                    removed_sids);
}

/** Error strings for calc_PVT() negative (failure) return codes.
 *  e.g. `pvt_err_msg[-ret - 1]`
 *    where `ret` is the return value of calc_PVT(). */
const char *pvt_err_msg[] = {
    "PDOP too high",
    "Altitude unreasonable",
    "Velocity >= 1000 kts",
    "RAIM repair attempted, failed",
    "RAIM repair impossible (not enough measurements)",
    "Took too long to converge",
    "Not enough measurements for solution (< 4)",
};

/** Try to calculate a single point gps solution
 *
 * Note: Observations must have SPP OK flag set, and a valid pseudorange.
 * A valid doppler value is required if disable_velocity is false.
 *
 * \param n_used number of measurements
 * \param nav_meas array of measurements of length `n_used`
 * \param disable_raim passing True will omit RAIM check/repair functionality
 * \param disable_velocity passing True will disable velocity output
 * \param soln output solution struct
 * \param dops output dilution of precision information
 * \param raim_removed_sids optional arg that returns the sids of excluded
 *        observations if RAIM successfully excluded a signal / signals
 * \return Non-negative values indicate a valid solution.
 *   -  `2`: Solution converged but RAIM unavailable or disabled
 *   -  `1`: Solution converged, failed RAIM but was successfully repaired
 *   -  `0`: Solution converged and verified by RAIM
 *   - `-1`: PDOP is too high to yield a good solution.
 *   - `-2`: Altitude is unreasonable.
 *   - `-3`: Velocity is greater than or equal to 1000 kts.
 *   - `-4`: RAIM check failed and repair was unsuccessful
 *   - `-5`: RAIM check failed and repair was impossible (not enough
 * measurements)
 *   - `-6`: pvt_iter didn't converge
 *   - `-7`: < 4 measurements
 */
s8 calc_PVT(const u8 n_used,
            const navigation_measurement_t nav_meas[],
            bool disable_raim,
            bool disable_velocity,
            gnss_solution *soln,
            dops_t *dops,
            gnss_sid_set_t *raim_removed_sids) {
  /* Initial state is the center of the Earth with zero velocity and zero
   * clock error
   *  rx_state format:
   *    pos[3], clock error, vel[3], intermediate freq error
   */
  double rx_state[8];

  /* unweighted geometry matrix for DOP computations */
  double H[4][4];
  /* solution covariance matrix [m^2] */
  double V[4][4];
  /* velocity covariance matrix [(m/s)^2] */
  double V_vel[4][4];

  for (u8 i = 0; i < n_used; i++) {
    if (!(nav_meas[i].flags & NAV_MEAS_FLAG_CODE_VALID)) {
      assert(
          !"SPP attempted on measurements that did not have valid pseudorange");
    }

    /* if velocity output is requested, every signal must have valid Doppler */
    if (!disable_velocity &&
        !(nav_meas[i].flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID) &&
        !(nav_meas[i].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID)) {
      assert(
          "SPP velocity requested but not all measurements have valid Doppler");
    }
  }

  if (n_used < 4) {
    return PVT_INSUFFICENT_MEAS;
  }

  soln->valid = 0;
  soln->n_sats_used = 0;
  soln->n_sigs_used = 0;

  gnss_sid_set_t removed_sids;
  sid_set_init(&removed_sids);
  s8 raim_flag = pvt_solve_raim(rx_state,
                                n_used,
                                nav_meas,
                                disable_raim,
                                disable_velocity,
                                H,
                                V,
                                V_vel,
                                &removed_sids,
                                NULL);

  if (raim_flag < 0) {
    /* Didn't converge or least squares integrity check failed. */
    return raim_flag;
  }

  /* Initial solution failed, but repair was successful. */
  /* pvt_solve_raim() return of 1, means signal(s) were removed due to RAIM
   * and removed_sids holds the signal that were removed. */
  if (raim_flag == PVT_CONVERGED_RAIM_REPAIR) {
    if (raim_removed_sids != NULL) {
      *raim_removed_sids = removed_sids;
    }
  }

  /* Count number of unique satellites in the solution */
  gnss_sid_set_t sid_set;
  sid_set_init(&sid_set);
  for (u8 j = 0; j < n_used; j++) {
    /* Skip the removed SIDs */
    /* pvt_solve_raim() return of 1, means signal(s) were removed due to RAIM
     * and removed_sids lists the signal that were removed. */
    if ((PVT_CONVERGED_RAIM_REPAIR == raim_flag) &&
        sid_set_contains(&removed_sids, nav_meas[j].sid)) {
      continue;
    }
    soln->n_sigs_used++;
    sid_set_add(&sid_set, nav_meas[j].sid);
  }
  soln->n_sats_used = sid_set_get_sat_count(&sid_set);

  /* Compute various dilution of precision metrics. */
  compute_dops((const double(*)[4])H, rx_state, dops);
  soln->err_cov[6] = dops->gdop;

  /* Populate error covariances according to layout in definition
   * of gnss_solution struct.
   */
  soln->err_cov[0] = V[0][0];
  soln->err_cov[1] = V[0][1];
  soln->err_cov[2] = V[0][2];
  soln->err_cov[3] = V[1][1];
  soln->err_cov[4] = V[1][2];
  soln->err_cov[5] = V[2][2];

  if (!disable_velocity) {
    /* Populate the velocity covariances similarly to err_cov  */
    soln->vel_cov[0] = V_vel[0][0];
    soln->vel_cov[1] = V_vel[0][1];
    soln->vel_cov[2] = V_vel[0][2];
    soln->vel_cov[3] = V_vel[1][1];
    soln->vel_cov[4] = V_vel[1][2];
    soln->vel_cov[5] = V_vel[2][2];
    /* Velocity is computed from same geometry as position, so
     * GDOP is also same. */
    soln->vel_cov[6] = dops->gdop;
  }

  /* Save as x, y, z. */
  for (u8 i = 0; i < 3; i++) {
    soln->pos_ecef[i] = rx_state[i];
    soln->vel_ecef[i] = rx_state[4 + i];
  }

  wgsecef2ned(soln->vel_ecef, soln->pos_ecef, soln->vel_ned);

  /* Convert to lat, lon, hgt. */
  wgsecef2llh(rx_state, soln->pos_llh);

  soln->clock_offset = rx_state[3] / GPS_C;
  soln->clock_drift = rx_state[7] / GPS_C;
  soln->clock_offset_var = V[3][3] / GPS_C / GPS_C;
  soln->clock_drift_var = V_vel[3][3] / GPS_C / GPS_C;

  /* Time at receiver is TOT plus time of flight. Time of flight is equal to
   * the raw pseudorange minus the clock offset. */

  /* Produce time estimate from the first signal not excluded by RAIM */
  u8 j = 0;
  while ((PVT_CONVERGED_RAIM_REPAIR == raim_flag) &&
         sid_set_contains(&removed_sids, nav_meas[j].sid)) {
    j++;
  }
  soln->time = nav_meas[j].tot;
  soln->time.tow += nav_meas[j].raw_pseudorange / GPS_C;
  /* Correct for satellite and receiver clock offsets. */
  soln->time.tow += nav_meas[j].sat_clock_err;
  soln->time.tow -= rx_state[3] / GPS_C;
  normalize_gps_time(&soln->time);

  u8 ret = filter_solution(soln, dops);
  if (0 != ret) {
    memset(soln, 0, sizeof(*soln));
    return ret;
  }

  soln->valid = 1;

  if (!disable_velocity) {
    soln->velocity_valid = 1;
  } else {
    soln->velocity_valid = 0;
    rx_state[4] = 0.0;
    rx_state[5] = 0.0;
    rx_state[6] = 0.0;
    rx_state[7] = 0.0;
  }

  return raim_flag;
}

/** Calculates and applies Iono and Tropo correction for pseudorange,
 * carrier phase and Doppler measurements
 *
 * Note: this function does not check pseudorange or carrier phase validty
 * flags as a correction to an invalid value will just result in another invalid
 * value.
 *
 * \param n_ready_tdcp The number of measurements provided in nav_meas_tdcp
 * \param nav_meas_tdcp Pointer to measurements array
 * \param pos_ecef position in WGS84 Earth Centered, Earth Fixed (ECEF)
 * Cartesian
 * \param pos_llh position in WGS84 geodetic coordinates
 * \param iono_params pointer to ionosphere parameters received from L1CA stream
 *                    or null if not available */
void calc_iono_tropo(u8 n_ready_tdcp,
                     navigation_measurement_t *nav_meas_tdcp,
                     const double *pos_ecef,
                     const double *pos_llh,
                     const ionosphere_t *iono_params) {
  for (u8 i = 0; i < n_ready_tdcp; i++) {
    double az, el, cp_correction;
    double tropo_correction = 0;
    double iono_correction = 0;
    double sat_pos0[3];
    double iono_correction_delta = 0;
    double tropo_correction_delta = 0;
    double az0, el0;
    double h = 1.0; /* length of the finite difference, seconds */
    double doppler = 0.0;

    /* use doppler to increase correction accuracy if available,
     * otherwise just use the carrier frequency */
    if (0 != (nav_meas_tdcp[i].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID)) {
      doppler = nav_meas_tdcp[i].computed_doppler;
    } else if (0 !=
               (nav_meas_tdcp[i].flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
      doppler = nav_meas_tdcp[i].measured_doppler;
    }

    /* this signal's frequency */
    double carrier_freq = sid_to_carr_freq(nav_meas_tdcp[i].sid) + doppler;

    /* calculate azimuth and elevation of SV */
    wgsecef2azel(nav_meas_tdcp[i].sat_pos, pos_ecef, &az, &el);
    /* calculate azimuth and elevation h seconds ago */
    for (u8 j = 0; j < 3; j++) {
      sat_pos0[j] =
          nav_meas_tdcp[i].sat_pos[j] - h * nav_meas_tdcp[i].sat_vel[j];
    }
    wgsecef2azel(sat_pos0, pos_ecef, &az0, &el0);

    /* calc iono correction if available */
    if (iono_params) {
      /* calculate iono correction */
      iono_correction = calc_ionosphere(
          &nav_meas_tdcp[i].tot, pos_llh[0], pos_llh[1], az, el, iono_params);

      /* finite differences estimate of iono correction time derivative */
      iono_correction_delta =
          (iono_correction - calc_ionosphere(&nav_meas_tdcp[i].tot,
                                             pos_llh[0],
                                             pos_llh[1],
                                             az0,
                                             el0,
                                             iono_params)) /
          h;

      if (nav_meas_tdcp[i].sid.code != CODE_GPS_L1CA) {
        /* convert from L1CA Klobuchar correction */
        iono_correction *= pow(GPS_L1_HZ / carrier_freq, 2);
        iono_correction_delta *= pow(GPS_L1_HZ / carrier_freq, 2);
      }

      /* correct pseudorange */
      nav_meas_tdcp[i].pseudorange -= iono_correction;
      /* correct carrier phase based on iono correction */
      cp_correction = iono_correction * carrier_freq / GPS_C;
      nav_meas_tdcp[i].carrier_phase -= cp_correction;
      /* correct Doppler (sign opposite to pseudorange correction) */
      nav_meas_tdcp[i].measured_doppler +=
          iono_correction_delta * carrier_freq / GPS_C;
      nav_meas_tdcp[i].computed_doppler +=
          iono_correction_delta * carrier_freq / GPS_C;
    }

    /* calculate tropo correction.
     * ellipsoidal height is used due to lack of a geoid model */
    tropo_correction =
        calc_troposphere(&nav_meas_tdcp[i].tot, pos_llh[0], pos_llh[2], el);
    /* finite differences estimate of tropo correction time derivative */
    tropo_correction_delta =
        (tropo_correction -
         calc_troposphere(&nav_meas_tdcp[i].tot, pos_llh[0], pos_llh[2], el0)) /
        h;
    /* correct pseudorange */
    nav_meas_tdcp[i].pseudorange -= tropo_correction;
    /* correct carrier phase based on tropo correction */
    cp_correction = tropo_correction * carrier_freq / GPS_C;
    /* sign here is opposite to normal due to
     * Piksi's unusual sign convention on carrier phase */
    nav_meas_tdcp[i].carrier_phase += cp_correction;
    /* correct Doppler (sign opposite to pseudorange correction) */
    nav_meas_tdcp[i].measured_doppler +=
        tropo_correction_delta * carrier_freq / GPS_C;
    nav_meas_tdcp[i].computed_doppler +=
        tropo_correction_delta * carrier_freq / GPS_C;
    log_debug("%u: I %10.5f, T %10.5f", i, iono_correction, tropo_correction);
  }
}
