/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "clock_filter.h"

#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>

/** Process noise parameters for clock state Kalman filter.
 *  Determined empirically from the raw PVT solutions of several boards on HITL
 */
#define CLOCK_BIAS_VAR 5.0e-20
#define CLOCK_DRIFT_VAR 6.6e-22

/* Time in which clock state uncertainty does not change significantly */
#define CLOCK_STATE_MIN_PROPAGATE_TIME_S 1e-3

/** Propagate clock state to current epoch
 *
 * Update the clock_state in place to represent the propagated clock state
 * estimate at tick `tc`.
 *
 * \param clock_state Pointer to clock state estimate to be propagated
 * \param tc NAP tick counter of the current epoch
 */
void propagate_clock_state(clock_est_state_t *clock_state, u64 tc) {
  u64 ref_tc = clock_state->tc;

  if (tc < ref_tc) {
    /* do not propagate backwards */
    return;
  }

  double x[2];
  double P[2][2];

  x[0] = clock_state->t_gps.tow;
  x[1] = clock_state->clock_rate;
  matrix_copy(2, 2, (const double *)clock_state->P, (double *)P);

  double dt = (double)(tc - ref_tc) * clock_state->tick_length_s;

  /* x := F*x */
  /* where state transfer matrix F := [1 dt; 0 1] */
  x[0] = x[0] + dt * x[1];

  if (dt > CLOCK_STATE_MIN_PROPAGATE_TIME_S) {
    /* save cycles and do not propagate covariances over insignificant time
     * deltas */

    /* process noise covariance matrix */
    double Q[2][2];
    Q[0][0] = dt * CLOCK_BIAS_VAR + dt * dt * dt * CLOCK_DRIFT_VAR / 3;
    Q[0][1] = dt * dt * CLOCK_DRIFT_VAR / 2;
    Q[1][0] = Q[0][1];
    Q[1][1] = dt * CLOCK_DRIFT_VAR;

    /* P: = F*P*F' + Q */
    P[0][0] = P[0][0] + 2 * dt * P[0][1] + dt * dt * P[1][1] + Q[0][0];
    P[0][1] = P[0][1] + dt * P[1][1] + Q[0][1];
    P[1][0] = P[0][1];
    P[1][1] = P[1][1] + Q[1][1];
  }

  /* Write propagated values back to the state struct */
  clock_state->t_gps.tow = x[0];
  clock_state->clock_rate = x[1];
  matrix_copy(2, 2, (const double *)P, (double *)clock_state->P);
  clock_state->tc = tc;
}

/** Kalman filter update step.
 *
 * Update the clock_state in place with the current solved time estimate.
 *
 * \param clock_state Pointer to clock state estimate to be updated (needs
 *                    to be already propagated to current epoch)
 * \param sol Pointer to the PVT solution for the current epoch
 */
void update_clock_state(clock_est_state_t *clock_state,
                        const gnss_solution *sol) {
  /* a priori state estimate */
  double xm[2];
  /* a priori state covariance matrix */
  double Pm[2][2];

  xm[0] = clock_state->t_gps.tow;
  xm[1] = clock_state->clock_rate;
  matrix_copy(2, 2, (const double *)clock_state->P, (double *)Pm);

  /* measurement covariance matrix */
  double R[2][2];
  R[0][0] = sol->clock_offset_var;
  R[0][1] = 0;
  R[1][0] = 0;
  R[1][1] = sol->clock_drift_var;

  /* Innovation */
  double inno[2];
  inno[0] = sol->time.tow - xm[0];
  inno[1] = sol->clock_drift - (1 - xm[1]);

  /* protect against week roll-over */
  if (inno[0] > WEEK_SECS / 2) {
    inno[0] -= WEEK_SECS;
  } else if (inno[0] <= -WEEK_SECS / 2) {
    inno[0] += WEEK_SECS;
  }

  /* measurement matrix (note it is symmetric) */
  double H[2][2];
  H[0][0] = 1;
  H[0][1] = 0;
  H[1][0] = 0;
  H[1][1] = -1;

  /* innovation covariance matrix */
  double S[2][2];

  /* S := R + H*Pm*H' where H=[1 0; 0 -1] */
  S[0][0] = R[0][0] + Pm[0][0];
  S[1][0] = R[1][0] - Pm[1][0];
  S[0][1] = S[1][0];
  S[1][1] = R[1][1] + Pm[1][1];

  double invS[2][2];
  matrix_inverse(2, (double *)S, (double *)invS);

  /* Kalman gain */
  double K[2][2];
  /* K = Pm*H'*inv(S) */
  double tmp[2][2];
  matrix_multiply(2, 2, 2, (double *)Pm, (double *)H, (double *)tmp);
  matrix_multiply(2, 2, 2, (double *)tmp, (double *)invS, (double *)K);

  /* posteriori state */
  double x[2];
  /* state correction */
  double dx[2];

  matrix_multiply(2, 2, 1, (double *)K, inno, dx);
  vector_add(2, xm, dx, x);

  double eye[2][2];
  matrix_eye(2, (double *)eye);

  /* posteriori covariance */
  double P[2][2];

  /* P = (I - K*H)*Pm */
  matrix_multiply(2, 2, 2, (double *)K, (double *)H, (double *)tmp);
  matrix_add_sc(2, 2, (double *)eye, (double *)tmp, -1, (double *)tmp);
  matrix_multiply(2, 2, 2, (double *)tmp, (double *)Pm, (double *)P);

  /* ensure symmetry */
  double cross_term = (P[0][1] + P[1][0]) / 2;
  P[0][1] = cross_term;
  P[1][0] = cross_term;

  /* Write updated values back to the state struct */
  clock_state->t_gps.tow = x[0];
  clock_state->clock_rate = x[1];
  matrix_copy(2, 2, (const double *)P, (double *)clock_state->P);
}
