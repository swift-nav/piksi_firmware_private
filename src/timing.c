/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <math.h>
#include <string.h>

#include <libsbp/sbp.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "main.h"
#include "ndb/ndb_utc.h"
#include "sbp.h"
#include "timing.h"

/** \defgroup timing Timing
 * Maintains the time state of the receiver and provides time related
 * functions. The timing module tries to establish a relationship between GPS
 * time and local receiver time, i.e. the timer in the SwiftNAP.
 * \{ */

/** Clock state */
static volatile clock_est_state_t clock_state;

/** Mutex for guarding clock state access */
static MUTEX_DECL(clock_mutex);

/** Process noise parameters for clock state Kalman filter */
#define CLOCK_BIAS_VAR 1e-19
#define CLOCK_DRIFT_VAR 1e-23

/* default value for clock drift when not solved */
#define NOMINAL_CLOCK_DRIFT -1e-6;
#define NOMINAL_CLOCK_DRIFT_VAR 1e-13;

/* The thresholds for time qualities */
#define TIME_COARSE_THRESHOLD_S 10e-3
#define TIME_PROPAGATED_THRESHOLD_S 1e-6
#define TIME_FINE_THRESHOLD_S 100e-9
#define TIME_FINEST_THRESHOLD_S 10e-9

/* Time in which clock state uncertainty does not change significantly */
#define CLOCK_STATE_MIN_PROPAGATE_TIME_S 1e-3

/* Fetch clock state and propagate it to time tc */
static bool propagate_clock_state(u64 tc, double x[2], double P[2][2]) {
  chMtxLock(&clock_mutex);
  u64 ref_tc = clock_state.tc;
  x[0] = clock_state.t_gps.tow;
  x[1] = clock_state.clock_rate;
  matrix_copy(2, 2, (const double *)clock_state.P, (double *)P);
  chMtxUnlock(&clock_mutex);

  if (tc == 0) {
    return false;
  }

  double dt = (tc - ref_tc) * RX_DT_NOMINAL;

  if (dt < CLOCK_STATE_MIN_PROPAGATE_TIME_S) {
    /* save cycles and do not propagate over insignificant time deltas */
    return true;
  }

  /* process noise covariance matrix */
  double Q[2][2];
  Q[0][0] = dt * CLOCK_BIAS_VAR + dt * dt * dt * CLOCK_DRIFT_VAR / 3;
  Q[0][1] = dt * dt * CLOCK_DRIFT_VAR / 2;
  Q[1][0] = Q[0][1];
  Q[1][1] = dt * CLOCK_DRIFT_VAR;

  /* x := F*x */
  /* where state transfer matrix F := [1 dt; 0 1] */
  x[0] = x[0] + dt * x[1];

  /* P: = F*P*F' + Q */
  P[0][0] = P[0][0] + 2 * dt * P[0][1] + dt * dt * P[1][1] + Q[0][0];
  P[0][1] = P[0][1] + dt * P[1][1] + Q[0][1];
  P[1][0] = P[0][1];
  P[1][1] = P[1][1] + Q[1][1];

  return true;
}

/* Convert clock variance into a time quality level */
static time_quality_t clock_var_to_time_quality(double clock_var) {
  /* 3 sigma confidence limit for the clock error */
  double clock_confidence = 3 * sqrt(clock_var);

  if (clock_confidence < TIME_FINEST_THRESHOLD_S) return TIME_FINEST;
  if (clock_confidence < TIME_FINE_THRESHOLD_S) return TIME_FINE;
  if (clock_confidence < TIME_PROPAGATED_THRESHOLD_S) return TIME_PROPAGATED;
  if (clock_confidence < TIME_COARSE_THRESHOLD_S) return TIME_COARSE;

  return TIME_UNKNOWN;
}

/** Update GPS time estimate.
 *
 * This function uses the PVT solution to update the model relating receiver
 * internal clock with GPS time.
 *
 */
void update_time(u64 tc, const gnss_solution *sol) {
  if (!sol->valid) {
    log_warn("Tried to adjust clock with invalid solution");
    return;
  }

  chMtxLock(&clock_mutex);
  if (clock_state.tc == 0) {
    /* Initialize clock state estimate */
    clock_state.tc = tc;
    clock_state.t_gps = sol->time;
    gps_time_t t0 = sol->time;
    t0.tow -= RX_DT_NOMINAL * tc;
    normalize_gps_time(&t0);
    clock_state.t0_gps = t0;
    clock_state.clock_rate = 1 - sol->clock_drift;
    clock_state.P[0][0] = sol->clock_offset_var;
    clock_state.P[0][1] = 0.0;
    clock_state.P[1][0] = 0.0;
    clock_state.P[1][1] = sol->clock_drift_var;
    chMtxUnlock(&clock_mutex);

    time_quality_t time_quality =
        clock_var_to_time_quality(clock_state.P[0][0]);
    time_t unix_t = gps2time(&sol->time);
    log_info("(quality=%d) Time set to: %s", time_quality, ctime(&unix_t));

    return;
  }

  time_quality_t old_quality = clock_var_to_time_quality(clock_state.P[0][0]);
  chMtxUnlock(&clock_mutex);

  /* a priori state estimate */
  double xm[2];
  /* a priori state covariance matrix */
  double Pm[2][2];

  /* Get the a priori state estimate xm and covariance Pm at time tc */
  propagate_clock_state(tc, xm, Pm);

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

  /* match week number to the solution time */
  gps_time_t t = {.wn = WN_UNKNOWN, .tow = x[0]};
  normalize_gps_time(&t);
  gps_time_match_weeks(&t, &sol->time);

  /* write the values back into clock_state */
  chMtxLock(&clock_mutex);
  clock_state.t_gps = t;
  clock_state.clock_rate = x[1];
  clock_state.tc = tc;
  matrix_copy(2, 2, (double *)P, (double *)clock_state.P);
  chMtxUnlock(&clock_mutex);

  time_quality_t new_quality = clock_var_to_time_quality(P[0][0]);

  if (new_quality != old_quality) {
    log_info("Time quality changed: %d -> %d", old_quality, new_quality);
  }
}

/** Set/initialize clock model with coarse time from ephemeris or peer. If
 * internal clock model is already better then it is not updated.
 *
 * \param tc SwiftNAP timing count.
 * \param t Pointer to GPS time estimate.
 * \param accuracy Accuracy of the estimate (standard deviation)
 */
void set_time(u64 tc, const gps_time_t *t, double accuracy) {
  /* form a mock solution and reuse the update function */
  gnss_solution sol;

  sol.valid = 1;
  sol.time = *t;
  sol.clock_offset_var = accuracy * accuracy;

  sol.clock_drift = NOMINAL_CLOCK_DRIFT;
  sol.clock_drift_var = NOMINAL_CLOCK_DRIFT_VAR;

  update_time(tc, &sol);
}

/** Retrieve GPS time estimate quality.
 *
 */
time_quality_t get_time_quality(void) {
  /* Get the clock state and its variance estimate propagated at current NAP
   * tick */
  double state[2];
  double clock_var[2][2];
  if (!propagate_clock_state(nap_timing_count(), state, clock_var)) {
    return TIME_UNKNOWN;
  }

  return clock_var_to_time_quality(clock_var[0][0]);
}

void clock_est_init(clock_est_state_t *s) {
  chMtxLock(&clock_mutex);
  s->t_gps = GPS_TIME_UNKNOWN;
  s->clock_rate = 1.0;
  s->tc = 0;
  s->P[0][0] = 500e-3;
  s->P[0][1] = 0;
  s->P[1][0] = 0;
  s->P[1][1] = (double)RX_DT_NOMINAL * RX_DT_NOMINAL / 1e12; /* 1ppm. */
  chMtxUnlock(&clock_mutex);
}

/** Fine adjust the receiver time offset (to keep the current receiver time
 * close to GPS time)
 *
 * \param dt clock adjustment (s)
 */
void adjust_rcvtime_offset(const double dt) {
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = clock_state.t0_gps;
  double clock_rate = clock_state.clock_rate;
  gps_time.tow -= dt / clock_rate;
  normalize_gps_time(&gps_time);
  clock_state.t0_gps = gps_time;
  chMtxUnlock(&clock_mutex);
}

/** Get current RCV time.
 *
 * \note The RCV time may only be a guess or completely unknown. time_quality
 *       should be checked first to determine the quality of the RCV time
 *       estimate.
 *
 * This function should be used only for approximate timing purposes as simply
 * calling this function does not give a well defined instant at which the RCV
 * time is queried.
 *
 * \return Current GPS time.
 */
gps_time_t get_current_time(void) {
  /* TODO: Return invalid when TIME_UNKNOWN. */
  /* TODO: Think about what happens when nap_timing_count overflows. */
  u64 tc = nap_timing_count();
  gps_time_t t = napcount2gpstime(tc);

  return t;
}

/** Get current GPS time.
 *
 * \note The GPS time may only be a guess or completely unknown. time_quality
 *       should be checked first to determine the quality of the GPS time
 *       estimate.
 *
 * \return Current GPS time, or {WN_UNKNOWN, TOW_UNKNOWN} if no fix
 */
gps_time_t get_current_gps_time(void) {
  /* TODO: Think about what happens when nap_timing_count overflows. */
  u64 tc = nap_timing_count();
  gps_time_t t = napcount2gpstime(tc);

  return t;
}

/** Convert receiver time to GPS time.
 *
 * \note The GPS time may only be a guess or completely unknown. time_quality
 *       should be checked first to determine the quality of the GPS time
 *       estimate.
 *
 * \param tc Timing count in units of RX_DT_NOMINAL.
 * \return GPS time corresponding to Timing count.
 */
gps_time_t napcount2gpstime(const double tc) {
  chMtxLock(&clock_mutex);
  gps_time_t t = clock_state.t_gps;
  double rate = clock_state.clock_rate;
  u64 ref_tc = clock_state.tc;
  chMtxUnlock(&clock_mutex);

  if (!gps_time_valid(&t)) {
    return GPS_TIME_UNKNOWN;
  }

  t.tow += (tc - ref_tc) * RX_DT_NOMINAL * rate;
  normalize_gps_time(&t);

  return t;
}

/** Convert receiver time to receiver time in GPS time frame.
 *
 * \note The GPS time may only be a guess or completely unknown. Rcv time
 *  should be continuous with ms jumps
 *
 * \param tc Timing count in units of RX_DT_NOMINAL.
 * \return Rcv time in GPS time frame corresponding to Timing count.
 */
gps_time_t napcount2rcvtime(const double tc) {
  chMtxLock(&clock_mutex);
  gps_time_t t = clock_state.t0_gps;
  chMtxUnlock(&clock_mutex);

  if (!gps_time_valid(&t)) {
    return GPS_TIME_UNKNOWN;
  }

  t.tow += tc * RX_DT_NOMINAL;
  normalize_gps_time(&t);
  return t;
}

/** Convert GPS time to receiver time.
 *
 * \note The GPS time may only be a guess or completely unknown. time_quality
 *       should be checked first to determine the quality of the GPS time
 *       estimate.
 *
 * \param t gps_time_t to convert.
 * \return Timing count in units of RX_DT_NOMINAL.
 */
double gpstime2napcount(const gps_time_t *t) {
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = clock_state.t_gps;
  double rate = clock_state.clock_rate;
  double ref_tc = (double)clock_state.tc;
  chMtxUnlock(&clock_mutex);

  return ref_tc + gpsdifftime(t, &gps_time) / (RX_DT_NOMINAL * rate);
}

/** Convert Rcv time to rx time.
 *
 * \note The RCV time may only be a guess or completely unknown.
 *
 * \param t gps_time_t to convert.
 * \return Timing count in units of RX_DT_NOMINAL.
 */
double rcvtime2napcount(const gps_time_t *t) {
  chMtxLock(&clock_mutex);
  gps_time_t ref_time = clock_state.t0_gps;
  chMtxUnlock(&clock_mutex);

  return gpsdifftime(t, &ref_time) / RX_DT_NOMINAL;
}

/** Callback to set receiver GPS time estimate. */
static void set_time_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)sender_id;
  (void)len;
  (void)context;

  gps_time_t *t = (gps_time_t *)msg;

  log_info("Coarse time received from peer");

  /* this could come even over network, so set accuracy to 1s to be on the safe
   * side */
  set_time(nap_timing_count(), t, 1.0);
}

/** Setup timing functionality.
 * For now just register a callback so that a coarse time can be sent by the
 * host. */
void timing_setup(void) {
  /* TODO: Perhaps setup something to check for nap_timing_count overflows
   * periodically. */
  static sbp_msg_callbacks_node_t set_time_node;

  sbp_register_cbk(SBP_MSG_SET_TIME, &set_time_callback, &set_time_node);

  clock_est_init((clock_est_state_t *)&clock_state);
}

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
u64 timing_getms(void) {
  return (u64)(nap_timing_count() * (RX_DT_NOMINAL * 1000.0));
}

/** A convenience wrapper for glo2gps() API. Adds UTC params reading from NDB.
 * \param mesid ME sid for debugging info
 * \param glo_t GLO time
 * \return The resulting GPS time
 */
gps_time_t glo2gps_with_utc_params(me_gnss_signal_t mesid,
                                   const glo_time_t *glo_t) {
  gps_time_t gps_time = GPS_TIME_UNKNOWN;
  utc_params_t utc_params;
  ndb_op_code_t ndb_op_code;

  ndb_op_code = ndb_utc_params_read(&utc_params, /* is_nv = */ NULL);

  if (NDB_ERR_NONE == ndb_op_code) {
    gps_time = glo2gps(glo_t, &utc_params);
  } else {
    log_debug_mesid(mesid,
                    "GLO->GPS time conversion w/o up-to-date UTC params");
  }
  return gps_time;
}

/** Given a gps time, return the gps time of the nearest solution epoch
 * \param gps_time_t *time
 * \param double soln_freq
 * \return The GPS time of nearest epoch
 */

gps_time_t gps_time_round_to_epoch(const gps_time_t *time, double soln_freq) {
  gps_time_t rounded_time = GPS_TIME_UNKNOWN;
  /* round the time-of-week */
  rounded_time.tow = round(time->tow * soln_freq) / soln_freq;
  /* handle case where rounding caused tow roll-over */
  normalize_gps_time(&rounded_time);
  /* pick the correct week number */
  gps_time_match_weeks(&rounded_time, time);
  return rounded_time;
}

double get_clock_drift() {
  /* fetch clock state info */
  chMtxLock(&clock_mutex);
  double clock_rate = clock_state.clock_rate;
  chMtxUnlock(&clock_mutex);

  return 1.0 - clock_rate;
}

/** \} */
