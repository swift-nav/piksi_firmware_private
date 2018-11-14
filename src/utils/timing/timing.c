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
#include <swiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "clock_filter/clock_filter.h"
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
static volatile clock_est_state_t persistent_clock_state = {
    .tc = 0,
    .t_gps = GPS_TIME_UNKNOWN,
    .t0_gps = GPS_TIME_UNKNOWN,
    .tick_length_s = 0,
    .clock_rate = 0,
    .P = {{0, 0}, {0, 0}},
};
static volatile time_quality_t current_time_quality = TIME_UNKNOWN;

/** Mutex for guarding clock state access */
static MUTEX_DECL(clock_mutex);

static const char *time_quality_names[] = {
    "Unknown", "Coarse", "Propagated", "Fine", "Finest",
};

/* The thresholds for time qualities */
#define TIME_COARSE_THRESHOLD_S 10e-3
#define TIME_PROPAGATED_THRESHOLD_S 1e-6
#define TIME_FINE_THRESHOLD_S 100e-9
#define TIME_FINEST_THRESHOLD_S 10e-9

/* default value for clock drift when not solved */
#define NOMINAL_CLOCK_DRIFT -1e-6;
#define NOMINAL_CLOCK_DRIFT_VAR 1e-13;

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

/* log the changes of time quality */
static void log_time_quality(time_quality_t new_quality) {
  chMtxLock(&clock_mutex);
  time_quality_t old_quality = current_time_quality;
  current_time_quality = new_quality;
  chMtxUnlock(&clock_mutex);

  if (new_quality != old_quality) {
    log_info("Quality of time solution changed from %s to %s",
             time_quality_names[old_quality],
             time_quality_names[new_quality]);
  }
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
  clock_est_state_t clock_state = persistent_clock_state;
  chMtxUnlock(&clock_mutex);

  /* Is the clock state initialized? */
  if (!gps_time_valid(&clock_state.t_gps)) {
    /* Initialize clock state estimate with the given solution */
    clock_state.tc = tc;
    clock_state.t_gps = sol->time;
    gps_time_t t0 = sol->time;
    t0.tow -= RX_DT_NOMINAL * tc;
    normalize_gps_time(&t0);
    clock_state.t0_gps = t0;
    clock_state.clock_rate = 1 - sol->clock_drift;
    clock_state.tick_length_s = RX_DT_NOMINAL;
    clock_state.P[0][0] = sol->clock_offset_var;
    clock_state.P[0][1] = 0.0;
    clock_state.P[1][0] = 0.0;
    clock_state.P[1][1] = sol->clock_drift_var;

    time_quality_t time_quality =
        clock_var_to_time_quality(clock_state.P[0][0]);
    time_t unix_time = gps2time(&sol->time);
    log_info("Time set to: %s", ctime(&unix_time));
    log_time_quality(time_quality);

    chMtxLock(&clock_mutex);
    persistent_clock_state = clock_state;
    chMtxUnlock(&clock_mutex);

    return;
  }

  if (gpsdifftime(&sol->time, &clock_state.t_gps) < 0) {
    log_warn("Tried to update time with an old solution");
    return;
  }

  /* propagate the previous clock state estimate to current epoch tc */
  propagate_clock_state(&clock_state, tc);

  /* update the clock state with the solved time for this epoch */
  update_clock_state(&clock_state, sol);

  /* match week number to the solution time */
  unsafe_normalize_gps_time(&clock_state.t_gps);
  gps_time_match_weeks(&clock_state.t_gps, &sol->time);
  assert(gps_time_valid(&clock_state.t_gps));

  /* write the values back into the persistent clock_state */
  chMtxLock(&clock_mutex);
  persistent_clock_state = clock_state;
  chMtxUnlock(&clock_mutex);

  /* finally log the updated time quality */
  time_quality_t new_quality = clock_var_to_time_quality(clock_state.P[0][0]);
  log_time_quality(new_quality);
}

/** Set/initialize clock model with coarse time from ephemeris or peer. If
 * internal clock model is already better then it is not updated.
 *
 * \param tc SwiftNAP timing count.
 * \param t Pointer to GPS time estimate.
 * \param accuracy Accuracy of the estimate (standard deviation)
 */
void set_time(u64 tc, const gps_time_t *t, double accuracy) {
  if (!gps_time_valid(t)) {
    log_warn("Invalid gps time in set_time");
    return;
  }

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
  chMtxLock(&clock_mutex);
  clock_est_state_t clock_state = persistent_clock_state;
  chMtxUnlock(&clock_mutex);

  if (!gps_time_valid(&clock_state.t_gps)) {
    return TIME_UNKNOWN;
  }

  propagate_clock_state(&clock_state, nap_timing_count());

  time_quality_t new_quality = clock_var_to_time_quality(clock_state.P[0][0]);
  log_time_quality(new_quality);

  return new_quality;
}

/** Get current GPS time.
 *
 * \note The GPS time may only be a guess or completely unknown. time_quality
 *       should be checked first to determine the quality of the GPS time
 *       estimate.
 *
 * \return Current GPS time, or {WN_UNKNOWN, TOW_UNKNOWN} if no fix
 */
gps_time_t get_current_time(void) {
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
  gps_time_t t = persistent_clock_state.t_gps;
  double rate = persistent_clock_state.clock_rate;
  u64 ref_tc = persistent_clock_state.tc;
  chMtxUnlock(&clock_mutex);

  if (!gps_time_valid(&t)) {
    return GPS_TIME_UNKNOWN;
  }

  t.tow += (tc - ref_tc) * RX_DT_NOMINAL * rate;
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
u64 gpstime2napcount(const gps_time_t *t) {
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = persistent_clock_state.t_gps;
  double rate = persistent_clock_state.clock_rate;
  u64 ref_tc = persistent_clock_state.tc;
  chMtxUnlock(&clock_mutex);

  return ref_tc +
         (s64)round(gpsdifftime(t, &gps_time) / (RX_DT_NOMINAL * rate));
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

  /* initialize the clock state to unknown */
  chMtxLock(&clock_mutex);
  persistent_clock_state.t_gps = GPS_TIME_UNKNOWN;
  persistent_clock_state.t0_gps = GPS_TIME_UNKNOWN;
  chMtxUnlock(&clock_mutex);
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
 * \param time time to round
 * \param soln_freq solution frequency
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
  double clock_rate = persistent_clock_state.clock_rate;
  chMtxUnlock(&clock_mutex);

  return 1.0 - clock_rate;
}

/* Compute the sub-second portion of difference between NAP counter and gps time
 */
double subsecond_cpo_correction(u64 ref_tc) {
  /* Careful with numerical cancellation, we need this correction accurate to
   * the 10th decimal place. */
  gps_time_t ref_time = napcount2gpstime(ref_tc);
  double time_subsecond = ref_time.tow - round(ref_time.tow);
  u64 tc_subsecond = ref_tc % (u64)NAP_FRONTEND_SAMPLE_RATE_Hz;
  double cpo_correction = time_subsecond - RX_DT_NOMINAL * tc_subsecond;

  return cpo_correction - round(cpo_correction);
}

/** \} */
