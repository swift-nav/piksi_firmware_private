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

/** Global time estimate quality state.
 * See \ref time_quality_t for possible values. */
volatile time_quality_t time_quality = TIME_UNKNOWN;

/** Clock state */
static volatile clock_est_state_t clock_state;

/** Mutex for guarding clock state access */
static MUTEX_DECL(clock_mutex);

/** Update GPS time estimate.
 *
 * This function updates the GPS time estimate precisely referenced against
 * receiver time. Depending on the current and new time quality, we either
 * re-initialize the clock model, or fine-tune the clock offset, or ignore the
 * new estimate altogether if current time quality is already better.
 *
 * \param quality Quality of the time estimate.
 * \param t Pointer to GPS time estimate.
 * \param tc SwiftNAP timing count.
 */
void set_time(time_quality_t new_quality, const gps_time_t *t, u64 tc) {
  bool quality_updated = false;

  if (time_quality >= TIME_FINE && new_quality >= TIME_FINE) {
    /* Time quality is already fine, so we just fine-tune the clock offset */
    gps_time_t rcv_time = napcount2rcvtime(tc);
    double time_diff = gpsdifftime(&rcv_time, t);

    chMtxLock(&clock_mutex);
    clock_state.clock_offset = time_diff;
    if (new_quality != time_quality) {
      time_quality = new_quality;
      quality_updated = true;
    }
    chMtxUnlock(&clock_mutex);
  } else if (new_quality >= time_quality) {
    /* Current time quality is less than fine: re-initialize the clock state */
    gps_time_t t0 = *t;
    add_secs(&t0, -(tc * RX_DT_NOMINAL));

    chMtxLock(&clock_mutex);
    clock_state.t0_gps = t0;
    clock_state.clock_offset = 0.0;
    clock_state.clock_period = RX_DT_NOMINAL;
    time_quality = new_quality;
    chMtxUnlock(&clock_mutex);

    quality_updated = true;
  }

  if (quality_updated) {
    time_t unix_t = gps2time(t);
    log_info("(quality=%d) Time set to: %s", new_quality, ctime(&unix_t));
  }
}

/** Downgrade GPS time estimate quality.
 *
 * This is to be used when time quality drops.
 *
 * \param quality Quality of the time estimate.
 */
void downgrade_time_quality(time_quality_t quality) {
  bool updated = false;

  chMtxLock(&clock_mutex);
  if (quality < time_quality) {
    time_quality = quality;
    updated = true;
  }
  chMtxUnlock(&clock_mutex);

  if (updated) {
    log_info("(quality=%d) Downgraded time quality", quality);
  }
}

/** Retrieve GPS time estimate quality.
 *
 */
time_quality_t get_time_quality(void) {
  chMtxLock(&clock_mutex);
  time_quality_t tq = time_quality;
  chMtxUnlock(&clock_mutex);
  return tq;
}

void clock_est_init(clock_est_state_t *s) {
  chMtxLock(&clock_mutex);
  s->t0_gps = GPS_TIME_UNKNOWN;
  s->clock_period = RX_DT_NOMINAL;
  s->clock_offset = 0.0;
  s->P[0][0] = 500e-3;
  s->P[0][1] = 0;
  s->P[1][0] = 0;
  s->P[1][1] = (double)RX_DT_NOMINAL * RX_DT_NOMINAL / 1e12; /* 1ppm. */
  chMtxUnlock(&clock_mutex);
}

/** Update receiver time referenced to GPS time
 *
 * \param dt clock adjustment (s)
 */
void adjust_time_fine(const double dt) {
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = clock_state.t0_gps;
  gps_time.tow -= dt;
  normalize_gps_time(&gps_time);
  clock_state.t0_gps = gps_time;
  clock_state.clock_offset -= dt;
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
  gps_time_t t = napcount2rcvtime(tc);

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
  gps_time_t t = clock_state.t0_gps;
  if (gps_time_valid(&t)) {
    t.tow += tc * clock_state.clock_period - clock_state.clock_offset;
    normalize_gps_time(&t);
  }
  chMtxUnlock(&clock_mutex);

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
  if (gps_time_valid(&t)) {
    t.tow += tc * clock_state.clock_period;
    normalize_gps_time(&t);
  }
  chMtxUnlock(&clock_mutex);

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
  gps_time_t gps_time = clock_state.t0_gps;
  gps_time.tow -= clock_state.clock_offset;
  double clock_period = clock_state.clock_period;
  chMtxUnlock(&clock_mutex);

  return gpsdifftime(t, &gps_time) / clock_period;
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
  gps_time_t gps_time = clock_state.t0_gps;
  double clock_period = clock_state.clock_period;
  chMtxUnlock(&clock_mutex);

  return gpsdifftime(t, &gps_time) / clock_period;
}

/** Callback to set receiver GPS time estimate. */
static void set_time_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)sender_id;
  (void)len;
  (void)context;

  gps_time_t *t = (gps_time_t *)msg;

  set_time(TIME_COARSE, t, nap_timing_count());
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
  assert(gps_time_valid(time));
  gps_time_t rounded_time = GPS_TIME_UNKNOWN;
  /* round the time-of-week */
  rounded_time.tow = round(time->tow * soln_freq) / soln_freq;
  /* handle case where rounding caused tow roll-over */
  normalize_gps_time(&rounded_time);
  /* pick the correct week number */
  gps_time_match_weeks(&rounded_time, time);
  return rounded_time;
}

/** \} */
