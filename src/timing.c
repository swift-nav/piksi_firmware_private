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

#include <math.h>
#include <string.h>
#include <time.h>

#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/linear_algebra.h>

#include "board/nap/nap_common.h"
#include "main.h"
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
static mutex_t clock_mutex;

/** Update GPS time estimate.
 *
 * This function may be called to update the GPS time estimate. If the time is
 * already known more precisely than the new estimate, the new estimate will be
 * ignored.
 *
 * This function should not be used to give an estimate with TIME_FINE quality
 * as this must be referenced to a particular value of the SwiftNAP timing
 * count.
 *
 * \param quality Quality of the time estimate.
 * \param t GPS time estimate.
 */
void set_time(time_quality_t quality, gps_time_t t)
{
  bool updated = false;
  gps_time_t norm_time = t;
  norm_time.tow -= nap_timing_count() * RX_DT_NOMINAL;
  normalize_gps_time(&norm_time);

  chMtxLock(&clock_mutex);
  if (quality > time_quality) {
    clock_state.t0_gps = norm_time;
    time_quality = quality;
    updated = true;
  }
  chMtxUnlock(&clock_mutex);

  if (updated) {
    log_warn("test1 set time");
    time_t unix_t = gps2time(&t);
    log_warn("test1 set time passed");
    log_info("Time set to: %s (quality=%d)", ctime(&unix_t), quality);
  }
}

void clock_est_init(clock_est_state_t *s)
{
  chMtxLock(&clock_mutex);
  s->t0_gps.wn = 0;
  s->t0_gps.tow = 0;
  s->clock_period = RX_DT_NOMINAL;
  s->clock_offset = 0.0;
  s->P[0][0] = 500e-3;
  s->P[0][1] = 0;
  s->P[1][0] = 0;
  s->P[1][1] = (double)RX_DT_NOMINAL * RX_DT_NOMINAL / 1e12; /* 1ppm. */
  chMtxUnlock(&clock_mutex);
}

/** Update GPS time estimate precisely referenced to the local receiver time.
 *
 * \param tc SwiftNAP timing count.
 * \param t GPS time estimate associated with timing count.
 */
void set_time_fine(u64 tc, gps_time_t t)
{
  gps_time_t norm_time = t;
  norm_time.tow -= tc * RX_DT_NOMINAL;
  normalize_gps_time(&norm_time);

  chMtxLock(&clock_mutex);
  clock_state.t0_gps = norm_time;
  clock_state.clock_offset = 0.0;
  time_quality = TIME_FINE;
  chMtxUnlock(&clock_mutex);

  time_t unix_t = gps2time(&t);
  log_info("Time set to: %s (quality=%d)", ctime(&unix_t), TIME_FINE);
}

/** Update GPS time estimate precisely referenced to the local receiver time.
 *
 * \param tc SwiftNAP timing count.
 * \param t GPS time estimate associated with timing count.
 */
void set_gps_time_offset(u64 tc, gps_time_t t)
{
  gps_time_t rcv_time = rx2gpstime(tc);
  double time_diff = gpsdifftime(&rcv_time, &t);

  chMtxLock(&clock_mutex);
  clock_state.clock_offset += time_diff;
  chMtxUnlock(&clock_mutex);
}

/** Update receiver time referenced to GPS time
 *
 * \param dt clock adjustment (s)
 */
void adjust_time_fine(double dt)
{
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = clock_state.t0_gps;
  gps_time.tow -= dt;
  normalize_gps_time(&gps_time);
  clock_state.t0_gps = gps_time;
  clock_state.clock_offset -= dt;
  chMtxUnlock(&clock_mutex);
}

/** Get current GPS time.
 *
 * \note The GPS time may only be a guess or completely unknown. time_quality
 *       should be checked first to determine the quality of the GPS time
 *       estimate.
 *
 * This function should be used only for approximate timing purposes as simply
 * calling this function does not give a well defined instant at which the GPS
 * time is queried.
 *
 * \return Current GPS time.
 */
gps_time_t get_current_time(void)
{
  /* TODO: Return invalid when TIME_UNKNOWN. */
  /* TODO: Think about what happens when nap_timing_count overflows. */
  u64 tc = nap_timing_count();
  gps_time_t t = rx2gpstime(tc);

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
gps_time_t rx2gpstime(double tc)
{
  chMtxLock(&clock_mutex);
  gps_time_t t = clock_state.t0_gps;
  t.tow += tc * clock_state.clock_period - clock_state.clock_offset;
  chMtxUnlock(&clock_mutex);

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
double gps2rxtime(const gps_time_t* t)
{
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = clock_state.t0_gps;
  gps_time.tow -= clock_state.clock_offset;
  double clock_period = clock_state.clock_period;
  chMtxUnlock(&clock_mutex);

  return gpsdifftime(t, &gps_time) / clock_period;
}

/** Callback to set receiver GPS time estimate. */
static void set_time_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  gps_time_t *t = (gps_time_t *)msg;

  set_time(TIME_COARSE, *t);
}

/** Setup timing functionality.
 * For now just register a callback so that a coarse time can be sent by the
 * host. */
void timing_setup(void)
{
  /* TODO: Perhaps setup something to check for nap_timing_count overflows
   * periodically. */
  static sbp_msg_callbacks_node_t set_time_node;

  chMtxObjectInit(&clock_mutex);

  sbp_register_cbk(SBP_MSG_SET_TIME, &set_time_callback, &set_time_node);

  clock_est_init((clock_est_state_t*)&clock_state);
}

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
u64 timing_getms(void)
{
  return (u64)(nap_timing_count() * (RX_DT_NOMINAL * 1000.0));
}
/** \} */
