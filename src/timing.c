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
#include "board/v3/clk_dac.h"
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
    time_t unix_t = gps2time(&t);
    log_info("Time set to: %s (quality=%d)", ctime(&unix_t), quality);
  }
}

void clock_est_init(clock_est_state_t *s)
{
  chMtxLock(&clock_mutex);
  s->t0_gps.wn = 0;
  s->t0_gps.tow = 0;
  s->clock_period = RX_DT_NOMINAL;
  s->P[0][0] = 500e-3;
  s->P[0][1] = 0;
  s->P[1][0] = 0;
  s->P[1][1] = (double)RX_DT_NOMINAL * RX_DT_NOMINAL / 1e12; /* 1ppm. */
  chMtxUnlock(&clock_mutex);
}
/*
    def update(self, est_gpsT, est_bias, localT, q, rT, rTdot):
        phi_t_0 = array([[1., localT], [0, 1]])
        phi_0_t = array([[1., -localT], [0, 1]])

        # Predict:
        # No state update, static
        x_ = self.x
        # Predict covariance
        P_ = self.P + array([[q, 0.], [0, 0]])

        # Update:
        # Calc. innovation
        z = array([est_gpsT, est_bias])
        y = z - phi_t_0.dot(x_)
        # Calc. innovation covariance
        S = phi_t_0.dot(P_).dot(phi_t_0.transpose()) + array([[rT, 0.], [0, rTdot]])
        # Kalman gain
        #K = phi_t_0.dot(P_).dot(phi_t_0.transpose()).dot(linalg.inv(S))
        K = P_.dot(phi_t_0.transpose()).dot(linalg.inv(S))
        # Update state estimate
        self.x += K.dot(y)
        # Update covariance
        self.P = (array([[1., 0], [0, 1]]) - K.dot(phi_t_0)).dot(P_)
  */

void clock_est_update(clock_est_state_t *s, gps_time_t meas_gpst,
                      double meas_clock_period, double localt, double q,
                      double r_gpst, double r_clock_period)
{
  chMtxLock(&clock_mutex);

  double temp[2][2];

  double phi_t_0[2][2] = {{1, localt}, {0, 1}};
  double phi_t_0_tr[2][2];
  matrix_transpose(2, 2, (const double *)phi_t_0, (double *)phi_t_0_tr);

  double P_[2][2];
  memcpy(P_, s->P, sizeof(P_));
  P_[0][0] += q;

  double y[2];
  gps_time_t pred_gpst = s->t0_gps;
  pred_gpst.tow += localt * s->clock_period;
  normalize_gps_time(&pred_gpst);
  y[0] = gpsdifftime(&meas_gpst, &pred_gpst);
  y[1] = meas_clock_period - s->clock_period;

  double S[2][2];
  matrix_multiply(2, 2, 2, (const double *)phi_t_0, (const double *)P_, (double *)temp);
  matrix_multiply(2, 2, 2, (const double *)temp, (const double *)phi_t_0_tr, (double *)S);
  S[0][0] += r_gpst;
  S[1][1] += r_clock_period;
  double Sinv[2][2];
  matrix_inverse(2, (const double *)S, (double *)Sinv);

  double K[2][2];
  matrix_multiply(2, 2, 2, (const double *)P_, (const double *)phi_t_0_tr, (double *)temp);
  matrix_multiply(2, 2, 2, (const double *)temp, (const double *)Sinv, (double *)K);

  double dx[2];
  matrix_multiply(2, 2, 1, (const double *)K, (const double *)y, (double *)dx);
  s->t0_gps.tow += dx[0];
  normalize_gps_time(&s->t0_gps);
  s->clock_period += dx[1];

  matrix_multiply(2, 2, 2, (const double *)K, (const double *)phi_t_0, (double *)temp);
  temp[0][0] = 1 - temp[0][0];
  temp[0][1] = -temp[0][1];
  temp[1][1] = 1 - temp[1][1];
  temp[1][0] = -temp[1][0];
  matrix_multiply(2, 2, 2, (const double *)temp, (const double *)P_, (double *)s->P);

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
  time_quality = TIME_FINE;
  chMtxUnlock(&clock_mutex);

  time_t unix_t = gps2time(&t);
  log_info("Time set to: %s (quality=%d)", ctime(&unix_t), TIME_FINE);
}

/** Update GPS time estimate precisely referenced to the local receiver time.
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
  t.tow += tc * clock_state.clock_period;
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
double gps2rxtime(gps_time_t* t)
{
  chMtxLock(&clock_mutex);
  gps_time_t gps_time = clock_state.t0_gps;
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

/* TODO doxygen */
void steer_clock(double clock_offset, double clock_drift) {
  log_info("clock_offset, %.10f, clock_drift %.10f", clock_offset, clock_drift);

  /* Max TCXO drift before PLL lock loss is +/- 10 Hz/s = +/- 1 ppm @ 10 MHz
   * For IQD 10 MHz TCXO and 8 bit DAC (0 - 3.3V) the output values are:
   * DAC value    Control voltage    Clock drift
   * 112          1.444 V            -1.125 ppm
   * 113          1.457 V            -0.867 ppm
   * 114          1.470 V            -0.609 ppm
   * 115          1.482 V            -0.352 ppm
   * 116          1.495 V            -0.094 ppm
   * 117          1.508 V             0.164 ppm
   * 118          1.521 V             0.422 ppm
   * 119          1.534 V             0.680 ppm
   * 120          1.547 V             0.938 ppm
   * 121          1.560 V             1.195 ppm
   */

  /* 8 bit DAC drives 0 - 3.3V, TCXO control midpoint is 1.5V */
  set_clk_dac(128, CLK_DAC_MODE_0);
}

/** \} */
