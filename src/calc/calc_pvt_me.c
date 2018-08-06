/*
 * Copyright (C) 2014-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "calc_pvt_me.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <libswiftnav/almanac.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_meas.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/single_epoch_solver.h>

#include "calc_base_obs.h"
#include "calc_pvt_common.h"
#include "calc_starling_obs_array.h"
#include "hal/piksi_systime.h"
#include "main/main.h"
#include "ndb/ndb.h"
#include "observation_thread.h"
#include "track/track_sid_db.h"
#include "utils/ephemeris/ephemeris.h"
#include "utils/position/position.h"
#include "utils/signal_db/signal_db.h"
#include "utils/system_monitor/system_monitor.h"
#include "utils/timing/timing.h"

/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

/* Maximum time to maintain POSITION_FIX after last successful solution */
#define POSITION_FIX_TIMEOUT_S 60

#define ME_CALC_PVT_THREAD_PRIORITY (HIGHPRIO - 3)
#define ME_CALC_PVT_THREAD_STACK (64 * 1024)

/* STATIC FUNCTIONS */

/** Update the satellite azimuth & elevation database with current angles
 * \param rcv_pos Approximate receiver position
 * \param t Approximate time
 */
static void update_sat_azel(const double rcv_pos[3], const gps_time_t t) {
  ephemeris_t ephemeris;
  almanac_t almanac;

  /* compute elevation for any valid ephemeris/almanac we can pull from NDB */
  for (u16 sv_index = 0; sv_index < NUM_SATS; sv_index++) {
    /* form a SID with the first code for the constellation */
    gnss_signal_t sid = sv_index_to_sid(sv_index);
    if (!sid_valid(sid)) {
      continue;
    }
    /* try to compute from ephemeris */
    ndb_op_code_t res = ndb_ephemeris_read(sid, &ephemeris);
    if (NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res) {
      if (0 == update_azel_from_ephemeris(&ephemeris, &t, rcv_pos)) {
        /* success */
        continue;
      }
    }
    /* else try to compute from almanac */
    if (NDB_ERR_NONE == ndb_almanac_read(sid, &almanac)) {
      update_azel_from_almanac(&almanac, &t, rcv_pos);
    }
  }
}

/** Sleep until the next solution deadline.
 *
 * \param next_epoch  Next_epoch deadline, updated by this function.
 * \param interval_us Interval by which the deadline should be advanced [us].
 */
static void me_thd_sleep(piksi_systime_t *next_epoch, u32 interval_us) {
  u32 slept_us = 0;
  while (TRUE) {
    slept_us = piksi_systime_sleep_until_us(next_epoch);
    piksi_systime_inc_us(next_epoch, interval_us);

    if (0 == slept_us) {
      log_warn("High CPU, skip epoch");
    } else {
      break;
    }
  }
}

static THD_WORKING_AREA(wa_me_calc_pvt_thread, ME_CALC_PVT_THREAD_STACK);

static void me_calc_pvt_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("me_calc_pvt");

  last_good_fix_t lgf;
  ndb_op_code_t res = ndb_lgf_read(&lgf);
  if (NDB_ERR_NONE != res && NDB_ERR_GPS_TIME_MISSING != res) {
    lgf.position_solution.valid = false;
    lgf.position_quality = POSITION_UNKNOWN;
  }

  /* solve position once a second */
  const double soln_freq = 1;

  piksi_systime_t next_epoch;
  piksi_systime_get(&next_epoch);
  piksi_systime_inc_us(&next_epoch, SECS_US / soln_freq);

  while (TRUE) {
    drop_glo_signals_on_leap_second();

    /* sleep until next epoch, and update the deadline */
    me_thd_sleep(&next_epoch, SECS_US / soln_freq);
    watchdog_notify(WD_NOTIFY_ME_CALC_PVT);

    if (TIME_UNKNOWN != get_time_quality() && lgf.position_solution.valid &&
        lgf.position_quality >= POSITION_GUESS) {
      /* Update the satellite elevation angles so that they stay current
       * (currently once every 30 seconds) */
      DO_EACH_MS(MAX_AZ_EL_AGE_SEC * SECS_MS / 2,
                 update_sat_azel(lgf.position_solution.pos_ecef,
                                 lgf.position_solution.time));
    }

    /* Take the current nap count as the reception time*/
    u64 current_tc = nap_timing_count();
    gps_time_t current_time = napcount2gpstime(current_tc);

    /* The desired output time is at the closest solution epoch to current GPS
     * time  */
    gps_time_t output_time = gps_time_round_to_epoch(&current_time, soln_freq);

    if (gps_time_valid(&output_time) &&
        gps_time_valid(&lgf.position_solution.time)) {
      /* too long time from last time solution, downgrade position quality */
      if (lgf.position_quality > POSITION_STATIC &&
          gpsdifftime(&output_time, &lgf.position_solution.time) >
              POSITION_FIX_TIMEOUT_S) {
        lgf.position_quality = POSITION_STATIC;
      }

      if (gpsdifftime(&output_time, &lgf.position_solution.time) <= 0) {
        /* We are already past the next solution epoch, can happen when solution
         * frequency changes */
        log_info(
            "Next epoch (wn %d tow %f) is in the past wrt (wn %d tow %f), "
            "skipping",
            output_time.wn,
            output_time.tow,
            lgf.position_solution.time.wn,
            lgf.position_solution.time.tow);
        continue;
      }

      /* get NAP count at the desired output time */
      u64 output_tc = gpstime2napcount(&output_time);

      /* time difference of current NAP count from the output epoch */
      double dt = ((s64)output_tc - (s64)current_tc) * RX_DT_NOMINAL;

      if (fabs(dt) < OBS_PROPAGATION_LIMIT) {
        /* dampen small adjustments to get stabler corrections */
        dt *= 0.5;
      }

      /* Adjust the deadline for the next wake-up to get it to land closer to
       * the epoch. Note that the sleep time can be set only at the resolution
       * of system tick frequency, and also due to other CPU load, we can expect
       * this adjustment to be somewhere between +-0.5 milliseconds */
      piksi_systime_add_us(&next_epoch, round(dt * SECS_US));
    }

    /* Collect measurements from trackers, load ephemerides and compute flags.
     * Reference the measurements to the current time. */
    static navigation_measurement_t nav_meas[MAX_CHANNELS];
    static ephemeris_t e_meas[MAX_CHANNELS];

    u8 n_ready =
        collect_nav_meas(current_tc, &current_time, &lgf, nav_meas, e_meas);

    if (n_ready < MINIMUM_SV_COUNT ||
        nav_meas_get_sat_count(n_ready, nav_meas) < MINIMUM_SV_COUNT) {
      /* Not enough sats to even try PVT */
      continue;
    }

    dops_t dops;
    gnss_solution current_fix;

    /* Calculate the SPP position
     * disable_raim controlled by external setting. Defaults to false. */
    /* Don't skip velocity solving. If there is a cycle slip, tdcp_doppler will
     * just return the rough value from the tracking loop. */
    s8 pvt_ret = calc_PVT(n_ready,
                          nav_meas,
                          &current_time,
                          disable_raim,
                          false,
                          GPS_L1CA_WHEN_POSSIBLE,
                          &current_fix,
                          &dops,
                          NULL);
    if (pvt_ret < 0 || (lgf.position_quality == POSITION_FIX &&
                        gate_covariance(&current_fix))) {
      if (pvt_ret < 0) {
        /* An error occurred with calc_PVT! */
        /* pvt_err_msg defined in libswiftnav/pvt.c */
        /* Print out max. once per second */
        DO_EACH_MS(SECS_MS,
                   log_warn("PVT solver: %s (code %d)",
                            pvt_err_msg[-pvt_ret - 1],
                            pvt_ret));
      }

      /* If we already had a good fix, degrade its quality to STATIC */
      if (lgf.position_quality > POSITION_STATIC) {
        lgf.position_quality = POSITION_STATIC;
      }

      continue;
    }

    time_quality_t old_time_quality = get_time_quality();

    /* Update the relationship between the solved GPS time and NAP count tc.*/
    update_time(current_tc, &current_fix);

    /* Get the updated time and drift */
    gps_time_t smoothed_time = napcount2gpstime(current_tc);
    double smoothed_drift = get_clock_drift();

    /* if desired output time is still unknown, use the epoch closest to the fix
     * time */
    if (!gps_time_valid(&output_time)) {
      output_time = gps_time_round_to_epoch(&current_fix.time, soln_freq);
    }

    /* offset of smoothed solution time from the desired output time */
    double output_offset = gpsdifftime(&output_time, &smoothed_time);

    /* Update global position solution state. */
    lgf.position_solution = current_fix;
    lgf.position_quality = POSITION_FIX;
    /* Store the smoothed clock solution into lgf */
    lgf.position_solution.time = smoothed_time;
    lgf.position_solution.clock_drift = smoothed_drift;
    ndb_lgf_store(&lgf);

    if (TIME_PROPAGATED > old_time_quality) {
      /* If the time quality was not at least TIME_PROPAGATED then this solution
       * likely causes a clock jump and should be discarded.
       *
       * Note that the lack of knowledge of the receiver clock bias does NOT
       * degrade the quality of the position solution but the rapid change in
       * bias after the time estimate is first improved may cause issues for
       * e.g. carrier smoothing. Easier just to discard this first solution.
       */

      log_info("first fix clk_offset %.3e clk_drift %.3e",
               current_fix.clock_offset,
               current_fix.clock_drift);

      /* adjust the deadline of the next fix to land on output epoch */
      piksi_systime_add_us(&next_epoch, round(output_offset * SECS_US));
      continue;
    }

    if (fabs(current_fix.clock_offset) > MAX_CLOCK_ERROR_S) {
      /* Note we should not enter here except in very exceptional circumstances,
       * like time solved grossly wrong on the first fix. */

      log_warn("Receiver clock offset %g ms larger than %g ms",
               current_fix.clock_offset * SECS_MS,
               MAX_CLOCK_ERROR_S * SECS_MS);
    }
  }
}

void me_calc_pvt_setup() {
  /* Start solution thread */
  chThdCreateStatic(wa_me_calc_pvt_thread,
                    sizeof(wa_me_calc_pvt_thread),
                    ME_CALC_PVT_THREAD_PRIORITY,
                    me_calc_pvt_thread,
                    NULL);
}
