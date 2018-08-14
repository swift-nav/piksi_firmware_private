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

/* Solve ME PVT every 2 seconds */
#define ME_PVT_SOLN_FREQ 0.5

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

static void drop_gross_outlier(const navigation_measurement_t *nav_meas,
                               const gnss_solution *current_fix) {
  /* Check how large the outlier roughly is, and if it is a gross one,
   * drop the channel and delete the possibly corrupt ephemeris */
  double geometric_range[3];
  for (u8 j = 0; j < 3; j++) {
    geometric_range[j] = nav_meas->sat_pos[j] - current_fix->pos_ecef[j];
  }
  double pseudorng_error =
      fabs(nav_meas->pseudorange - current_fix->clock_offset * GPS_C -
           vector_norm(3, geometric_range));

  bool generic_gross_outlier = pseudorng_error > RAIM_DROP_CHANNEL_THRESHOLD_M;
  if (generic_gross_outlier) {
    /* mark channel for dropping */
    tracker_set_raim_flag(nav_meas->sid);
    /* clear the ephemeris for this signal */
    ndb_ephemeris_erase(nav_meas->sid);
  }

  bool boc_halfchip_outlier =
      (CODE_GAL_E1B == nav_meas->sid.code) && (pseudorng_error > 100);
  if (boc_halfchip_outlier) {
    /* mark channel for dropping */
    tracker_set_raim_flag(nav_meas->sid);
  }
}

static void me_calc_pvt_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("me_calc_pvt");

  last_good_fix_t lgf;
  ndb_op_code_t res = ndb_lgf_read(&lgf);
  if (NDB_ERR_NONE != res && NDB_ERR_GPS_TIME_MISSING != res) {
    lgf.position_solution.valid = false;
    lgf.position_quality = POSITION_UNKNOWN;
  }

  piksi_systime_t next_epoch;
  piksi_systime_get(&next_epoch);
  piksi_systime_inc_us(&next_epoch, SECS_US / soln_freq_setting);

  while (TRUE) {
    /* compute ME PVT solution at low frequency, unless there is currently no
     * time solution, in which case use the set soln frequency in order to not
     * degrade TTFF
     */
    double soln_freq = ME_PVT_SOLN_FREQ;
    if (TIME_PROPAGATED >= get_time_quality()) {
      chSysLock();
      soln_freq = soln_freq_setting;
      chSysUnlock();
    }

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
    gnss_sid_set_t raim_removed_sids;

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
                          &raim_removed_sids);
    if (pvt_ret < 0) {
      /* An error occurred with calc_PVT! */
      /* pvt_err_msg defined in libswiftnav/pvt.c */
      /* Print out max. once per second */
      DO_EACH_MS(
          SECS_MS,
          log_warn(
              "PVT solver: %s (code %d)", pvt_err_msg[-pvt_ret - 1], pvt_ret));

      /* If we already had a good fix, degrade its quality to STATIC */
      if (lgf.position_quality > POSITION_STATIC) {
        lgf.position_quality = POSITION_STATIC;
      }

      continue;
    }

    /* If we have a success RAIM repair, check for gross outliers and tracker to
     * drop those channels. */
    if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
      for (u8 i = 0; i < n_ready; i++) {
        if (sid_set_contains(&raim_removed_sids, nav_meas[i].sid)) {
          /* Check how large the outlier roughly is, and if it is a gross one,
           * drop the channel and delete the possibly corrupt ephemeris */
          drop_gross_outlier(&nav_meas[i], &current_fix);
        }
      }
    }

    log_info("PVT solution: tow %.3f clk_offset %.3e clk_drift %.3e",
             current_fix.time.tow,
             current_fix.clock_offset,
             current_fix.clock_drift);

    time_quality_t old_time_quality = get_time_quality();

    /* Update the relationship between the solved GPS time and NAP count tc.*/
    update_time(current_tc, &current_fix);

    /* Get the updated time and drift */
    gps_time_t smoothed_time = napcount2gpstime(current_tc);
    double smoothed_drift = get_clock_drift();

    /* Update global position solution state. */
    lgf.position_solution = current_fix;
    lgf.position_quality = POSITION_FIX;
    /* Store the smoothed clock solution into lgf */
    lgf.position_solution.time = smoothed_time;
    lgf.position_solution.clock_drift = smoothed_drift;
    ndb_lgf_store(&lgf);

    if (TIME_PROPAGATED > old_time_quality) {
      log_info("first fix clk_offset %.3e clk_drift %.3e",
               current_fix.clock_offset,
               current_fix.clock_drift);
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
