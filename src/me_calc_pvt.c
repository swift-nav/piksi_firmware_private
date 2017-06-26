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
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <libswiftnav/cnav_msg.h>
#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/troposphere.h>
#include <libswiftnav/sid_set.h>

#include "me_calc_pvt.h"


#include "peripherals/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "manage.h"

#include "settings.h"
#include "timing.h"
#include "ephemeris.h"
#include "signal.h"
#include "system_monitor.h"
#include "main.h"
#include "cnav_msg_storage.h"
#include "ndb.h"
#include "shm.h"
#include "common_calc_pvt.h"

/* Maximum CPU time the solution thread is allowed to use. */
#define SOLN_THD_CPU_MAX (0.60f)

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

/** Mandatory flags filter for measurements */
#define MANAGE_TRACK_FLAGS_FILTER (MANAGE_TRACK_FLAG_ACTIVE | \
                                   MANAGE_TRACK_FLAG_NO_ERROR | \
                                   MANAGE_TRACK_FLAG_CONFIRMED | \
                                   MANAGE_TRACK_FLAG_CN0_SHORT | \
                                   MANAGE_TRACK_FLAG_ELEVATION | \
                                   MANAGE_TRACK_FLAG_HAS_EPHE | \
                                   MANAGE_TRACK_FLAG_HEALTHY | \
                                   MANAGE_TRACK_FLAG_NAV_SUITABLE | \
                                   MANAGE_TRACK_FLAG_TOW )
/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

MemoryPool obs_buff_pool;
mailbox_t  obs_mailbox;

double soln_freq = 5.0;
u32 obs_output_divisor = 1;

s16 msg_obs_max_size = SBP_FRAMING_MAX_PAYLOAD_SIZE;



static bool disable_raim = false;

static soln_stats_t last_stats = { .signals_tracked = 0, .signals_useable = 0 };
static soln_pvt_stats_t last_pvt_stats = { .systime = -1, .signals_used = 0 };


/* RFT_TODO *
 * check that Klobuchar is used in SPP solver */

/* STATIC FUNCTIONS */



static void me_post_observations(u8 n,
                                 const navigation_measurement_t _meas[],
                                 const ephemeris_t _ephem[])
{
  /* TODO: use a buffer from the pool from the start instead of
   * allocating nav_meas_tdcp as well. Downside, if we don't end up
   * pushing the message into the mailbox then we just wasted an
   * observation from the mailbox for no good reason. */

  me_msg_obs_t *me_msg_obs = chPoolAlloc(&obs_buff_pool);
  msg_t ret;
  if (me_msg_obs == NULL) {
    log_error("ME: Could not allocate pool!");
    return;
  }

  me_msg_obs->size = n;
  if (n) {
    memcpy(me_msg_obs->obs,    _meas, n*sizeof(navigation_measurement_t));
    memcpy(me_msg_obs->ephem, _ephem, n*sizeof(ephemeris_t));
  }

  ret = chMBPost(&obs_mailbox, (msg_t)me_msg_obs, TIME_IMMEDIATE);
  if (ret != MSG_OK) {
    /* We could grab another item from the mailbox, discard it and then
     * post our obs again but if the size of the mailbox and the pool
     * are equal then we should have already handled the case where the
     * mailbox is full when we handled the case that the pool was full.
     * */
    log_error("ME: Mailbox should have space!");
    chPoolFree(&obs_buff_pool, me_msg_obs);
  }
}


static void me_send_all(u8 _num_obs,
                        const navigation_measurement_t _meas[],
                        const ephemeris_t _ephem[],
                        const gps_time_t *_t)
{
  me_post_observations(_num_obs, _meas, _ephem);
  send_observations(_num_obs, msg_obs_max_size, _meas, _t);
}


static void me_send_emptyobs(void) {
  me_post_observations(0, NULL, NULL);
  send_observations(0, msg_obs_max_size, NULL, NULL);
}

/** Update the satellite azimuth & elevation database with current angles
 * \param rcv_pos Approximate receiver position
 * \param t Approximate time
 */
static void update_sat_azel(const double rcv_pos[3], const gps_time_t t)
{
  ephemeris_t ephemeris;
  almanac_t almanac;
  double az, el;
  u64 nap_count = gpstime2napcount(&t);

  /* compute elevation for any valid ephemeris/almanac we can pull from NDB */
  for (u16 sv_index = 0; sv_index < NUM_SATS; sv_index++) {

    /* form a SID with the first code for the constellation */
    gnss_signal_t sid = sv_index_to_sid(sv_index);
    if (!sid_valid(sid)) {
      continue;
    }
    ndb_op_code_t res = ndb_ephemeris_read(sid, &ephemeris);

    /* try to compute elevation from any valid ephemeris */
    if ((NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res)
        && ephemeris_valid(&ephemeris, &t)
        && calc_sat_az_el(&ephemeris, &t, rcv_pos, &az, &el, false) >= 0) {
      sv_azel_degrees_set(sid, round(az * R2D), round(el * R2D), nap_count);
      log_debug_sid(sid, "Updated elevation from ephemeris %.1f", el * R2D);

    /* else try to fetch almanac and use it if it is valid */
    } else if (NDB_ERR_NONE == ndb_almanac_read(sid, &almanac)
               && calc_sat_az_el_almanac(&almanac, &t, rcv_pos, &az, &el) >= 0) {
      sv_azel_degrees_set(sid, round(az * R2D), round(el * R2D), nap_count);
      log_debug_sid(sid, "Updated elevation from almanac %.1f", el * R2D);
    }
  }
}

/** Sleep until the next solution deadline.
 *
 * \param deadline    Pointer to the current deadline, updated by this function.
 * \param interval    Interval by which the deadline should be advanced.
 */
static void sol_thd_sleep(systime_t *deadline, systime_t interval)
{
  *deadline += interval;

  chSysLock();
  while (1) {
    /* Sleep for at least (1-SOLN_THD_CPU_MAX) * interval ticks so that
     * execution time is limited to SOLN_THD_CPU_MAX. */
    systime_t systime = chVTGetSystemTimeX();
    systime_t delta = *deadline - systime;
    systime_t sleep_min = (systime_t)ceilf((1.0f-SOLN_THD_CPU_MAX) * interval);
    if ((systime_t)(delta - sleep_min) <= ((systime_t)-1) / 2) {
      chThdSleepS(delta);
      break;
    } else {
      chSysUnlock();
      if (delta <= ((systime_t)-1) / 2) {
        /* Deadline is in the future. Skipping due to high CPU usage. */
        log_warn("Solution thread skipping deadline, "
                 "time = %lu, deadline = %lu", systime, *deadline);
      } else {
        /* Deadline is in the past. */
        log_warn("Solution thread missed deadline, "
                 "time = %lu, deadline = %lu", systime, *deadline);
      }
      *deadline += interval;
      chSysLock();
    }
  }
  chSysUnlock();
}

/**
 * Collects channel measurements, ephemerides and auxiliary data.
 *
 * \param[in]  rec_tc    Timestamp [samples].
 * \param[out] meas      Destination measurement array.
 * \param[out] in_view   Destination in_view array.
 * \param[out] ephe      Destination ephemeris array.
 * \param[out] pn_ready  Destination for measurement array size.
 * \param[out] pn_ready  Destination for in-view array size.
 * \param[out] pn_total  Destination for total active trackers count.
 *
 * \return None
 */
static void collect_measurements(u64 rec_tc,
                                 channel_measurement_t meas[MAX_CHANNELS],
                                 channel_measurement_t in_view[MAX_CHANNELS],
                                 ephemeris_t ephe[MAX_CHANNELS],
                                 u8 *pn_ready,
                                 u8 *pn_inview,
                                 u8 *pn_total)
{
  u8 n_collected = 0;
  u8 n_inview = 0;
  u8 n_active = 0;

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    u32 flags = 0; /* Channel flags accumulator */
    /* Load measurements from the tracking channel and ephemeris from NDB */
    flags = get_tracking_channel_meas(i, rec_tc,
                                      &meas[n_collected],
                                      &ephe[n_collected]);

    if (0 != (flags & TRACKER_FLAG_ACTIVE) &&
        0 != (flags & TRACKER_FLAG_CONFIRMED) &&
        0 == (flags & TRACKER_FLAG_ERROR) &&
        0 == (flags & TRACKER_FLAG_MASKED))
    {
      /* Tracking channel is active & not masked */
      n_active++;

      if (0 == (flags & TRACKER_FLAG_XCORR_SUSPECT)) {
        /* Tracking channel is not XCORR suspect so it's an actual SV in view */
        in_view[n_inview++] = meas[n_collected];
      }

      chan_meas_flags_t meas_flags = meas[n_collected].flags;

      if (0 != (flags & TRACKER_FLAG_HEALTHY) &&
          0 != (flags & TRACKER_FLAG_NAV_SUITABLE) &&
          0 != (flags & TRACKER_FLAG_ELEVATION) &&
          0 != (flags & TRACKER_FLAG_TOW_VALID) &&
          0 != (flags & TRACKER_FLAG_HAS_EPHE) &&
          0 != (flags & TRACKER_FLAG_CN0_SHORT) &&
          0 != (meas_flags & CHAN_MEAS_FLAG_CODE_VALID) &&
          0 != (meas_flags & CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
        /* Tracking channel is suitable for solution calculation */
        n_collected++;
      }
    }
  }

  *pn_ready = n_collected;
  *pn_inview = n_inview;
  *pn_total = n_active;
}

static THD_WORKING_AREA(wa_me_calc_pvt_thread, 1024*1024);
static void me_calc_pvt_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("me_calc_pvt");

  systime_t deadline = chVTGetSystemTime();

  /* RFT_TODO *
   * clock_jump was after all masked by the TIME_FINE state variable
   * and as not used generated compiler errors */
  last_good_fix_t lgf;

  ndb_lgf_read(&lgf);

  while (TRUE) {

    sol_thd_sleep(&deadline, CH_CFG_ST_FREQUENCY/soln_freq);
    watchdog_notify(WD_NOTIFY_ME_CALC_PVT);

    // Take the current nap count
    u64 current_tc = nap_timing_count();
    u64 rec_tc = current_tc;

    // Work out the expected receiver time in GPS time frame for the current nap count
    gps_time_t rec_time = napcount2rcvtime(rec_tc);
    gps_time_t expected_time = GPS_TIME_UNKNOWN;

    // If we've previously had a solution, we can work out our expected obs time
    if (time_quality == TIME_FINE) {
      // Work out the time of the current nap count
      expected_time = napcount2gpstime(rec_tc);

      // Round this time to the nearest GPS solution time
      expected_time.tow = round(expected_time.tow * soln_freq)
                                 / soln_freq;
      normalize_gps_time(&expected_time);

      // This time, taken back to nap count, is the nap count we want the observations at
      rec_tc = (u64)(round(gpstime2napcount(&expected_time)));
    }
    // The difference between the current nap count and the nap count we want the observations at
    // is the amount we want to adjust our deadline by at the end of the solution
    double delta_tc = -((double)current_tc - (double)rec_tc);

    // Get the expected nap count in receiver time (gps time frame)
    rec_time = napcount2rcvtime(rec_tc);

    if (time_quality >= TIME_COARSE
        && lgf.position_solution.valid
        && lgf.position_quality >= POSITION_GUESS) {
      /* Update the satellite elevation angles so that they stay current
       * (currently once every 30 seconds) */
      DO_EVERY((u32)soln_freq * MAX_AZ_EL_AGE_SEC/2,
               update_sat_azel(lgf.position_solution.pos_ecef,
                               lgf.position_solution.time));
    }

    u8 n_ready = 0;
    u8 n_inview = 0;
    u8 n_total = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    channel_measurement_t in_view[MAX_CHANNELS];
    static ephemeris_t e_meas[MAX_CHANNELS];

    /* TODO: package meas, nav_meas and ephemeris into e.g. an union to
     * keep them from being separated? */

    /* Collect measurements from trackers, load ephemerides and compute flags */
    collect_measurements(rec_tc,
                         meas,
                         in_view,
                         e_meas,
                         &n_ready,
                         &n_inview,
                         &n_total);

    nmea_send_gsv(n_inview, in_view);

    log_debug("Selected %" PRIu8 " measurement(s) out of %" PRIu8 " in view "
              " (total=%" PRIu8 ")", n_ready, n_inview, n_total);

    /* Update stats */
    last_stats.signals_tracked = n_total;
    last_stats.signals_useable = n_ready;

    if (n_ready < MINIMUM_SV_COUNT) {
      /* Not enough sats, keep on looping. */
      me_send_emptyobs();
      continue;
    }

    cnav_msg_t cnav_30[MAX_CHANNELS];
    const cnav_msg_type_30_t *p_cnav_30[MAX_CHANNELS];
    for (u8 i=0; i < n_ready; i++) {
      p_cnav_30[i] = cnav_msg_get(meas[i].sid, CNAV_MSG_TYPE_30, &cnav_30[i]) ?
                     &cnav_30[i].data.type_30 : NULL;
    }

    static navigation_measurement_t nav_meas[MAX_CHANNELS];
    const channel_measurement_t *p_meas[n_ready];
    navigation_measurement_t *p_nav_meas[n_ready];
    const ephemeris_t *p_e_meas[n_ready];

    /* Create arrays of pointers for use in calc_navigation_measurement */
    for (u8 i = 0; i < n_ready; i++) {
      p_meas[i] = &meas[i];
      p_nav_meas[i] = &nav_meas[i];
      p_e_meas[i] = &e_meas[i];
    }

    /* Create navigation measurements from the channel measurements */
    /* If we have timing then we can calculate the relationship between
     * receiver time and GPS time and hence provide the pseudorange
     * calculation with the local GPS time of reception. */
    /* If a FINE quality time solution is not available then don't pass in a
     * `nav_time`. This will result in valid pseudoranges but with a large
     * and arbitrary receiver clock error. We may want to discard these
     * observations after doing a PVT solution. */
    gps_time_t *p_rec_time = (time_quality == TIME_FINE) ? &rec_time : NULL;

    s8 nm_ret = calc_navigation_measurement(n_ready, p_meas, p_nav_meas,
                                            p_rec_time);

    if (nm_ret != 0) {
      log_error("calc_navigation_measurement() returned an error");
      me_send_emptyobs();
      continue;
    }

    /* RFT_TODO *
     * agreed this should go in starling but ME SPP solver needs it too */
    s8 sc_ret = calc_sat_clock_corrections(n_ready, p_nav_meas, p_e_meas);

    if (sc_ret != 0) {
      log_error("calc_sat_clock_correction() returned an error");
      me_send_emptyobs();
      continue;
    }

    calc_isc(n_ready, p_nav_meas, p_cnav_30);

    static double clock_offset_previous = 0;
    static u64 rec_tc_old = 0;
    static u8 n_ready_old = 0;
    static navigation_measurement_t nav_meas_old[MAX_CHANNELS];
    static navigation_measurement_t nav_meas_tdcp[MAX_CHANNELS];

    double rec_tc_delta = (double) (rec_tc - rec_tc_old)
                                  / NAP_FRONTEND_SAMPLE_RATE_Hz;
    u8 n_ready_tdcp;

    /* Pass the nav_meas with the measured Dopplers as is */
    memcpy(nav_meas_tdcp, nav_meas, sizeof(nav_meas));
    n_ready_tdcp = n_ready;

    /* Store current observations for next time for
     * TDCP Doppler calculation. */
    memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
    n_ready_old = n_ready;
    rec_tc_old = rec_tc;

    gnss_sid_set_t codes_tdcp;
    sid_set_init(&codes_tdcp);
    for (u8 i=0; i<n_ready_tdcp; i++) {
      sid_set_add(&codes_tdcp, nav_meas_tdcp[i].sid);
    }

    if (sid_set_get_sat_count(&codes_tdcp) < 4) {
      /* Not enough sats to compute PVT */
      me_send_emptyobs();
      continue;
    }

    /* check if we have a solution, if yes calc iono and tropo correction */
    if (lgf.position_quality >= POSITION_GUESS) {
      ionosphere_t i_params;
      ionosphere_t *p_i_params = &i_params;
      /* get iono parameters if available */
      if (ndb_iono_corr_read(p_i_params) != NDB_ERR_NONE) {
        p_i_params = NULL;
      }
      calc_iono_tropo(n_ready_tdcp, nav_meas_tdcp,
                    lgf.position_solution.pos_ecef,
                    lgf.position_solution.pos_llh,
                    p_i_params);
    }

    dops_t dops;
    gnss_solution current_fix;
    gnss_sid_set_t raim_removed_sids;

    /* Calculate the SPP position
     * disable_raim controlled by external setting. Defaults to false. */
    /* Don't skip velocity solving. If there is a cycle slip, tdcp_doppler will
     * just return the rough value from the tracking loop. */
     // TODO(Leith) check velocity_valid
    s8 pvt_ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp, disable_raim, false,
                          &current_fix, &dops, &raim_removed_sids);
    if (pvt_ret < 0
        || (lgf.position_quality == POSITION_FIX && gate_covariance(&current_fix))) {

      if (pvt_ret < 0) {
        /* An error occurred with calc_PVT! */
        /* pvt_err_msg defined in libswiftnav/pvt.c */
        DO_EVERY((u32)soln_freq,
                 log_warn("PVT solver: %s (code %d)", pvt_err_msg[-pvt_ret-1], pvt_ret);
        );
      }

      /* If we can't report a SPP position, something is wrong and no point
       * continuing to process this epoch - send out solution and observation
       * failed messages if not in time matched mode
       */
      me_send_emptyobs();

      /* If we already had a good fix, degrade its quality to STATIC */
      if (lgf.position_quality > POSITION_STATIC) {
        lgf.position_quality = POSITION_STATIC;
      }
      continue;
    }

    /* Determine the time differenced clock offset which is used to remove
       receiver clock bias from the computed doppler. */
    double computed_clock_rate =
            (current_fix.clock_offset - clock_offset_previous) / rec_tc_delta;
    clock_offset_previous = current_fix.clock_offset;

    /* If we have a success RAIM repair, mark the removed observation as
       invalid. In practice, this means setting only the CN0 flag valid. */
    if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        if (sid_set_contains(&raim_removed_sids, nav_meas_tdcp[i].sid)) {
          log_warn_sid(nav_meas_tdcp[i].sid, "RAIM repair, setting observation invalid.");
          nav_meas_tdcp[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;
        }
      }
    }

    if (time_quality < TIME_FINE) {
      /* If the time quality is not FINE then our receiver clock bias isn't
       * known. We should only use this PVT solution to update our time
       * estimate and then skip all other processing.
       *
       * Note that the lack of knowledge of the receiver clock bias does NOT
       * degrade the quality of the position solution but the rapid change in
       * bias after the time estimate is first improved may cause issues for
       * e.g. carrier smoothing. Easier just to discard this first solution.
       */
      set_time_fine(rec_tc, current_fix.time);

      me_send_emptyobs();

      /* store this fix as a guess so the satellite elevations and iono/tropo
       * corrections can be computed for the first actual fix */
      lgf.position_solution = current_fix;
      lgf.position_quality = POSITION_GUESS;
      ndb_lgf_store(&lgf);
      continue;
    }
    // We now have the nap count we expected the measurements to be at, plus the GPS time error for that nap count
    // so we need to store this error in the GPS time (GPS time frame)
    set_gps_time_offset(rec_tc, current_fix.time);

    /* Update global position solution state. */
    lgf.position_solution = current_fix;
    lgf.position_quality = POSITION_FIX;
    ndb_lgf_store(&lgf);

    /* RFT_TODO: Output solution. */



    /*
     * We need to correct our pseudorange and create a new one that is valid for
     * a different time of arrival.  In particular we'd like to propagate all the
     * observations such that they represent what we would have observed had
     * the observations all arrived at the current epoch (t').
     */

    /* Calculate the time of the nearest solution epoch, where we expected
     * to be, and calculate how far we were away from it. */
    gps_time_t new_obs_time = GPS_TIME_UNKNOWN;
    new_obs_time.tow = round(current_fix.time.tow * soln_freq)
                              / soln_freq;
    normalize_gps_time(&new_obs_time);
    gps_time_match_weeks(&new_obs_time, &current_fix.time);
    double t_err = gpsdifftime(&new_obs_time, &current_fix.time);

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(t_err) < OBS_PROPAGATION_LIMIT) {

      /* Propagate observations to desired time. */
      /* We have to use the tdcp_doppler result to account for TCXO drift. */
      /* nav_meas_tdcp is updated in place, skipping elements if required. */
      u8 n_ready_tdcp_new = 0;
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        navigation_measurement_t *nm = &nav_meas_tdcp[n_ready_tdcp_new];

        /* Copy measurement to new index if a previous measurement
         * has been skipped. */
        if (i != n_ready_tdcp_new) {
          memcpy(nm, &nav_meas_tdcp[i], sizeof(*nm));
        }

        double doppler = 0.0;
        if (0 != (nm->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
          doppler = nm->raw_measured_doppler;
        }

        nm->raw_carrier_phase += t_err * doppler;
        /* Note, the pseudorange correction has opposite sign because Doppler
         * has the opposite sign compared to the pseudorange rate. */
        nm->raw_pseudorange -= t_err * doppler *
                               sid_to_lambda(nm->sid);

        /* Correct the observations for the receiver clock error. */
        nm->raw_carrier_phase += current_fix.clock_offset *
                                      GPS_C / sid_to_lambda(nm->sid);
        nm->raw_measured_doppler += current_fix.clock_bias *
                                    GPS_C / sid_to_lambda(nm->sid);
        nm->raw_pseudorange -= current_fix.clock_offset * GPS_C;
        nm->raw_computed_doppler += computed_clock_rate *
                                      GPS_C / sid_to_lambda(nm->sid);

        /* Also apply the time correction to the time of transmission so the
         * satellite positions can be calculated for the correct time. */
        nm->tot.tow += t_err;
        normalize_gps_time(&nm->tot);

        ephemeris_t *e = NULL;

        /* Find the original index of this measurement in order to point to
         * the correct ephemeris. (Do not load it again from NDB because it may
         * have changed meanwhile.) */
        for (u8 j = 0; j < n_ready; j++) {
          if (sid_is_equal(nm->sid, meas[j].sid)) {
            e = &e_meas[j];
            break;
          }
        }

        if (e == NULL || 1 != ephemeris_valid(e, &nm->tot)) {
          continue;
        }

        /* Recompute satellite position, velocity and clock errors */
        if (0 != calc_sat_state(e,
                                &nm->tot,
                                nm->sat_pos,
                                nm->sat_vel,
                                &nm->sat_clock_err,
                                &nm->sat_clock_err_rate,
                                &nm->IODC,
                                &nm->IODE)) {
          continue;
        }

        n_ready_tdcp_new++;
      }

      /* Update n_ready_tdcp. */
      n_ready_tdcp = n_ready_tdcp_new;

      /* Output observations only every obs_output_divisor times, taking
       * care to ensure that the observations are aligned. */
      /* Also only output observations once our receiver clock is
       * correctly set. */
      double t_check = new_obs_time.tow * (soln_freq / obs_output_divisor);
      if (1 &&
          time_quality == TIME_FINE &&
          fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {

        /* Send the observations. */
        me_send_all(n_ready_tdcp, nav_meas_tdcp, e_meas, &new_obs_time);
      }
    }

    /* Calculate the receiver clock error and if >1ms perform a clock jump */
    double rx_err = gpsdifftime(&rec_time, &current_fix.time);
    log_debug("RX clock offset = %f", rx_err);

    if (fabs(rx_err) >= 1e-3) {
      log_info("Receiver clock offset larger than 1 ms, applying millisecond jump");
      /* round the time adjustment to even milliseconds */
      double dt = round(rx_err * 1000.0) / 1000.0;
      /* adjust the RX to GPS time conversion */
      adjust_time_fine(dt);
      /* adjust all the carrier phase offsets */
      /* note that the adjustment is always in even cycles because millisecond
       * breaks up exactly into carrier cycles
       * TODO: verify this holds for GLONASS as well */
      tracking_channel_carrier_phase_offsets_adjust(dt);
      /* adjust the stored CP measurements so that next TDCP is correct */
      for (u8 i = 0; i < n_ready_old; i++) {
        nav_meas_old[i].raw_carrier_phase += dt *
            sid_to_carr_freq(nav_meas_old[i].sid);
      }
      clock_offset_previous -= dt;
    }

    /* Calculate the correction to the current deadline by converting nap count
     * difference to seconds, we convert to ms to adjust deadline later */
    double dt = delta_tc * RX_DT_NOMINAL + current_fix.clock_offset / soln_freq;

    /* Limit dt to twice the max soln rate */
    double max_deadline = ((1.0 / soln_freq) * 2.0);
    if (fabs(dt) > max_deadline) {
      dt = (dt > 0.0) ? max_deadline : -1.0 * max_deadline;
    }

    /* Reset timer period with the count that we will estimate will being
     * us up to the next solution time. */
    deadline += round(dt * CH_CFG_ST_FREQUENCY);
  }
}


soln_stats_t solution_last_stats_get(void)
{
  return last_stats;
}

soln_pvt_stats_t solution_last_pvt_stats_get(void)
{
  return last_pvt_stats;
}



void me_calc_pvt_setup()
{

  SETTING("solution", "soln_freq", soln_freq, TYPE_FLOAT);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);
  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  static msg_t obs_mailbox_buff[OBS_N_BUFF];

  chMBObjectInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);

  chPoolObjectInit(&obs_buff_pool, sizeof(me_msg_obs_t), NULL);

  static me_msg_obs_t obs_buff[OBS_N_BUFF] _CCM;

  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  /* Start solution thread */
  chThdCreateStatic(wa_me_calc_pvt_thread, sizeof(wa_me_calc_pvt_thread),
                    HIGHPRIO-2, me_calc_pvt_thread, NULL);

}
