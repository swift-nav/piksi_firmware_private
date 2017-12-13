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
#include <assert.h>
#include <stdio.h>

#include <libsbp/sbp.h>
#include <libswiftnav/cnav_msg.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/nav_meas_calc.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/troposphere.h>

#include "me_calc_pvt.h"

#include "manage.h"
#include "nmea.h"
#include "peripherals/leds.h"
#include "position.h"
#include "sbp.h"
#include "sbp_utils.h"

#include "cnav_msg_storage.h"
#include "common_calc_pvt.h"
#include "ephemeris.h"
#include "main.h"
#include "ndb.h"
#include "observation_biases_calibration.h"
#include "settings.h"
#include "shm.h"
#include "simulator.h"
#include "system_monitor.h"
#include "timing.h"

/** Mandatory flags filter for measurements */
#define MANAGE_TRACK_FLAGS_FILTER                               \
  (MANAGE_TRACK_FLAG_ACTIVE | MANAGE_TRACK_FLAG_NO_ERROR |      \
   MANAGE_TRACK_FLAG_CONFIRMED | MANAGE_TRACK_FLAG_CN0_SHORT |  \
   MANAGE_TRACK_FLAG_ELEVATION | MANAGE_TRACK_FLAG_HAS_EPHE |   \
   MANAGE_TRACK_FLAG_HEALTHY | MANAGE_TRACK_FLAG_NAV_SUITABLE | \
   MANAGE_TRACK_FLAG_TOW)
/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

#define ME_CALC_PVT_THREAD_PRIORITY (HIGHPRIO - 3)
#define ME_CALC_PVT_THREAD_STACK (64 * 1024)

memory_pool_t obs_buff_pool;
mailbox_t obs_mailbox;

double soln_freq_setting = 10.0;
u32 obs_output_divisor = 2;

s16 msg_obs_max_size = SBP_FRAMING_MAX_PAYLOAD_SIZE;

static bool disable_raim = false;

static soln_stats_t last_stats = {.signals_tracked = 0, .signals_useable = 0};

/* RFT_TODO *
 * check that Klobuchar is used in SPP solver */

/* STATIC FUNCTIONS */

static void me_post_observations(u8 n,
                                 const navigation_measurement_t _meas[],
                                 const ephemeris_t _ephem[],
                                 const gps_time_t *_t) {
  /* TODO: use a buffer from the pool from the start instead of
   * allocating nav_meas as well. Downside, if we don't end up
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
    MEMCPY_S(me_msg_obs->obs,
             sizeof(me_msg_obs->obs),
             _meas,
             n * sizeof(navigation_measurement_t));
    MEMCPY_S(me_msg_obs->ephem,
             sizeof(me_msg_obs->ephem),
             _ephem,
             n * sizeof(ephemeris_t));
  }
  if (_t != NULL) {
    me_msg_obs->obs_time = *_t;
  } else {
    me_msg_obs->obs_time.wn = WN_UNKNOWN;
    me_msg_obs->obs_time.tow = TOW_UNKNOWN;
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

static bool decimate_observations(const gps_time_t *_t) {
  /* We can use the solution setting directly here as we have no
   * later dependencies on being consistent, all we want to know
   * is should this epoch be decimated from output. */
  gps_time_t epoch =
      gps_time_round_to_epoch(_t, soln_freq_setting / obs_output_divisor);
  return fabs(gpsdifftime(_t, &epoch)) < TIME_MATCH_THRESHOLD;
}

static void me_send_all(u8 _num_obs,
                        const navigation_measurement_t _meas[],
                        const ephemeris_t _ephem[],
                        const gps_time_t *_t) {
  me_post_observations(_num_obs, _meas, _ephem, _t);
  /* Output observations only every obs_output_divisor times, taking
   * care to ensure that the observations are aligned. */
  if (decimate_observations(_t) && !simulation_enabled()) {
    send_observations(_num_obs, msg_obs_max_size, _meas, _t);
  }
  DO_EVERY(biases_message_freq_setting, send_glonass_biases());
}

static void me_send_emptyobs(void) {
  me_post_observations(0, NULL, NULL, NULL);
  /* When we don't have a time solve, we still want to decimate our
   * observation output, we can use the GPS time if we have one,
   * otherwise we'll default to using receiver time. */
  const gps_time_t _t = get_current_gps_time();
  if (decimate_observations(&_t) && !simulation_enabled()) {
    send_observations(0, msg_obs_max_size, NULL, NULL);
  }
}

/* remove the clock offset and bias from measurements */
static void remove_clock_offset(navigation_measurement_t *nm,
                                double clock_offset,
                                double clock_drift,
                                u64 ref_tc) {
  double doppler = 0.0;
  if (0 != (nm->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
    doppler = nm->raw_measured_doppler;
  }

  /* The pseudorange correction has opposite sign because Doppler has the
   * opposite sign compared to the pseudorange rate. */
  nm->raw_pseudorange -= clock_offset * doppler * sid_to_lambda(nm->sid);
  nm->raw_pseudorange -= clock_offset * GPS_C;

  /* Remove the fractional 2-ms residual FCN contribution */
  if (IS_GLO(nm->sid)) {
    nm->raw_carrier_phase += glo_2ms_fcn_residual(nm->sid, ref_tc);
  }

  /* Carrier Phase corrected by clock offset */
  nm->raw_carrier_phase -= clock_offset * doppler;
  nm->raw_carrier_phase -= clock_offset * GPS_C / sid_to_lambda(nm->sid);

  /* Use P**V**T to determine the oscillator drift which is used to adjust
   * computed doppler. */
  nm->raw_measured_doppler += clock_drift * GPS_C / sid_to_lambda(nm->sid);
  nm->raw_computed_doppler = nm->raw_measured_doppler;

  /* Also apply the time correction to the time of transmission so the
   * satellite positions can be calculated for the correct time. */
  nm->tot.tow += clock_offset;
  normalize_gps_time(&(nm->tot));
}

/** Roughly propagate and send the observations when PVT solution failed or is
 * not available. Flag all with RAIM exclusion so they do not get used
 * downstream. */
static void me_send_failed_obs(u8 _num_obs,
                               navigation_measurement_t _meas[],
                               const ephemeris_t _ephem[],
                               const gps_time_t *_t) {
  /* require at least some timing quality */
  if (TIME_PROPAGATED > get_time_quality() || !gps_time_valid(_t) ||
      _num_obs == 0) {
    me_send_emptyobs();
    return;
  }

  double clock_offset = 0;
  double clock_drift = 0;
  last_good_fix_t lgf;
  if (NDB_ERR_NONE == ndb_lgf_read(&lgf) && lgf.position_solution.valid) {
    /* estimate the current clock offset from LGF */
    double dt = gpsdifftime(_t, &lgf.position_solution.time);
    clock_offset = lgf.position_solution.clock_offset +
                   dt * lgf.position_solution.clock_drift;
    clock_drift = lgf.position_solution.clock_drift;
  }

  u64 ref_tc = rcvtime2napcount(_t);
  for (u8 i = 0; i < _num_obs; i++) {
    /* mark the measurement unusable to be on the safe side */
    /* TODO: could relax this in order to send also under-determined measurement
     * sets to Starling */
    _meas[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;

    /* propagate the measurements with the clock bias and drift from LGF */
    remove_clock_offset(&_meas[i], clock_offset, clock_drift, ref_tc);
  }

  me_post_observations(_num_obs, _meas, _ephem, _t);
  /* Output observations only every obs_output_divisor times, taking
  * care to ensure that the observations are aligned. */
  if (decimate_observations(_t) && !simulation_enabled()) {
    send_observations(_num_obs, msg_obs_max_size, _meas, _t);
  }
}

/** Update the satellite azimuth & elevation database with current angles
 * \param rcv_pos Approximate receiver position
 * \param t Approximate time
 */
static void update_sat_azel(const double rcv_pos[3], const gps_time_t t) {
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
    if ((NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res) &&
        ephemeris_valid(&ephemeris, &t) &&
        calc_sat_az_el(&ephemeris, &t, rcv_pos, &az, &el, false) >= 0) {
      sv_azel_degrees_set(sid, round(az * R2D), round(el * R2D), nap_count);
      log_debug_sid(sid, "Updated elevation from ephemeris %.1f", el * R2D);

      /* else try to fetch almanac and use it if it is valid */
    } else if (NDB_ERR_NONE == ndb_almanac_read(sid, &almanac) &&
               almanac_valid(&almanac, &t) &&
               calc_sat_az_el_almanac(&almanac, &t, rcv_pos, &az, &el) >= 0) {
      sv_azel_degrees_set(sid, round(az * R2D), round(el * R2D), nap_count);
      log_debug_sid(sid, "Updated elevation from almanac %.1f", el * R2D);
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

/**
 * Collects channel measurements, ephemerides and auxiliary data.
 *
 * \param[in]  rec_tc    Timestamp [ticks].
 * \param[out] meas      Destination measurement array.
 * \param[out] in_view   Destination in_view array.
 * \param[out] ephe      Destination ephemeris array.
 * \param[out] pn_ready  Destination for measurement array size.
 * \param[out] pn_inview Destination for in-view array size.
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
                                 u8 *pn_total) {
  u8 n_collected = 0;
  u8 n_inview = 0;
  u8 n_active = 0;
  bool any_gps = false;

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    u32 flags = 0; /* Channel flags accumulator */
    /* Load measurements from the tracking channel and ephemeris from NDB */
    flags = get_tracking_channel_meas(
        i, rec_tc, &meas[n_collected], &ephe[n_collected]);

    if (0 != (flags & TRACKER_FLAG_ACTIVE) &&
        0 != (flags & TRACKER_FLAG_CONFIRMED) &&
        0 == (flags & TRACKER_FLAG_ERROR) &&
        0 == (flags & TRACKER_FLAG_MASKED)) {
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
        any_gps |= IS_GPS(meas[n_collected].sid);
        n_collected++;
      }
    }
  }

  /* require that the measurements contain at least one valid GPS measurement
   * before returning anything */
  if (any_gps) {
    *pn_ready = n_collected;
  }

  *pn_inview = n_inview;
  *pn_total = n_active;
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
  if (fabs(nav_meas->pseudorange - current_fix->clock_offset * GPS_C -
           vector_norm(3, geometric_range)) > RAIM_DROP_CHANNEL_THRESHOLD_M) {
    /* mark channel for dropping */
    tracking_channel_set_raim_flag(nav_meas->sid);
    /* clear the ephemeris for this signal */
    ndb_ephemeris_erase(nav_meas->sid);
  }
}

static void me_calc_pvt_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("me_calc_pvt");

  last_good_fix_t lgf;
  ndb_lgf_read(&lgf);

  piksi_systime_t next_epoch;
  piksi_systime_get(&next_epoch);
  piksi_systime_inc_us(&next_epoch, SECS_US / soln_freq_setting);

  while (TRUE) {
    /* read current value of soln_freq into a local variable that does not
     * change during this loop iteration */
    chSysLock();
    double soln_freq = soln_freq_setting;
    chSysUnlock();

    /* sleep until next epoch, and update the deadline */
    me_thd_sleep(&next_epoch, SECS_US / soln_freq);
    watchdog_notify(WD_NOTIFY_ME_CALC_PVT);

    if (TIME_UNKNOWN != get_time_quality() && lgf.position_solution.valid &&
        lgf.position_quality >= POSITION_GUESS) {
      /* Update the satellite elevation angles so that they stay current
       * (currently once every 30 seconds) */
      DO_EVERY((u32)soln_freq * MAX_AZ_EL_AGE_SEC / 2,
               update_sat_azel(lgf.position_solution.pos_ecef,
                               lgf.position_solution.time));
    }

    /* Take the current nap count as the reception time*/
    u64 current_tc = nap_timing_count();

    /* The desired solution NAP counter and epoch */
    u64 epoch_tc = current_tc;

    gps_time_t epoch_time = GPS_TIME_UNKNOWN;

    /* If gps time is available, round the reception time to the nearest
     * solution epoch */
    if (TIME_PROPAGATED <= get_time_quality()) {
      /* If we have timing then we can calculate the relationship between
       * receiver time and GPS time and hence provide the pseudorange
       * calculation with the local GPS time of reception. */
      gps_time_t rcv_time = napcount2rcvtime(current_tc);

      if (gpsdifftime(&rcv_time, &lgf.position_solution.time) >
          MAX_TIME_PROPAGATED_S) {
        if (lgf.position_quality > POSITION_STATIC) {
          lgf.position_quality = POSITION_STATIC;
        }
      }

      /* round current estimated GPS time to the epoch boundary */
      epoch_time = gps_time_round_to_epoch(&rcv_time, soln_freq);

      if (gpsdifftime(&epoch_time, &lgf.position_solution.time) <= 0) {
        /* We are already past the next solution epoch, can happen when solution
         * frequency changes */
        log_info(
            "Next epoch (wn %d tow %f) is in the past wrt (wn %d tow %f), "
            "skipping",
            epoch_time.wn,
            epoch_time.tow,
            lgf.position_solution.time.wn,
            lgf.position_solution.time.tow);
        continue;
      }

      /* get NAP at the epoch with a round GPS time */
      epoch_tc = (u64)round(rcvtime2napcount(&epoch_time));
    }

    /* Collect measurements from trackers, load ephemerides and compute flags.
     * Reference the measurements to the solution epoch. */
    u8 n_ready = 0;
    u8 n_inview = 0;
    u8 n_total = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    channel_measurement_t in_view[MAX_CHANNELS];
    static ephemeris_t e_meas[MAX_CHANNELS];

    collect_measurements(
        epoch_tc, meas, in_view, e_meas, &n_ready, &n_inview, &n_total);

    nmea_send_gsv(n_inview, in_view);

    log_debug("Selected %" PRIu8 " measurement(s) out of %" PRIu8
              " in view "
              " (total=%" PRIu8 ")",
              n_ready,
              n_inview,
              n_total);

    /* Update stats */
    last_stats.signals_tracked = n_total;
    last_stats.signals_useable = n_ready;

    if (n_ready == 0) {
      me_send_emptyobs();
      continue;
    }

    cnav_msg_t cnav_30[MAX_CHANNELS];
    const cnav_msg_type_30_t *p_cnav_30[MAX_CHANNELS];
    for (u8 i = 0; i < n_ready; i++) {
      p_cnav_30[i] = cnav_msg_get(meas[i].sid, CNAV_MSG_TYPE_30, &cnav_30[i])
                         ? &cnav_30[i].data.type_30
                         : NULL;
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

    /* If `epoch_time` is invalid (i.e. GPS_TIME_UNKNOWN), then pass in NULL
     * instead of `epoch_time`. This will result in valid pseudoranges but with
     * a large and arbitrary receiver clock error. We will discard these
     * measurements in any case after getting the PVT solution and initializing
     * clock. */
    s8 nm_ret = calc_navigation_measurement(
        n_ready,
        p_meas,
        p_nav_meas,
        gps_time_valid(&epoch_time) ? &epoch_time : NULL);

    if (nm_ret != 0) {
      log_error("calc_navigation_measurement() returned an error");
      me_send_emptyobs();
      continue;
    }

    s8 sc_ret = calc_sat_clock_corrections(n_ready, p_nav_meas, p_e_meas);

    if (sc_ret != 0) {
      log_error("calc_sat_clock_correction() returned an error");
      me_send_emptyobs();
      continue;
    }

    calc_isc(n_ready, p_nav_meas, p_cnav_30);

    apply_isc_table(n_ready, p_nav_meas);

    gnss_sid_set_t codes;
    sid_set_init(&codes);
    for (u8 i = 0; i < n_ready; i++) {
      sid_set_add(&codes, nav_meas[i].sid);
    }

    /* check if we have a solution, if yes calc iono and tropo correction */
    if (lgf.position_quality >= POSITION_GUESS) {
      ionosphere_t i_params;
      ionosphere_t *p_i_params = &i_params;
      /* get iono parameters if available */
      if (ndb_iono_corr_read(p_i_params) != NDB_ERR_NONE) {
        p_i_params = NULL;
      }
      calc_iono_tropo(n_ready,
                      nav_meas,
                      lgf.position_solution.pos_ecef,
                      lgf.position_solution.pos_llh,
                      p_i_params);
    }

    if (sid_set_get_sat_count(&codes) < 4) {
      /* Not enough sats to compute PVT, send them as unusable */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &epoch_time);
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
                          disable_raim,
                          false,
                          &current_fix,
                          &dops,
                          &raim_removed_sids);
    if (pvt_ret < 0 || (lgf.position_quality == POSITION_FIX &&
                        gate_covariance(&current_fix))) {
      if (pvt_ret < 0) {
        /* An error occurred with calc_PVT! */
        /* pvt_err_msg defined in libswiftnav/pvt.c */
        DO_EVERY((u32)soln_freq,
                 log_warn("PVT solver: %s (code %d)",
                          pvt_err_msg[-pvt_ret - 1],
                          pvt_ret););
      }

      /* If we can't report a SPP position, something is wrong and no point
       * continuing to process this epoch - mark observations unusable but send
       * them out to enable debugging. */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &epoch_time);

      /* If we already had a good fix, degrade its quality to STATIC */
      if (lgf.position_quality > POSITION_STATIC) {
        lgf.position_quality = POSITION_STATIC;
      }

      continue;
    }

    /* If we have a success RAIM repair, mark the removed observations as
       invalid, and ask tracker to drop the channels (if needed). */
    if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
      for (u8 i = 0; i < n_ready; i++) {
        if (sid_set_contains(&raim_removed_sids, nav_meas[i].sid)) {
          log_debug_sid(nav_meas[i].sid,
                        "RAIM repair, setting observation invalid.");
          nav_meas[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;

          /* Check how large the outlier roughly is, and if it is a gross one,
           * drop the channel and delete the possibly corrupt ephemeris */
          drop_gross_outlier(&nav_meas[i], &current_fix);
        }
      }
    }

    /* Update global position solution state. */
    lgf.position_solution = current_fix;
    lgf.position_quality = POSITION_FIX;
    ndb_lgf_store(&lgf);

    time_quality_t old_time_quality = get_time_quality();

    /* Update the relationship between the solved GPS time and NAP count tc.*/
    update_time(epoch_tc, &current_fix);

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

      me_send_failed_obs(n_ready, nav_meas, e_meas, &epoch_time);
      continue;
    }

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(current_fix.clock_offset) < OBS_PROPAGATION_LIMIT) {
      log_debug("clk_offset %.3e clk_drift %.3e",
                current_fix.clock_offset,
                current_fix.clock_drift);

      double clock_offset = current_fix.clock_offset;
      double clock_drift = current_fix.clock_drift;

      for (u8 i = 0; i < n_ready; i++) {
        navigation_measurement_t *nm = &nav_meas[i];

        /* remove clock offset from the measurement */
        remove_clock_offset(nm, clock_offset, clock_drift, epoch_tc);

        /* Recompute satellite position, velocity and clock errors */
        /* NOTE: calc_sat_state changes `tot` */
        if (0 != calc_sat_state(&e_meas[i],
                                &(nm->tot),
                                nm->sat_pos,
                                nm->sat_vel,
                                &(nm->sat_clock_err),
                                &(nm->sat_clock_err_rate),
                                &(nm->IODC),
                                &(nm->IODE))) {
          log_error_sid(nm->sid, "Recomputing sat state failed");
          continue;
        }
      }

      /* Send the observations. */
      me_send_all(n_ready, nav_meas, e_meas, &epoch_time);
    } else {
      log_warn("clock_offset %.9lf greater than OBS_PROPAGATION_LIMIT",
               (current_fix.clock_offset));
      /* Send the observations, but marked unusable */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &epoch_time);
    }

    if (fabs(current_fix.clock_offset) > MAX_CLOCK_ERROR_S) {
      /* Note we should not enter here except in very exceptional circumstances,
       * like time solved grossly wrong on the first fix. */

      /* round the time adjustment to even milliseconds */
      double dt =
          round(current_fix.clock_offset * (SECS_MS / 2)) / (SECS_MS / 2);

      log_info("Receiver clock offset larger than %g ms, applying %g ms jump",
               MAX_CLOCK_ERROR_S * SECS_MS,
               dt * SECS_MS);

      /* adjust all the carrier phase offsets */
      /* note that the adjustment is always in even cycles because millisecond
       * breaks up exactly into carrier cycles */
      tracking_channel_carrier_phase_offsets_adjust(dt);

      /* adjust the RX to GPS time conversion */
      adjust_rcvtime_offset(dt);
    }

    /* */
    double delta_tc = -((s64)current_tc - (s64)epoch_tc);

    /* The difference between the current nap count and the nap count we
     * would have wanted the observations at is the amount we want to
     * adjust our deadline by at the end of the solution */
    double dt = delta_tc * RX_DT_NOMINAL + current_fix.clock_offset / soln_freq;

    /* Limit dt to twice the max soln rate */
    double max_deadline = ((1.0 / soln_freq) * 2.0);
    if (fabs(dt) > max_deadline) {
      dt = (dt > 0.0) ? max_deadline : -1.0 * max_deadline;
    }

    /* Reset timer period with the count that we will estimate will being
     * us up to the next solution time. dt as microseconds. */
    if (0 < dt) {
      piksi_systime_inc_us(&next_epoch, round(dt * SECS_US));
    } else if (0 > dt) {
      piksi_systime_dec_us(&next_epoch, round(-dt * SECS_US));
    }
  }
}

soln_stats_t solution_last_stats_get(void) { return last_stats; }

void me_calc_pvt_setup() {
  SETTING("solution", "soln_freq", soln_freq_setting, TYPE_FLOAT);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);
  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  static msg_t obs_mailbox_buff[OBS_N_BUFF];

  chMBObjectInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);

  chPoolObjectInit(&obs_buff_pool, sizeof(me_msg_obs_t), NULL);

  static me_msg_obs_t obs_buff[OBS_N_BUFF];

  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  /* Start solution thread */
  chThdCreateStatic(wa_me_calc_pvt_thread,
                    sizeof(wa_me_calc_pvt_thread),
                    ME_CALC_PVT_THREAD_PRIORITY,
                    me_calc_pvt_thread,
                    NULL);
}
