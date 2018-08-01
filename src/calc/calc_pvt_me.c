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
#include <float.h>
#include <stdio.h>

#include <libsbp/sbp.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/correct_iono_tropo.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/single_epoch_solver.h>
#include <libswiftnav/troposphere.h>
#include <starling/starling.h>
#include <starling/starling_platform.h>

#include "board/nap/track_channel.h"
#include "calc_base_obs.h"
#include "calc_nav_meas.h"
#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "main.h"
#include "manage.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "obs_bias/obs_bias.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "simulator.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"
#include "track/track_sid_db.h"
#include "track/track_utils.h"

/** Mandatory flags filter for measurements */
#define MANAGE_TRACK_FLAGS_FILTER                               \
  (MANAGE_TRACK_FLAG_ACTIVE | MANAGE_TRACK_FLAG_NO_ERROR |      \
   MANAGE_TRACK_FLAG_CONFIRMED | MANAGE_TRACK_FLAG_CN0_SHORT |  \
   MANAGE_TRACK_FLAG_ELEVATION | MANAGE_TRACK_FLAG_HAS_EPHE |   \
   MANAGE_TRACK_FLAG_HEALTHY | MANAGE_TRACK_FLAG_NAV_SUITABLE | \
   MANAGE_TRACK_FLAG_TOW)
/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

/* Maximum time to maintain POSITION_FIX after last successful solution */
#define POSITION_FIX_TIMEOUT_S 60

#define ME_CALC_PVT_THREAD_PRIORITY (HIGHPRIO - 3)
#define ME_CALC_PVT_THREAD_STACK (64 * 1024)

/* Limits the sets of possible solution frequencies (in increasing order) */
static const double valid_soln_freqs_hz[] = {1.0, 2.0, 4.0, 5.0, 10.0};

#define SOLN_FREQ_SETTING_MIN (valid_soln_freqs_hz[0])
#define SOLN_FREQ_SETTING_MAX \
  (valid_soln_freqs_hz[ARRAY_SIZE(valid_soln_freqs_hz) - 1])

double soln_freq_setting = 10.0;
u32 obs_output_divisor = 10;

s16 msg_obs_max_size = SBP_FRAMING_MAX_PAYLOAD_SIZE;

static soln_stats_t last_stats = {.signals_tracked = 0, .signals_useable = 0};

/* RFT_TODO *
 * check that Klobuchar is used in SPP solver */

/* STATIC FUNCTIONS */

static void me_post_ephemerides(u8 n, const ephemeris_t ephemerides[]) {
  for (u8 i = 0; i < n; ++i) {
    ephemeris_t *p_eph = platform_mailbox_item_alloc(MB_ID_EPHEMERIS);
    if (NULL == p_eph) {
      log_error("ME: All ephemeris objects in use, unable to allocate.");
      break;
    } else {
      *p_eph = ephemerides[i];  
      errno_t error = platform_mailbox_post(MB_ID_EPHEMERIS, p_eph, MB_NONBLOCKING);
      if (error) {
        log_error("ME: Mailbox should have space for ephemeris.");
        platform_mailbox_item_free(MB_ID_EPHEMERIS, p_eph);
        break;
      }
    }
  }
}

static void me_post_observations(u8 n,
                                 const navigation_measurement_t _meas[],
                                 const ephemeris_t _ephem[],
                                 const gps_time_t *_t) {
  /* Post all ephemerides prior to posting any measurements. This way
   * when Starling engine wakes on receiving measurements, the ephemerides
   * are guaranteed to already be there. */
  me_post_ephemerides(n, _ephem);

  /* TODO: use a buffer from the pool from the start instead of
   * allocating nav_meas as well. Downside, if we don't end up
   * pushing the message into the mailbox then we just wasted an
   * observation from the mailbox for no good reason. */

  me_msg_obs_t *me_msg_obs = platform_mailbox_item_alloc(MB_ID_ME_OBS);
  if (NULL == me_msg_obs) {
    log_error("ME: Could not allocate pool for obs!");
    return;
  }

  me_msg_obs->size = n;
  if (n) {
    MEMCPY_S(me_msg_obs->obs,
             sizeof(me_msg_obs->obs),
             _meas,
             n * sizeof(navigation_measurement_t));
  }
  if (_t != NULL) {
    me_msg_obs->obs_time = *_t;
  } else {
    me_msg_obs->obs_time.wn = WN_UNKNOWN;
    me_msg_obs->obs_time.tow = TOW_UNKNOWN;
  }

  errno_t ret = platform_mailbox_post(MB_ID_ME_OBS, me_msg_obs, MB_NONBLOCKING);
  if (ret != 0) {
    /* We could grab another item from the mailbox, discard it and then
     * post our obs again but if the size of the mailbox and the pool
     * are equal then we should have already handled the case where the
     * mailbox is full when we handled the case that the pool was full.
     * */
    log_error("ME: Mailbox should have space for obs!");
    platform_mailbox_item_free(MB_ID_ME_OBS, me_msg_obs);
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
  gps_time_t current_time = get_current_time();
  if (!gps_time_valid(&current_time)) {
    /* gps time not available, so fill the tow with seconds from restart */
    current_time.tow = nap_timing_count() * RX_DT_NOMINAL;
    current_time = gps_time_round_to_epoch(&current_time, soln_freq_setting);
  }
  if (decimate_observations(&current_time) && !simulation_enabled()) {
    send_observations(0, msg_obs_max_size, NULL, NULL);
  }
}

/* remove the effect of local clock offset and drift from measurements */
static void remove_clock_offset(navigation_measurement_t *nm,
                                double clock_offset,
                                double clock_drift,
                                u64 current_tc) {
  assert(0 != (nm->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID));

  /* Adjust Doppler with smoothed oscillator drift. */
  nm->raw_measured_doppler += clock_drift * GPS_C / sid_to_lambda(nm->sid);
  nm->raw_computed_doppler = nm->raw_measured_doppler;

  /* Range correction caused by clock offset */
  double corr_cycles = clock_offset * nm->raw_measured_doppler;
  nm->raw_pseudorange -= corr_cycles * sid_to_lambda(nm->sid);
  nm->raw_carrier_phase -= corr_cycles;

  /* Compensate for NAP counter drift since cpo computation */
  double cpo_drift = subsecond_cpo_correction(current_tc);
  nm->raw_carrier_phase += cpo_drift * sid_to_carr_freq(nm->sid);

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

  /* offset assumed steered to zero */
  double clock_offset = 0;

  u64 ref_tc = gpstime2napcount(_t);

  /* get the estimated clock drift value */
  double clock_drift = get_clock_drift();

  for (u8 i = 0; i < _num_obs; i++) {
    /* mark the measurement unusable to be on the safe side */
    /* TODO: could relax this in order to send also under-determined measurement
     * sets to Starling */
    _meas[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;

    /* propagate the measurements with the smoothed drift value */
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
        0 == (flags & TRACKER_FLAG_DROP_CHANNEL) &&
        0 == (flags & TRACKER_FLAG_MASKED)) {
      /* Tracking channel is active & not masked */
      n_active++;

      if (0 == (flags & TRACKER_FLAG_XCORR_SUSPECT)) {
        /* Tracking channel is not XCORR suspect so it's an actual SV in view */
        in_view[n_inview++] = meas[n_collected];
      }

      chan_meas_flags_t meas_flags = meas[n_collected].flags;

      if (0 != (flags & TRACKER_FLAG_NAV_SUITABLE) &&
          0 != (flags & TRACKER_FLAG_ELEVATION) &&
          0 != (flags & TRACKER_FLAG_TOW_VALID) &&
          0 != (flags & TRACKER_FLAG_HAS_EPHE) &&
          0 != (flags & TRACKER_FLAG_CN0_SHORT) &&
          0 != (meas_flags & CHAN_MEAS_FLAG_CODE_VALID) &&
          0 != (meas_flags & CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
        /* Tracking channel is suitable for solution calculation */
        any_gps |= IS_GPS(meas[n_collected].sid);
        n_collected++;
      } else {
        if (is_gal(meas[n_collected].sid.code)) {
          log_debug_sid(meas[n_collected].sid,
                        "NAV %s  ELEV %s  TOW %s  EPHE %s  CN0 %s",
                        (0 != (flags & TRACKER_FLAG_NAV_SUITABLE)) ? "Y" : "N",
                        (0 != (flags & TRACKER_FLAG_ELEVATION)) ? "Y" : "N",
                        (0 != (flags & TRACKER_FLAG_TOW_VALID)) ? "Y" : "N",
                        (0 != (flags & TRACKER_FLAG_HAS_EPHE)) ? "Y" : "N",
                        (0 != (flags & TRACKER_FLAG_CN0_SHORT)) ? "Y" : "N");
        }
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
    /* read current value of soln_freq into a local variable that does not
     * change during this loop iteration */
    chSysLock();
    double soln_freq = soln_freq_setting;
    chSysUnlock();

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
    u8 n_ready = 0;
    u8 n_inview = 0;
    u8 n_total = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    channel_measurement_t in_view[MAX_CHANNELS];
    static ephemeris_t e_meas[MAX_CHANNELS];

    /* Collect measurements propagated to the current NAP tick */
    collect_measurements(
        current_tc, meas, in_view, e_meas, &n_ready, &n_inview, &n_total);

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

    /* GPS time is invalid on the first fix, form a coarse estimate from the
     * first pseudorange measurement */
    if (!gps_time_valid(&current_time)) {
      current_time.tow =
          (double)meas[0].time_of_week_ms / SECS_MS + GPS_NOMINAL_RANGE / GPS_C;
      normalize_gps_time(&current_time);
      gps_time_match_weeks(&current_time, &e_meas[0].toe);
    }

    /* Create navigation measurements from the channel measurements */
    s8 nm_ret =
        calc_navigation_measurement(n_ready, p_meas, p_nav_meas, &current_time);

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

    apply_gps_cnav_isc(n_ready, p_nav_meas, p_cnav_30, p_e_meas);
    apply_isc_table(n_ready, p_nav_meas);

    gnss_sid_set_t codes;
    sid_set_init(&codes);
    for (u8 i = 0; i < n_ready; i++) {
      sid_set_add(&codes, nav_meas[i].sid);
    }

    /* check if we have a solution, if yes calc iono and tropo correction */
    if (lgf.position_quality >= POSITION_GUESS) {
      ionosphere_t i_params;
      /* get iono parameters if available, otherwise use default ones */
      if (ndb_iono_corr_read(&i_params) != NDB_ERR_NONE) {
        i_params = DEFAULT_IONO_PARAMS;
      }
      correct_tropo(lgf.position_solution.pos_ecef, n_ready, nav_meas);
      correct_iono(
          lgf.position_solution.pos_ecef, &i_params, n_ready, nav_meas);
    }

    if (sid_set_get_sat_count(&codes) < 4) {
      /* Not enough sats to compute PVT, send them as unusable */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &current_time);
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

      /* If we can't report a SPP position, something is wrong and no point
       * continuing to process this epoch - mark observations unusable but send
       * them out to enable debugging. */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &current_time);

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

      me_send_failed_obs(n_ready, nav_meas, e_meas, &current_time);
      /* adjust the deadline of the next fix to land on output epoch */
      piksi_systime_add_us(&next_epoch, round(output_offset * SECS_US));
      continue;
    }

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(output_offset) < OBS_PROPAGATION_LIMIT) {
      log_debug(
          "clk_offset %.4e, output offset %.4e, clk_drift %.3e, "
          "smoothed_drift %.3e",
          current_fix.clock_offset,
          output_offset,
          current_fix.clock_drift,
          smoothed_drift);

      for (u8 i = 0; i < n_ready; i++) {
        navigation_measurement_t *nm = &nav_meas[i];

        /* remove clock offset from the measurement */
        remove_clock_offset(nm, output_offset, smoothed_drift, current_tc);

        /* Recompute satellite position, velocity and clock errors */
        /* NOTE: calc_sat_state changes `tot` */
        if (0 != calc_sat_state(&e_meas[i],
                                &(nm->tot),
                                nm->sat_pos,
                                nm->sat_vel,
                                nm->sat_acc,
                                &(nm->sat_clock_err),
                                &(nm->sat_clock_err_rate),
                                &(nm->IODC),
                                &(nm->IODE))) {
          log_error_sid(nm->sid, "Recomputing sat state failed");
          continue;
        }
      }

      /* Send the observations. */
      me_send_all(n_ready, nav_meas, e_meas, &output_time);
    } else {
      log_info("clock_offset %.3f s greater than OBS_PROPAGATION_LIMIT",
               output_offset);
      /* Send the observations, but marked unusable */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &current_time);
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

soln_stats_t solution_last_stats_get(void) { return last_stats; }

static double validate_soln_freq(double requested_freq_hz) {
  /* Loop over valid frequencies, start from highest frequency */
  for (s8 i = ARRAY_SIZE(valid_soln_freqs_hz) - 1; i >= 0; --i) {
    double freq = valid_soln_freqs_hz[i];

    /* Equal */
    if (fabs(freq - requested_freq_hz) < DBL_EPSILON) {
      log_info("Solution frequency validated to %.2f Hz", freq);
      return freq;
    }

    /* No match available, floor to closest valid frequency */
    if (freq < requested_freq_hz) {
      log_info("Solution frequency floored to %.2f Hz", freq);
      return freq;
    }
  }

  log_warn(
      "Input solution frequency %.2f Hz couldn't be validated, "
      "defaulting to %.2f Hz",
      requested_freq_hz,
      SOLN_FREQ_SETTING_MIN);
  return SOLN_FREQ_SETTING_MIN;
}

/* Update the solution frequency used by the ME and by Starling. */
static bool soln_freq_setting_notify(struct setting *s, const char *val) {
  double old_value = soln_freq_setting;
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  /* Certain values are disallowed. */
  if (soln_freq_setting < (SOLN_FREQ_SETTING_MIN - DBL_EPSILON) ||
      soln_freq_setting > (SOLN_FREQ_SETTING_MAX + DBL_EPSILON)) {
    log_warn(
        "Solution frequency setting outside acceptable range: [%.2f, %.2f] Hz. "
        "Reverting to previous value: %.2f Hz.",
        0.0,
        SOLN_FREQ_SETTING_MAX,
        old_value);
    soln_freq_setting = old_value;
    return false;
  }
  soln_freq_setting = validate_soln_freq(soln_freq_setting);
  starling_set_solution_frequency(soln_freq_setting);
  return true;
}

void me_calc_pvt_setup() {
  SETTING_NOTIFY("solution",
                 "soln_freq",
                 soln_freq_setting,
                 TYPE_FLOAT,
                 soln_freq_setting_notify);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);
  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  /* Start solution thread */
  chThdCreateStatic(wa_me_calc_pvt_thread,
                    sizeof(wa_me_calc_pvt_thread),
                    ME_CALC_PVT_THREAD_PRIORITY,
                    me_calc_pvt_thread,
                    NULL);
}
