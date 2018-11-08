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
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include <libsbp/sbp.h>
#include <starling/integration/starling_input_bridge.h>
#include <starling/observation.h>
#include <starling/platform/starling_platform.h>
#include <starling/pvt_engine/firmware_binding.h>
#include <starling/starling.h>
#include <swiftnav/constants.h>
#include <swiftnav/correct_iono_tropo.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/sid_set.h>
#include <swiftnav/single_epoch_solver.h>
#include <swiftnav/troposphere.h>

#include "acq/manage.h"
#include "board/nap/track_channel.h"
#include "calc_nav_meas.h"
#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "main/main.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "obs_bias/obs_bias.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "simulator/simulator.h"
#include "starling_integration.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"
#include "track/track_sid_db.h"
#include "track/track_utils.h"
#include "utils/signal_db/signal_db.h"

/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

/* Maximum time to maintain POSITION_FIX after last successful solution */
#define POSITION_FIX_TIMEOUT_S 60

#define ME_CALC_PVT_THREAD_PRIORITY (HIGHPRIO - 3)
#define ME_CALC_PVT_THREAD_STACK (3 * 10 * 1024)

/* Limits the sets of possible solution frequencies (in increasing order) */
static const double valid_soln_freqs_hz[] = {1.0, 2.0, 4.0, 5.0, 10.0, 20.0};

#define SOLN_FREQ_SETTING_MIN (valid_soln_freqs_hz[0])
#define SOLN_FREQ_SETTING_MAX \
  (valid_soln_freqs_hz[ARRAY_SIZE(valid_soln_freqs_hz) - 1])

/* Maximum interval for computing ME PVT solution */
#define ME_PVT_INTERVAL_S 1.0

double soln_freq_setting = 10.0;
u32 obs_output_divisor = 10;

s16 msg_obs_max_size = SBP_FRAMING_MAX_PAYLOAD_SIZE;

static soln_stats_t last_stats = {.signals_tracked = 0, .signals_useable = 0};

/* STATIC FUNCTIONS */

/******************************************************************************/
static void me_post_observations(const obs_array_t *obs_array,
                                 const ephemeris_t _ephem[]) {
  assert(NULL != obs_array);
  int ret = starling_send_ephemerides(_ephem, obs_array->n);
  if (STARLING_SEND_OK != ret) {
    log_error("ME: Unable to send ephemeris array.");
  }

  ret = starling_send_rover_obs(obs_array);
  if (STARLING_SEND_OK != ret) {
    log_error("ME: Unable to send observations.");
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

static void me_send_all(const obs_array_t *obs_array,
                        const ephemeris_t _ephem[]) {
  me_post_observations(obs_array, _ephem);
  /* Output observations only every obs_output_divisor times, taking
   * care to ensure that the observations are aligned. */
  if (decimate_observations(&obs_array->t) && !simulation_enabled()) {
    send_observations(obs_array, msg_obs_max_size);
  }
  DO_EVERY(biases_message_freq_setting, send_glonass_biases());
}

static void me_send_emptyobs(void) {
  obs_array_t obs_array;
  obs_array.n = 0;
  obs_array.t = GPS_TIME_UNKNOWN;
  me_post_observations(&obs_array, NULL);
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
    send_observations(NULL, msg_obs_max_size);
  }
}

/* remove the effect of local clock offset and drift from measurements */
static void remove_clock_offset(obs_array_t *obs_array,
                                const gps_time_t *output_time,
                                double clock_drift,
                                u64 current_tc) {
  /* amount of clock offset to remove */
  double clock_offset = gpsdifftime(output_time, &obs_array->t);

  for (u8 i = 0; i < obs_array->n; i++) {
    starling_obs_t *obs = &obs_array->observations[i];
    assert(0 != (obs->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID));

    /* Adjust measured Doppler with smoothed oscillator drift. */
    obs->doppler += clock_drift * GPS_C / sid_to_lambda(obs->sid);

    /* Range correction caused by clock offset */
    double corr_cycles = clock_offset * obs->doppler;
    obs->pseudorange -= corr_cycles * sid_to_lambda(obs->sid);
    obs->carrier_phase -= corr_cycles;

    /* Compensate for NAP counter drift since cpo computation */
    double cpo_drift = subsecond_cpo_correction(current_tc);
    obs->carrier_phase += cpo_drift * sid_to_carr_freq(obs->sid);

    /* Also apply the time correction to the time of transmission so the
     * satellite positions can be calculated for the correct time. */
    obs->tot.tow += clock_offset;
    normalize_gps_time(&(obs->tot));
  }
  /* update TOR of the observation set */
  obs_array->t = *output_time;
}

/** Roughly propagate and send the observations when PVT solution failed or is
 * not available. Flag all with RAIM exclusion so they do not get used
 * downstream. */
static void me_send_failed_obs(obs_array_t *obs_array,
                               const ephemeris_t _ephem[]) {
  /* require at least some timing quality */
  if (TIME_PROPAGATED > get_time_quality() || !gps_time_valid(&obs_array->t) ||
      obs_array->n == 0) {
    me_send_emptyobs();
    return;
  }

  u64 ref_tc = gpstime2napcount(&obs_array->t);

  /* get the estimated clock drift value */
  double clock_drift = get_clock_drift();

  /* remove just the smoothed drift from observations */
  remove_clock_offset(obs_array, &obs_array->t, clock_drift, ref_tc);

  for (u8 i = 0; i < obs_array->n; i++) {
    /* mark the measurement unusable to be on the safe side */
    /* TODO: could relax this in order to send also under-determined measurement
     * sets to Starling */
    obs_array->observations[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;
  }

  me_post_observations(obs_array, _ephem);
  /* Output observations only every obs_output_divisor times, taking
   * care to ensure that the observations are aligned. */
  if (decimate_observations(&obs_array->t) && !simulation_enabled()) {
    send_observations(obs_array, msg_obs_max_size);
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
          0 != (flags & TRACKER_FLAG_CN0_USABLE) &&
          0 == (flags & TRACKER_FLAG_RECOVERY_MODE) &&
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

static THD_WORKING_AREA(wa_me_calc_pvt_thread, ME_CALC_PVT_THREAD_STACK);

static void drop_gross_outlier(const gnss_signal_t sid,
                               const double pseudorange_m,
                               const double sat_pos[],
                               const gnss_solution *current_fix) {
  /* Check how large the outlier roughly is, and if it is a gross one,
   * drop the channel and delete the possibly corrupt ephemeris */
  double geometric_range[3];
  for (u8 j = 0; j < 3; j++) {
    geometric_range[j] = sat_pos[j] - current_fix->pos_ecef[j];
  }
  double pseudorng_error =
      fabs(pseudorange_m - current_fix->clock_offset * GPS_C -
           vector_norm(3, geometric_range));

  bool generic_gross_outlier = pseudorng_error > RAIM_DROP_CHANNEL_THRESHOLD_M;
  if (generic_gross_outlier) {
    /* mark channel for dropping */
    tracker_set_raim_flag(sid);
    /* clear the ephemeris for this signal */
    ndb_ephemeris_erase(sid);
  }

  bool boc_halfchip_outlier = (CODE_GAL_E1B == sid.code) &&
                              (pseudorng_error > RAIM_DROP_E1B_THRESHOLD_M);
  if (boc_halfchip_outlier) {
    /* mark channel for dropping */
    tracker_set_raim_flag(sid);
  }
}

static void starling_obs_to_nav_meas(const starling_obs_t *obs,
                                     navigation_measurement_t *nm) {
  nm->raw_pseudorange = obs->pseudorange;
  nm->pseudorange = obs->pseudorange;
  nm->raw_carrier_phase = obs->carrier_phase;
  nm->carrier_phase = obs->carrier_phase;
  nm->raw_measured_doppler = obs->doppler;
  nm->measured_doppler = obs->doppler;
  nm->raw_computed_doppler = obs->doppler;
  nm->computed_doppler = obs->doppler;
  nm->computed_doppler_dt = 0;
  memset(nm->sat_pos, 0, sizeof(nm->sat_pos));
  memset(nm->sat_vel, 0, sizeof(nm->sat_vel));
  memset(nm->sat_acc, 0, sizeof(nm->sat_acc));
  nm->IODE = 0;
  nm->sat_clock_err = 0;
  nm->sat_clock_err_rate = 0;
  nm->IODC = 0;
  nm->cn0 = obs->cn0;
  nm->lock_time = obs->lock_time;
  nm->elevation = 0;
  nm->tot = obs->tot;
  nm->sid = obs->sid;
  nm->flags = obs->flags;
  double el;
  if (track_sid_db_elevation_degrees_get(nm->sid, &el)) {
    nm->elevation = el;
  }
}

/* Apply corrections and solve for position from the given navigation
 * measurements, and if succesful update LGF and clock model */
static s8 me_compute_pvt(const obs_array_t *obs_array,
                         u64 current_tc,
                         const ephemeris_t e_meas[],
                         last_good_fix_t *lgf,
                         gnss_sid_set_t *raim_sids,
                         gnss_sid_set_t *raim_removed_sids) {
  u8 n_ready = obs_array->n;
  sid_set_init(raim_sids);
  for (u8 i = 0; i < n_ready; i++) {
    sid_set_add(raim_sids, obs_array->observations[i].sid);
  }

  if (n_ready < MINIMUM_SV_COUNT ||
      sid_set_get_sat_count(raim_sids) < MINIMUM_SV_COUNT) {
    /* Not enough sats to even try PVT, mark all measurements as bad */
    *raim_removed_sids = *raim_sids;
    return PVT_INSUFFICENT_MEAS;
  }

  /* Copy navigation measurements to a local array and create array of pointers
   * to it */
  static navigation_measurement_t nav_meas[MAX_CHANNELS];
  navigation_measurement_t *p_nav_meas[n_ready];
  for (u8 i = 0; i < n_ready; i++) {
    starling_obs_to_nav_meas(&obs_array->observations[i], &nav_meas[i]);
    p_nav_meas[i] = &nav_meas[i];
  }

  /* Compute satellite positions, velocities, and satellite clock and
   * clock rate corrections for all measurements*/
  for (u8 i = 0; i < n_ready; i++) {
    s8 sc_ret = calc_sat_state(&e_meas[i],
                               &(p_nav_meas[i]->tot),
                               p_nav_meas[i]->sat_pos,
                               p_nav_meas[i]->sat_vel,
                               p_nav_meas[i]->sat_acc,
                               &(p_nav_meas[i]->sat_clock_err),
                               &(p_nav_meas[i]->sat_clock_err_rate),
                               &(p_nav_meas[i]->IODC),
                               &(p_nav_meas[i]->IODE));

    if (sc_ret != 0) {
      log_error_sid(
          e_meas[i].sid, "calc_sat_state() returned error %d", sc_ret);
      return PVT_INSUFFICENT_MEAS; /* TODO define "other error?" */
    }
  }

  /* correct measurements for satellite clock and clock rate errors */
  apply_sat_clock_corrections(n_ready, p_nav_meas);

  /* apply GPS inter-signal corrections from CNAV messages */
  apply_gps_cnav_isc(n_ready, p_nav_meas, e_meas);

  /* apply iono and tropo corrections if LGF available */
  if (lgf->position_quality >= POSITION_GUESS) {
    ionosphere_t i_params;
    /* get iono parameters if available, otherwise use default ones */
    if (ndb_iono_corr_read(&i_params) != NDB_ERR_NONE) {
      i_params = DEFAULT_IONO_PARAMS;
    }
    correct_tropo(lgf->position_solution.pos_ecef, n_ready, nav_meas);
    correct_iono(lgf->position_solution.pos_ecef, &i_params, n_ready, nav_meas);
  }

  dops_t dops;
  gnss_solution current_fix;
  sid_set_init(raim_removed_sids);

  /* Calculate the SPP position
   * disable_raim controlled by external setting. Defaults to false. */
  s8 pvt_ret = calc_PVT(n_ready,
                        nav_meas,
                        &obs_array->t,
                        disable_raim,
                        /*disable_velocity = */ false,
                        GPS_L1CA_WHEN_POSSIBLE,
                        &current_fix,
                        &dops,
                        raim_removed_sids);

  if (pvt_ret < 0) {
    /* An error occurred with calc_PVT! */
    /* pvt_err_msg defined in starling/pvt.c */
    /* Print out max. once per second */
    DO_EACH_MS(
        SECS_MS,
        log_warn(
            "PVT solver: %s (code %d)", pvt_err_msg[-pvt_ret - 1], pvt_ret));

    /* If we already had a good fix, degrade its quality to STATIC */
    if (lgf->position_quality > POSITION_STATIC) {
      lgf->position_quality = POSITION_STATIC;
    }
    /* Mark all measurements bad */
    *raim_removed_sids = *raim_sids;
    return pvt_ret;
  }

  if (pvt_ret == PVT_CONVERGED_NO_RAIM) {
    /* no signals went through RAIM */
    sid_set_init(raim_sids);
  } else if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
    /* If we have a successful RAIM repair, check if the failed observations are
       * gross enough to have their channel dropped */
    for (u8 i = 0; i < n_ready; i++) {
      if (sid_set_contains(raim_removed_sids, nav_meas[i].sid)) {
        /* Check how large the outlier roughly is, and if it is a gross one,
         * drop the channel and delete the possibly corrupt ephemeris */
        drop_gross_outlier(nav_meas[i].sid,
                           nav_meas[i].pseudorange,
                           nav_meas[i].sat_pos,
                           &current_fix);
      }
    }
  }

  if (lgf->position_quality <= POSITION_GUESS) {
    /* Notify of the first fix */
    log_info("first fix clk_offset %.3e clk_drift %.3e",
             current_fix.clock_offset,
             current_fix.clock_drift);
  }

  /* Update the relationship between the solved GPS time and NAP count */
  update_time(current_tc, &current_fix);

  /* Update global position solution state. */
  lgf->position_solution = current_fix;
  lgf->position_quality = POSITION_FIX;
  /* Store the smoothed clock solution into lgf */
  lgf->position_solution.time = napcount2gpstime(current_tc);
  lgf->position_solution.clock_drift = get_clock_drift();
  ndb_lgf_store(&*lgf);

  return pvt_ret;
}

/* Generate a reference time using the TOW field from tracking channel and
 * time-of-ephemeris. This will be accurate to couple of milliseconds */
static gps_time_t reference_time_from_meas(channel_measurement_t *meas,
                                           ephemeris_t *ephe) {
  gps_time_t ref_time = GPS_TIME_UNKNOWN;
  ref_time.tow =
      (double)meas->time_of_week_ms / SECS_MS + GPS_NOMINAL_RANGE / GPS_C;
  normalize_gps_time(&ref_time);
  gps_time_match_weeks(&ref_time, &ephe->toe);

  return ref_time;
}

/* Copy out the signals that went through the last RAIM round and flag the
 * failed signals */
static void copy_raimed_obs(const obs_array_t *obs_array,
                            const gnss_sid_set_t *raim_sids,
                            const gnss_sid_set_t *raim_failed_sids,
                            obs_array_t *send_obs_array) {
  send_obs_array->n = 0;
  send_obs_array->t = obs_array->t;
  for (u8 i = 0; i < obs_array->n; i++) {
    gnss_signal_t sid = obs_array->observations[i].sid;
    if (sid_set_contains(raim_sids, sid)) {
      /* Copy the measurement to be sent out */
      send_obs_array->observations[send_obs_array->n] =
          obs_array->observations[i];
      if (sid_set_contains(raim_failed_sids, sid)) {
        /* Flag signals that failed the last RAIM */
        send_obs_array->observations[send_obs_array->n].flags |=
            NAV_MEAS_FLAG_RAIM_EXCLUSION;
      }
      send_obs_array->n++;
    }
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

  gnss_sid_set_t raim_sids;
  gnss_sid_set_t raim_failed_sids;
  sid_set_init(&raim_sids);
  sid_set_init(&raim_failed_sids);

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

    time_quality_t time_quality = get_time_quality();

    if (TIME_UNKNOWN != time_quality && lgf.position_solution.valid &&
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
     * time. Note that this will be invalid time stamp before the first fix. */
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

      /* time difference of current time from the output epoch */
      double dt = gpsdifftime(&output_time, &current_time);

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
    static channel_measurement_t meas[MAX_CHANNELS];
    static channel_measurement_t in_view[MAX_CHANNELS];
    static ephemeris_t e_meas[MAX_CHANNELS];

    /* Collect measurements propagated to the current NAP tick */
    collect_measurements(
        current_tc, meas, in_view, e_meas, &n_ready, &n_inview, &n_total);

    /* Send GSV messages for all satellites in track */
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
      sid_set_init(&raim_sids);
      continue;
    }

    /* If quality of current_time is not known, do not use it but instead
     * form a reference time from the first channels' TOW */
    if (TIME_UNKNOWN == time_quality) {
      current_time = reference_time_from_meas(&meas[0], &e_meas[0]);
      output_time = gps_time_round_to_epoch(&current_time, soln_freq);
      /* adjust the next sleep */
      double dt = gpsdifftime(&output_time, &current_time);
      /* make sure the first sleep lasts at least one epoch */
      if (dt < 0) {
        dt += 1.0 / soln_freq;
      }
      piksi_systime_add_us(&next_epoch, round(dt * SECS_US));
    }

    /* Initialize the observation array and create navigation measurements from
     * the channel measurements */
    static obs_array_t obs_array;
    s8 nm_ret =
        calc_navigation_measurement(n_ready, meas, &obs_array, &current_time);

    if (nm_ret != 0) {
      log_error("calc_navigation_measurement() returned error %d", nm_ret);
      me_send_emptyobs();
      sid_set_init(&raim_sids);
      continue;
    }

    /* Apply empirical (Piksi Multi specific) inter-signal corrections */
    apply_isc_table(&obs_array);

    /* Compute a PVT solution from the measurements to update LGF and clock
     * models. Compute on every epoch until the time quality gets to FINEST,
     * otherwise at a lower rate */
    u32 interval_ms =
        (TIME_FINEST > time_quality) ? 0 : ME_PVT_INTERVAL_S * SECS_MS;
    DO_EACH_MS(interval_ms,
               me_compute_pvt(&obs_array,
                              current_tc,
                              e_meas,
                              &lgf,
                              &raim_sids,
                              &raim_failed_sids));

    /* Get the current estimate of GPS time and receiver clock drift */
    gps_time_t smoothed_time = napcount2gpstime(current_tc);
    double smoothed_drift = get_clock_drift();

    /* update the time of reception to the better estimate */
    obs_array.t = smoothed_time;

    /* Get the offset of measurement time from the desired output time */
    double output_offset = gpsdifftime(&output_time, &obs_array.t);

    /* Only send observations that are closely aligned with the desired
     * solution epoch to ensure they haven't been propagated too far. */
    if (fabs(output_offset) < OBS_PROPAGATION_LIMIT) {
      log_debug("output offset %.4e, smoothed_drift %.3e",
                output_offset,
                smoothed_drift);

      /* Propagate the measurements to the output epoch */
      remove_clock_offset(&obs_array, &output_time, smoothed_drift, current_tc);

      if (disable_raim) {
        /* Send all observations. */
        me_send_all(&obs_array, e_meas);
      } else {
        /* Send only the observations that have gone through RAIM. */
        static obs_array_t send_obs_array;
        copy_raimed_obs(
            &obs_array, &raim_sids, &raim_failed_sids, &send_obs_array);
        me_send_all(&send_obs_array, e_meas);
      }
    } else {
      if (TIME_UNKNOWN != time_quality) {
        log_info("Observations suppressed because time jumps %.2f seconds",
                 output_offset);
      }
      /* Send the observations, but marked unusable */
      me_send_failed_obs(&obs_array, e_meas);
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
