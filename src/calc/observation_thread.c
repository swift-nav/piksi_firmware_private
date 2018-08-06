/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "observation_thread.h"

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
#include <libswiftnav/troposphere.h>

#include <starling/starling.h>
#include <starling/starling_platform.h>

#include "board/nap/track_channel.h"
#include "calc_pvt_common.h"
#include "calc_starling_obs_array.h"
#include "main/main.h"
#include "ndb/ndb.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "utils/nmea/nmea.h"
#include "utils/obs_bias/obs_bias.h"
#include "utils/position/position.h"
#include "utils/settings/settings.h"
#include "utils/shm/shm.h"
#include "utils/system_monitor/system_monitor.h"

/** Mandatory flags filter for measurements */
#define MANAGE_TRACK_FLAGS_FILTER                               \
  (MANAGE_TRACK_FLAG_ACTIVE | MANAGE_TRACK_FLAG_NO_ERROR |      \
   MANAGE_TRACK_FLAG_CONFIRMED | MANAGE_TRACK_FLAG_CN0_SHORT |  \
   MANAGE_TRACK_FLAG_ELEVATION | MANAGE_TRACK_FLAG_HAS_EPHE |   \
   MANAGE_TRACK_FLAG_HEALTHY | MANAGE_TRACK_FLAG_NAV_SUITABLE | \
   MANAGE_TRACK_FLAG_TOW)

/* Maximum time to maintain POSITION_FIX after last successful solution */
#define POSITION_FIX_TIMEOUT_S 60

#define ME_OBS_THREAD_PRIORITY (HIGHPRIO - 3)
#define ME_OBS_THREAD_STACK (64 * 1024)

/* Limits the sets of possible solution frequencies (in increasing order) */
static const double valid_soln_freqs_hz[] = {1.0, 2.0, 4.0, 5.0, 10.0};

#define SOLN_FREQ_SETTING_MIN (valid_soln_freqs_hz[0])
#define SOLN_FREQ_SETTING_MAX \
  (valid_soln_freqs_hz[ARRAY_SIZE(valid_soln_freqs_hz) - 1])

double soln_freq_setting = 10.0;
u32 obs_output_divisor = 10;

s16 msg_obs_max_size = SBP_FRAMING_MAX_PAYLOAD_SIZE;

static soln_stats_t last_stats = {.signals_tracked = 0, .signals_useable = 0};

/* STATIC FUNCTIONS */
static void me_post_ephemerides(u8 n, const ephemeris_t ephemerides[]) {
  ephemeris_array_t *eph_array = platform_mailbox_item_alloc(MB_ID_EPHEMERIS);
  if (NULL == eph_array) {
    /* If we can't get allocate an item, fetch the oldest one and use that
     * instead. */
    int error = platform_mailbox_fetch(
        MB_ID_EPHEMERIS, (void **)&eph_array, MB_NONBLOCKING);
    if (error) {
      log_error(
          "ME: Unable to allocate ephemeris array, and mailbox is empty.");
      if (eph_array) {
        platform_mailbox_item_free(MB_ID_EPHEMERIS, eph_array);
      }
      return;
    }
  }

  assert(NULL != eph_array);
  /* Copy in all of the information. */
  eph_array->n = n;
  if (n > 0) {
    MEMCPY_S(eph_array->ephemerides,
             sizeof(eph_array->ephemerides),
             ephemerides,
             n * sizeof(ephemeris_t));
  }
  /* Try to post to Starling. */
  int error = platform_mailbox_post(MB_ID_EPHEMERIS, eph_array, MB_BLOCKING);
  if (error) {
    log_error("ME: Unable to send ephemeris array.");
    platform_mailbox_item_free(MB_ID_EPHEMERIS, eph_array);
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

      if (0 != (flags & TRACKER_FLAG_ELEVATION) &&
          0 != (flags & TRACKER_FLAG_TOW_VALID) &&
          0 != (flags & TRACKER_FLAG_HAS_EPHE) &&
          0 != (flags & TRACKER_FLAG_CN0_USABLE) &&
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
                        (0 != (flags & TRACKER_FLAG_CN0_USABLE)) ? "Y" : "N");
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

static THD_WORKING_AREA(wa_me_obs_thread, ME_OBS_THREAD_STACK);

u8 collect_nav_meas(u64 current_tc,
                    gps_time_t *current_time,
                    const last_good_fix_t *lgf,
                    navigation_measurement_t nav_meas[],
                    ephemeris_t e_meas[]) {
  u8 n_ready = 0;
  u8 n_inview = 0;
  u8 n_total = 0;
  channel_measurement_t meas[MAX_CHANNELS];
  channel_measurement_t in_view[MAX_CHANNELS];

  /* Collect measurements propagated to the current NAP tick */
  collect_measurements(
      current_tc, meas, in_view, e_meas, &n_ready, &n_inview, &n_total);

  /* Update stats */
  last_stats.signals_tracked = n_total;
  last_stats.signals_useable = n_ready;

  nmea_send_gsv(n_inview, in_view);

  log_debug("Selected %" PRIu8 " measurement(s) out of %" PRIu8
            " in view "
            " (total=%" PRIu8 ")",
            n_ready,
            n_inview,
            n_total);

  if (n_ready == 0) {
    return n_ready;
  }

  cnav_msg_t cnav_30[MAX_CHANNELS];
  const cnav_msg_type_30_t *p_cnav_30[MAX_CHANNELS];

  for (u8 i = 0; i < n_ready; i++) {
    p_cnav_30[i] = cnav_msg_get(meas[i].sid, CNAV_MSG_TYPE_30, &cnav_30[i])
                       ? &cnav_30[i].data.type_30
                       : NULL;
  }
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
  if (!gps_time_valid(current_time)) {
    current_time->tow =
        (double)meas[0].time_of_week_ms / SECS_MS + GPS_NOMINAL_RANGE / GPS_C;
    normalize_gps_time(current_time);
    gps_time_match_weeks(current_time, &e_meas[0].toe);
  }

  /* Create navigation measurements from the channel measurements */
  s8 nm_ret =
      calc_navigation_measurement(n_ready, p_meas, p_nav_meas, &*current_time);
  if (nm_ret != 0) {
    log_error("calc_navigation_measurement() returned an error");
    return 0;
  }

  s8 sc_ret = calc_sat_clock_corrections(n_ready, p_nav_meas, p_e_meas);
  if (sc_ret != 0) {
    log_error("calc_sat_clock_correction() returned an error");
    return 0;
  }

  apply_gps_cnav_isc(n_ready, p_nav_meas, p_cnav_30, p_e_meas);
  apply_isc_table(n_ready, p_nav_meas);

  /* check if we have a solution, if yes calc iono and tropo correction */
  if (lgf->position_quality >= POSITION_GUESS) {
    ionosphere_t i_params;
    /* get iono parameters if available, otherwise use default ones */
    if (ndb_iono_corr_read(&i_params) != NDB_ERR_NONE) {
      i_params = DEFAULT_IONO_PARAMS;
    }
    correct_tropo(lgf->position_solution.pos_ecef, n_ready, nav_meas);
    correct_iono(lgf->position_solution.pos_ecef, &i_params, n_ready, nav_meas);
  }
  return n_ready;
}

static void me_obs_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("me_obs");

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

    /* sleep until next epoch, and update the deadline */
    me_thd_sleep(&next_epoch, SECS_US / soln_freq);
    watchdog_notify(WD_NOTIFY_ME_OBS);

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

    if (n_ready == 0) {
      me_send_emptyobs();
      continue;
    }

    if (nav_meas_get_sat_count(n_ready, nav_meas) < 4) {
      /* Not enough sats to compute PVT, send them as unusable */
      me_send_failed_obs(n_ready, nav_meas, e_meas, &current_time);
      continue;
    }

    /* Get the updated time and drift */
    gps_time_t smoothed_time = napcount2gpstime(current_tc);
    double smoothed_drift = get_clock_drift();

    if (!gps_time_valid(&output_time)) {
      /* GPS time not solved yet */
      log_warn("GPS time not solved yet, can't make measurements");
      me_send_failed_obs(n_ready, nav_meas, e_meas, &current_time);
      continue;
    }

    /* offset of smoothed solution time from the desired output time */
    double output_offset = gpsdifftime(&output_time, &smoothed_time);

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(output_offset) < OBS_PROPAGATION_LIMIT) {
      /*log_debug(
          "clk_offset %.4e, output offset %.4e, clk_drift %.3e, "
          "smoothed_drift %.3e",
          current_fix.clock_offset,
          output_offset,
          current_fix.clock_drift,
          smoothed_drift);
      */
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
  }
}

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

soln_stats_t solution_last_stats_get(void) { return last_stats; }

void me_obs_setup() {
  SETTING_NOTIFY("solution",
                 "soln_freq",
                 soln_freq_setting,
                 TYPE_FLOAT,
                 soln_freq_setting_notify);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);
  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  /* Start solution thread */
  chThdCreateStatic(wa_me_obs_thread,
                    sizeof(wa_me_obs_thread),
                    ME_OBS_THREAD_PRIORITY,
                    me_obs_thread,
                    NULL);
}

u8 nav_meas_get_sat_count(u8 n_ready,
                          const navigation_measurement_t nav_meas[]) {
  gnss_sid_set_t codes;
  sid_set_init(&codes);
  for (u8 i = 0; i < n_ready; i++) {
    sid_set_add(&codes, nav_meas[i].sid);
  }
  return sid_set_get_sat_count(&codes);
}
