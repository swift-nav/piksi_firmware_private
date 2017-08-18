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
#include <string.h>

#include <libsbp/sbp.h>
#include <libswiftnav/cnav_msg.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
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
#include "settings.h"
#include "shm.h"
#include "signal.h"
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

memory_pool_t obs_buff_pool;
mailbox_t obs_mailbox;

double soln_freq = 5.0;
u32 obs_output_divisor = 1;

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
                        const gps_time_t *_t) {
  me_post_observations(_num_obs, _meas, _ephem, _t);
  /* Output observations only every obs_output_divisor times, taking
  * care to ensure that the observations are aligned. */
  double t_check = _t->tow * (soln_freq / obs_output_divisor);
  if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
    send_observations(_num_obs, msg_obs_max_size, _meas, _t);
  }
}

static void me_send_emptyobs(void) {
  me_post_observations(0, NULL, NULL, NULL);
  send_observations(0, msg_obs_max_size, NULL, NULL);
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
    *pn_inview = n_inview;
    *pn_total = n_active;
  }
}

static THD_WORKING_AREA(wa_me_calc_pvt_thread, 1024 * 1024);
static void me_calc_pvt_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("me_calc_pvt");

  last_good_fix_t lgf;
  ndb_lgf_read(&lgf);

  piksi_systime_t next_epoch;
  piksi_systime_get(&next_epoch);
  piksi_systime_inc_us(&next_epoch, SECS_US / soln_freq);

  while (TRUE) {
    me_thd_sleep(&next_epoch, SECS_US / soln_freq);
    watchdog_notify(WD_NOTIFY_ME_CALC_PVT);

    if (get_time_quality() >= TIME_COARSE && lgf.position_solution.valid &&
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
    if (TIME_FINE == get_time_quality()) {
      /* If we have timing then we can calculate the relationship between
       * receiver time and GPS time and hence provide the pseudorange
       * calculation with the local GPS time of reception. */
      gps_time_t rec_time = napcount2gpstime(epoch_tc);
      epoch_time = gps_time_round_to_epoch(rec_time, soln_freq);
      epoch_tc = (u64)round(gpstime2napcount(&epoch_time));
      epoch_tc = FCN_NCO_RESET_COUNT *
                 ((epoch_tc + (FCN_NCO_RESET_COUNT / 2)) / FCN_NCO_RESET_COUNT);
    }

    double delta_tc = -((double)current_tc - (double)epoch_tc);

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

    if (n_ready < MINIMUM_SV_COUNT) {
      /* Not enough sats, keep on looping. */
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

    /* If a FINE quality time solution is not available then pass in NULL
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

    gnss_sid_set_t codes;
    sid_set_init(&codes);
    for (u8 i = 0; i < n_ready; i++) {
      sid_set_add(&codes, nav_meas[i].sid);
    }

    if (sid_set_get_sat_count(&codes) < 4) {
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
      calc_iono_tropo(n_ready,
                      nav_meas,
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

    /* If we have a success RAIM repair, mark the removed observations as
       invalid, and ask tracker to drop the channels (if needed). */
    if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
      for (u8 i = 0; i < n_ready; i++) {
        if (sid_set_contains(&raim_removed_sids, nav_meas[i].sid)) {
          log_debug_sid(nav_meas[i].sid,
                        "RAIM repair, setting observation invalid.");
          nav_meas[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;

          /* Check how large the outlier roughly is, and if it is a gross one,
           * drop the channel */
          double geometric_range[3];
          for (u8 j = 0; j < 3; j++) {
            geometric_range[j] =
                nav_meas[i].sat_pos[j] - current_fix.pos_ecef[j];
          }
          if (fabs(+nav_meas[i].pseudorange +
                   -current_fix.clock_offset * GPS_C +
                   -vector_norm(3, geometric_range)) >
              RAIM_DROP_CHANNEL_THRESHOLD_M) {
            tracking_channel_set_raim_flag(nav_meas[i].sid);
          }
        }
      }
    }

    if (get_time_quality() < TIME_FINE) {
      /* If the time quality is not FINE then our receiver clock bias isn't
       * known. We should only use this PVT solution to update our time
       * estimate and then skip all other processing.
       *
       * Note that the lack of knowledge of the receiver clock bias does NOT
       * degrade the quality of the position solution but the rapid change in
       * bias after the time estimate is first improved may cause issues for
       * e.g. carrier smoothing. Easier just to discard this first solution.
       */
      set_time_fine(epoch_tc, current_fix.time);

      log_info("first fix clk_offset %.3e clk_drift %.3e",
               current_fix.clock_offset,
               current_fix.clock_bias);

      /* store this fix as a guess so the satellite elevations and iono/tropo
       * corrections can be computed for the first actual fix */
      lgf.position_solution = current_fix;
      lgf.position_quality = POSITION_GUESS;
      ndb_lgf_store(&lgf);
      continue;
    }

    /* We now have the nap count we expected the measurements to be at, plus
     * the GPS time error for that nap count so we need to store this error in
     * the the GPS time (GPS time frame) */
    set_gps_time_offset(epoch_tc, current_fix.time);

    /* Update global position solution state. */
    lgf.position_solution = current_fix;
    lgf.position_quality = POSITION_FIX;
    ndb_lgf_store(&lgf);

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(current_fix.clock_offset) < OBS_PROPAGATION_LIMIT) {
      log_debug("clk_offset %.3e clk_drift %.3e",
                current_fix.clock_offset,
                current_fix.clock_bias);

      for (u8 i = 0; i < n_ready; i++) {
        navigation_measurement_t *nm = &nav_meas[i];

        double doppler = 0.0;
        if (0 != (nm->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
          doppler = nm->raw_measured_doppler;
        }

        /* The pseudorange correction has opposite sign because Doppler
         * has the opposite sign compared to the pseudorange rate. */
        nm->raw_pseudorange -=
            (current_fix.clock_offset) * doppler * sid_to_lambda(nm->sid);
        nm->raw_pseudorange -= current_fix.clock_offset * GPS_C;
        /* Carrier Phase corrected by clock offset */
        nm->raw_carrier_phase += (current_fix.clock_offset) * doppler;
        nm->raw_carrier_phase +=
            current_fix.clock_offset * GPS_C / sid_to_lambda(nm->sid);
        /* Use P**V**T to determine the oscillator drift which is used
         * to adjust computed doppler. */
        nm->raw_measured_doppler +=
            current_fix.clock_bias * GPS_C / sid_to_lambda(nm->sid);
        nm->raw_computed_doppler = nm->raw_measured_doppler;

        /* Also apply the time correction to the time of transmission so the
        * satellite positions can be calculated for the correct time. */
        nm->tot.tow += (current_fix.clock_offset);
        normalize_gps_time(&(nm->tot));

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
    }

    if (fabs(current_fix.clock_offset) > MAX_CLOCK_ERROR_S) {
      /* Note we should not enter here except in very exceptional circumstances,
       * like time solved grossly wrong on the first fix. */
      log_warn("Receiver clock offset larger than %g ms, applying 2 ms jump",
               MAX_CLOCK_ERROR_S * SECS_MS);

      /* round the time adjustment to even milliseconds */
      double dt = round(current_fix.clock_offset * 2 * SECS_MS) / SECS_MS;
      /* adjust the RX to GPS time conversion */
      adjust_time_fine(dt);
      /* adjust all the carrier phase offsets */
      /* note that the adjustment is always in even cycles because millisecond
       * breaks up exactly into carrier cycles */
      tracking_channel_carrier_phase_offsets_adjust(dt);
    }

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
  SETTING("solution", "soln_freq", soln_freq, TYPE_FLOAT);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);
  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  static msg_t obs_mailbox_buff[OBS_N_BUFF];

  chMBObjectInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);

  chPoolObjectInit(&obs_buff_pool, sizeof(me_msg_obs_t), NULL);

  static me_msg_obs_t obs_buff[OBS_N_BUFF] _CCM;

  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  /* Start solution thread */
  chThdCreateStatic(wa_me_calc_pvt_thread,
                    sizeof(wa_me_calc_pvt_thread),
                    HIGHPRIO - 2,
                    me_calc_pvt_thread,
                    NULL);
}
