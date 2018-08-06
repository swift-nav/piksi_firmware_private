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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/correct_iono_tropo.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/glonass_phase_biases.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/signal.h>
#include <starling/starling.h>
#include <starling/starling_platform.h>

#include "calc_base_obs.h"
#include "calc_nav_meas.h"
#include "calc_pvt_me.h"
#include "manage.h"
#include "nav_msg/cnav_msg_storage.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "timing/timing.h"

bool disable_raim = false;

/**
 * Uncollapsed observation input type.
 * Remote observations may contain multiple useful signals for satellites.
 * This observation type can fit more observations than the obss_t.
 * Eventually signals in uncollapsed_obss_t are collapsed, and copied to obss_t.
 */
typedef struct {
  /** GPS system time of the observation. */
  gps_time_t tor;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** Observation Solution */
  pvt_engine_result_t soln;

  /** Number of observations in the set. */
  u8 n;
  u8 sender_id;
  /** Set of observations. */
  navigation_measurement_t nm[STARLING_MAX_OBS_COUNT];
} uncollapsed_obss_t;

/** \defgroup base_obs Base station observation handling
 * \{ */

/** Most recent observations from the base station. */
obss_t base_obss;

static u32 base_obs_msg_counter = 0;
static u8 old_base_sender_id = 0;

/** SBP callback for when the base station sends us a message containing its
 * known location in LLH coordinates.
 */
static void base_pos_llh_callback(u16 sender_id,
                                  u8 len,
                                  u8 msg[],
                                  void *context) {
  (void)context;
  (void)len;
  /* Skip forwarded sender_ids. See note in obs_callback about echo'ing
   * sender_id. */
  if (MSG_FORWARD_SENDER_ID == sender_id) {
    return;
  }
  /*TODO: keep track of sender_id to store multiple base positions?*/
  double llh_degrees[3];
  double llh[3], base_pos[3];
  MEMCPY_S(llh_degrees, sizeof(llh_degrees), msg, sizeof(llh_degrees));
  llh[0] = llh_degrees[0] * D2R;
  llh[1] = llh_degrees[1] * D2R;
  llh[2] = llh_degrees[2];
  wgsllh2ecef(llh, base_pos);

  starling_set_known_ref_pos(base_pos);
  /* Relay base station position using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_BASE_POS_LLH, len, msg, MSG_FORWARD_SENDER_ID);
}

/** SBP callback for when the base station sends us a message containing its
 * known location in ECEF coordinates.
 */
static void base_pos_ecef_callback(u16 sender_id,
                                   u8 len,
                                   u8 msg[],
                                   void *context) {
  (void)context;
  (void)len;
  /* Skip forwarded sender_ids. See note in obs_callback about echo'ing
   * sender_id. */
  if (MSG_FORWARD_SENDER_ID == sender_id) {
    return;
  }
  double base_pos[3];
  MEMCPY_S(base_pos, sizeof(base_pos), msg, sizeof(base_pos));

  starling_set_known_ref_pos(base_pos);
  /* Relay base station position using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_BASE_POS_ECEF, len, msg, MSG_FORWARD_SENDER_ID);
}

/** SBP callback for when the base station sends us a message containing its
 * known GLONASS code-phase bias (RTCM 1230).
 */
static void base_glonass_biases_callback(u16 sender_id,
                                         u8 len,
                                         u8 msg[],
                                         void *context) {
  (void)context;
  (void)len;
  /* Skip forwarded sender_ids. See note in obs_callback about echo'ing
   * sender_id. */
  if (MSG_FORWARD_SENDER_ID == sender_id) {
    return;
  }
  glo_biases_t biases;
  unpack_glonass_biases_content(*(msg_glo_biases_t *)msg, &biases);

  starling_set_known_glonass_biases(biases);
  /* Relay base station GLONASS biases using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_GLO_BIASES, len, msg, MSG_FORWARD_SENDER_ID);
}

static inline bool shm_suitable_wrapper(navigation_measurement_t meas) {
  return shm_navigation_suitable(meas.sid);
}

/* Helper function used for sorting starling observations based on their
 * SID field. */
static int compare_starling_obs_by_sid(const void *a, const void *b) {
  return sid_compare(((starling_obs_t *)a)->sid, ((starling_obs_t *)b)->sid);
}

/* Check that a given time is aligned (within some tolerance) to the
 * local solution epoch. */
static bool is_time_aligned_to_local_epoch(const gps_time_t *t) {
  gps_time_t epoch =
      gps_time_round_to_epoch(t, soln_freq_setting / obs_output_divisor);
  double dt = gpsdifftime(&epoch, t);
  return (fabs(dt) <= TIME_MATCH_THRESHOLD);
}

/* We can determine if an obs message is the first in sequence
 * by examining its "count" field. */
static bool is_first_message_in_obs_sequence(u8 count) { return count == 0; }

/* We can determine if this was the final obs message in the sequence
 * by comparing the "count" and "total" fields. */
static bool is_final_message_in_obs_sequence(u8 count, u8 total) {
  return count == total - 1;
}

/* Converter for moving into the intermediary uncollapsed observation type.
 * This performs some preliminary filtering of the observations in addition to
 * populating the navigation measurement fields. */
static void convert_starling_obs_array_to_uncollapsed_obss(
    obs_array_t *obs_array, uncollapsed_obss_t *obss) {
  assert(obs_array);
  assert(obss);
  assert(obs_array->n <= STARLING_MAX_OBS_COUNT);

  obss->sender_id = obs_array->sender;
  obss->tor = obs_array->t;
  obss->has_pos = 0;
  obss->soln.valid = 0;

  /* Selectively populate the navigation measurement array. */
  obss->n = 0;
  for (size_t i = 0; i < obs_array->n; ++i) {
    navigation_measurement_t *nm = &obss->nm[obss->n];
    convert_starling_obs_to_navigation_measurement(&obs_array->observations[i],
                                                   nm);

    /* Filter out any observation without a valid pseudorange observation. */
    if (!pseudorange_valid(*nm)) {
      continue;
    }

    /* Filter out any observation not marked healthy by the SHM. */
    if (shm_navigation_unusable(nm->sid)) {
      continue;
    }

    /* Calculate satellite parameters using the ephemeris. */
    ephemeris_t ephe;
    ndb_op_code_t res = ndb_ephemeris_read(nm->sid, &ephe);
    s8 cscc_ret, css_ret;
    const ephemeris_t *ephe_p = &ephe;

    /* TTFF shortcut: accept also unconfirmed ephemeris candidate when there
     * is no confirmed candidate */
    bool eph_valid = (NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res) &&
                     ephemeris_valid(&ephe, &nm->tot);

    if (eph_valid) {
      /* Apply corrections to the pseudorange, carrier phase and Doppler. */
      cscc_ret = calc_sat_clock_corrections(1, &nm, &ephe_p);

      /* After correcting the time of transmission for the satellite clock
         error,
         recalculate the satellite position. */
      css_ret = calc_sat_state(&ephe,
                               &nm->tot,
                               nm->sat_pos,
                               nm->sat_vel,
                               nm->sat_acc,
                               &nm->sat_clock_err,
                               &nm->sat_clock_err_rate,
                               &nm->IODC,
                               &nm->IODE);
    }

    if (!eph_valid || (cscc_ret != 0) || (css_ret != 0)) {
      continue;
    }

    /* Only update the output index if we actually passed all the tests. */
    obss->n++;
  }
}

/** Update the #base_obss state given a new set of obss.
 * First sorts by PRN and computes the TDCP Doppler for the observation set. If
 * #base_pos_known is false then a single point position solution is also
 * calculated. Next the `has_pos`, `pos_ecef` and `sat_dists` fields are filled
 * in. Finally the #base_obs_received semaphore is flagged to indicate that new
 * observations are available.
 *
 * \note This function is stateful as it must store the previous observation
 *       set for the TDCP Doppler.
 */
static void update_obss(obs_array_t *obs_array) {
  /* Ensure raw observations are sorted by PRN. */
  qsort(obs_array->observations,
        obs_array->n,
        sizeof(obs_array->observations[0]),
        compare_starling_obs_by_sid);

  /* First we need to convert the obs array into this type. */
  uncollapsed_obss_t uncollapsed_obss;
  uncollapsed_obss_t *new_uncollapsed_obss = &uncollapsed_obss;
  convert_starling_obs_array_to_uncollapsed_obss(obs_array,
                                                 new_uncollapsed_obss);

  /* Warn on receiving observations which are very old. This may be indicative
   * of a connectivity problem. Obviously, if we don't have a good local time
   * estimate, then we can't perform this check. */
  gps_time_t now = get_current_time();
  if (get_time_quality() > TIME_UNKNOWN &&
      gpsdifftime(&now, &new_uncollapsed_obss->tor) > BASE_LATENCY_TIMEOUT) {
    log_info("Communication latency exceeds 15 seconds");
  }

  /** Precheck any base station observations and filter if needed. This is not a
   *  permanent solution for actually correcting GPS L2 base station
   *  observations that have mixed tracking modes in a signal epoch. For more
   *  details, see:
   *  https://github.com/swift-nav/estimation_team_planning/issues/215.
   */
  filter_base_meas(&new_uncollapsed_obss->n, new_uncollapsed_obss->nm);

  if (new_uncollapsed_obss->n == 0) {
    log_info("All base obs filtered");
    return;
  }

  /* After collapsing the measurements to Piksi supported signals,
   * only less or equal than MAX_CHANNELS measurements should remain. */
  if (new_uncollapsed_obss->n > MAX_CHANNELS) {
    log_warn("Obs collapsing failure");
  }

  /* Copy contents of new_uncollapsed_obss into new_obss. */
  obss_t obss;
  obss_t *new_obss = &obss;
  new_obss->tor = new_uncollapsed_obss->tor;
  new_obss->has_pos = new_uncollapsed_obss->has_pos;
  new_obss->soln = new_uncollapsed_obss->soln;
  new_obss->n = new_uncollapsed_obss->n;
  new_obss->sender_id = new_uncollapsed_obss->sender_id;
  MEMCPY_S(new_obss->pos_ecef,
           sizeof(new_obss->pos_ecef),
           new_uncollapsed_obss->pos_ecef,
           sizeof(new_uncollapsed_obss->pos_ecef));
  MEMCPY_S(new_obss->nm,
           sizeof(new_obss->nm),
           new_uncollapsed_obss->nm,
           MAX_CHANNELS * sizeof(navigation_measurement_t));

  /* Count distinct satellites */
  gnss_sid_set_t codes;
  sid_set_init(&codes);
  for (u8 i = 0; i < new_obss->n; i++) {
    if (sid_to_constellation(new_obss->nm[i].sid) == CONSTELLATION_GPS) {
      sid_set_add(&codes, new_obss->nm[i].sid);
    }
  }

  /* Require at least 5 distinct satellites */
  if (sid_set_get_sat_count(&codes) >= MIN_SATS_FOR_PVT) {
    bool base_changed = (old_base_sender_id != 0) &&
                        (old_base_sender_id != new_obss->sender_id);
    /* check if we have fix, if yes, calculate iono and tropo correction */
    if (!base_changed && base_obss.has_pos) {
      log_debug("Base: IONO/TROPO correction");
      ionosphere_t i_params;
      /* get iono parameters if available, otherwise use default ones */
      if (ndb_iono_corr_read(&i_params) != NDB_ERR_NONE) {
        i_params = DEFAULT_IONO_PARAMS;
      }
      /* Use the previous ECEF position to get the iono/tropo for the new
       * measurements */
      correct_tropo(base_obss.pos_ecef, new_obss->n, new_obss->nm);
      correct_iono(base_obss.pos_ecef, &i_params, new_obss->n, new_obss->nm);
    }

    gnss_solution soln;
    dops_t dops;

    /* Calculate a position solution. */
    /* disable_raim controlled by external setting (see solution.c). */
    /* Skip velocity solving for the base incase we have bad doppler values
     * due to a cycle slip. */
    s32 ret = calc_PVT(new_obss->n,
                       new_obss->nm,
                       &new_obss->tor,
                       disable_raim,
                       true,
                       GPS_ONLY,
                       &soln,
                       &dops,
                       NULL);

    if (ret >= 0 && soln.valid) {
      /* Copy over the time. */
      base_obss.tor = new_obss->tor;

      base_obss.n = new_obss->n;
      MEMCPY_S(base_obss.nm,
               sizeof(base_obss.nm),
               new_obss->nm,
               new_obss->n * sizeof(navigation_measurement_t));

      MEMCPY_S(base_obss.pos_ecef,
               sizeof(base_obss.pos_ecef),
               soln.pos_ecef,
               sizeof(soln.pos_ecef));

      /* Copy over sender ID. */
      base_obss.sender_id = new_obss->sender_id;

      /* Check if the base sender ID has changed and reset the RTK filter if
       * it has.
       */
      if (base_changed) {
        log_warn(
            "Base station sender ID changed from %u to %u. Resetting RTK"
            " filter.",
            old_base_sender_id,
            base_obss.sender_id);
        starling_reset_rtk_filter();
      }
      old_base_sender_id = base_obss.sender_id;

      base_obss.has_pos = 1;

      obss_t *new_base_obs = platform_mailbox_item_alloc(MB_ID_BASE_OBS);
      if (new_base_obs == NULL) {
        log_warn(
            "Base obs pool full, discarding base obs at: wn: %d, tow: %.2f",
            base_obss.tor.wn,
            base_obss.tor.tow);
        return;
      }

      *new_base_obs = base_obss;

      const errno_t post_ret =
          platform_mailbox_post(MB_ID_BASE_OBS, new_base_obs, MB_NONBLOCKING);
      if (post_ret != 0) {
        log_error("Base obs mailbox should have space!");
        platform_mailbox_item_free(MB_ID_BASE_OBS, new_base_obs);
      }
    } else {
      base_obss.has_pos = 0;
      /* TODO(dsk) check for repair failure */
      /* There was an error calculating the position solution. */
      log_warn("Error calculating base station position: (%s).",
               pvt_err_msg[-ret - 1]);
    }
  }
}

/** SBP callback for observation messages.
 * SBP observation sets are potentially split across multiple SBP messages to
 * keep the payload within the size limit.
 *
 * The header contains a count of how many total messages there are in this set
 * of observations (all referring to the same observation time) and a count of
 * which message this is in the sequence.
 *
 * This function attempts to collect a full set of observations into a single
 * `obss_t` (`base_obss_rx`). Once a full set is received then update_obss()
 * is called.
 */
static void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)context;

  /* Keep track of where in the sequence of messages we were last time around
   * so we can verify we haven't dropped a message. */
  static s16 prev_count = 0;
  static gps_time_t prev_tor = GPS_TIME_UNKNOWN;

  /* Storage for collecting the incoming observations. */
  static obs_array_t obs_array;

  obs_array.sender = sender_id;

  /* An SBP sender ID of zero means that the messages are relayed observations
   * from the console, not from the base station. We don't want to use them and
   * we don't want to create an infinite loop by forwarding them again so just
   * ignore them. */
  if (MSG_FORWARD_SENDER_ID == sender_id) {
    return;
  }

  /* Relay observations using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_OBS, len, msg, MSG_FORWARD_SENDER_ID);

  /* GPS time of observation. */
  gps_time_t tor = GPS_TIME_UNKNOWN;
  /* Total number of messages in the observation set / sequence. */
  u8 total;
  /* The current message number in the sequence. */
  u8 count;

  /* Decode the message header to get the time and how far through the sequence
   * we are. */
  unpack_obs_header((observation_header_t *)msg, &tor, &total, &count);

  /* Check to see if the observation is aligned with our internal observations,
   * i.e. is it going to time match one of our local obs. */
  /* if tor is invalid this obs message was sent before a time was solved, so we
   * should ignore */
  if (!gps_time_valid(&tor)) {
    return;
  }

  /* Check that messages are in chronological order. We only perform this check
   * when receiving a new sequence of observations. */
  static gps_time_t tor_old = GPS_TIME_UNKNOWN;
  if (is_first_message_in_obs_sequence(count)) {
    if (gps_time_valid(&tor_old) && gpsdifftime(&tor, &tor_old) <= 0) {
      log_info(
          "Observation received with equal or earlier time stamp, ignoring");
      return;
    }
  }
  tor_old = tor;

  /* Check that the base station's messages align well enough to our local
   * processing epochs. */
  if (!is_time_aligned_to_local_epoch(&tor)) {
    if (is_first_message_in_obs_sequence(count)) {
      log_warn(
          "Unaligned observation from base ignored, tow = %.3f,"
          " Base station observation rate and solution"
          " frequency may be mismatched.",
          tor.tow);
    }
    return;
  }

  /* Verify sequence integrity */
  if (is_first_message_in_obs_sequence(count)) {
    prev_tor = tor;
    prev_count = 0;
  } else if ((fabs(gpsdifftime(&tor, &prev_tor)) > FLOAT_EQUALITY_EPS) ||
             (prev_tor.wn != tor.wn) || ((prev_count + 1) != count)) {
    log_info("Dropped one base observation packet, skipping this base epoch.");
    prev_count = -1;
    return;
  } else {
    prev_count = count;
  }

  /* If this is the first packet in the sequence then reset the base_obss_rx
   * state. */
  if (is_first_message_in_obs_sequence(count)) {
    obs_array.n = 0;
    obs_array.t = tor;
  }

  /* Calculate the number of observations in this message by looking at the SBP
   * `len` field. */
  u8 obs_in_msg =
      (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

  /* Pull out the contents of the message. */
  packed_obs_content_t *msg_raw_obs =
      (packed_obs_content_t *)(msg + sizeof(observation_header_t));

  /* Copy into local array. */
  for (size_t i = 0; i < obs_in_msg && obs_array.n < STARLING_MAX_OBS_COUNT;
       ++i) {
    starling_obs_t *current_obs = &obs_array.observations[obs_array.n++];
    unpack_obs_content(&msg_raw_obs[i], current_obs);
    /* We must also compute the TOT using the TOR from the header. */
    current_obs->tot = obs_array.t;
    current_obs->tot.tow -= current_obs->P / GPS_C;
    normalize_gps_time(&current_obs->tot);
  }

  /* Print msg if we encounter a remote which sends large amount of obs. */
  if (STARLING_MAX_OBS_COUNT == obs_array.n) {
    log_info("Remote obs reached maximum: %d", STARLING_MAX_OBS_COUNT);
  }

  /* If we can, and all the obs have been received, update to using the new
   * obss. */
  if (is_final_message_in_obs_sequence(count, total)) {
    // TODO(kevin) Do something with the obs array.
    update_obss(&obs_array);

    /* Calculate packet latency. */
    if (get_time_quality() >= TIME_COARSE) {
      gps_time_t now = get_current_time();
      float latency_ms = (float)(gpsdifftime(&now, &tor) * 1000.0);
      log_obs_latency(latency_ms);
    }
    /* Update message counter */
    base_obs_msg_counter++;
  }
}

/** SBP callback for the old style observation messages.
 * Just logs a deprecation warning. */
static void deprecated_callback(u16 sender_id,
                                u8 len,
                                u8 msg[],
                                void *context) {
  (void)context;
  (void)len;
  (void)msg;
  (void)sender_id;
  log_warn(
      "Received a deprecated obs msg. Verify firmware version on remote "
      "Piksi.");
}

/** SBP callback for Group delay message
 */
static void ics_msg_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)sender_id;
  (void)len;
  (void)context;

  /* Skip forwarded sender_ids. See note in obs_callback about echo'ing
   * sender_id. */
  if (MSG_FORWARD_SENDER_ID == sender_id) {
    return;
  }

  log_debug("Group delay received from peer");

  cnav_msg_t cnav;
  memset(&cnav, 0, sizeof(cnav));

  /* unpack received message */
  cnav.data.type_30.isc_l1ca = ((msg_group_delay_t *)msg)->isc_l1ca;
  cnav.data.type_30.tgd = ((msg_group_delay_t *)msg)->tgd;
  cnav.prn = ((msg_group_delay_t *)msg)->sid.sat;
  cnav.data.type_30.isc_l1ca_valid =
      (((msg_group_delay_t *)msg)->valid >> 2) & 0x1;
  cnav.data.type_30.isc_l2c_valid =
      (((msg_group_delay_t *)msg)->valid >> 1) & 0x1;
  cnav.data.type_30.tgd_valid = (((msg_group_delay_t *)msg)->valid) & 0x1;
  cnav.tow = ((msg_group_delay_t *)msg)->t_op.tow;
  cnav.alert = 0;
  cnav.bit_polarity = 0;
  cnav.msg_id = CNAV_MSG_TYPE_30;

  /* store received CNAV message */
  cnav_msg_put(&cnav);
}

/** Setup the base station observation handling subsystem. */
void base_obs_setup() {
  // The base obs can optionally enable RAIM exclusion algorithm.
  SETTING("solution", "disable_raim", disable_raim, TYPE_BOOL);

  platform_mailbox_init(MB_ID_BASE_OBS);

  /* Register callbacks on base station messages. */
  static sbp_msg_callbacks_node_t base_pos_llh_node;
  sbp_register_cbk(
      SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, &base_pos_llh_node);

  static sbp_msg_callbacks_node_t base_pos_ecef_node;
  sbp_register_cbk(
      SBP_MSG_BASE_POS_ECEF, &base_pos_ecef_callback, &base_pos_ecef_node);

  static sbp_msg_callbacks_node_t base_glonass_biases_node;
  sbp_register_cbk(SBP_MSG_GLO_BIASES,
                   &base_glonass_biases_callback,
                   &base_glonass_biases_node);

  static sbp_msg_callbacks_node_t obs_packed_node;
  sbp_register_cbk(SBP_MSG_OBS, &obs_callback, &obs_packed_node);

  static sbp_msg_callbacks_node_t deprecated_node;
  sbp_register_cbk(SBP_MSG_OBS_DEP_A, &deprecated_callback, &deprecated_node);

  static sbp_msg_callbacks_node_t deprecated_node_2;
  sbp_register_cbk(SBP_MSG_OBS_DEP_B, &deprecated_callback, &deprecated_node_2);

  static sbp_msg_callbacks_node_t deprecated_node_3;
  sbp_register_cbk(SBP_MSG_OBS_DEP_C, &deprecated_callback, &deprecated_node_3);

  static sbp_msg_callbacks_node_t ics_node;
  sbp_register_cbk(SBP_MSG_GROUP_DELAY, &ics_msg_callback, &ics_node);
}

/** Get a rolling count of received base observations. */
u32 base_obs_msg_counter_get(void) { return base_obs_msg_counter; }

/* \} */
