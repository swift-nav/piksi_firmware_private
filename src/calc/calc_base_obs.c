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

#include "calc_base_obs.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/correct_iono_tropo.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/sid_set.h>
#include <swiftnav/signal.h>

#include "acq/manage.h"
#include "calc/calc_pvt_me.h"
#include "nav_msg/cnav_msg_storage.h"
#include "ndb/ndb.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings_client.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator/simulator.h"
#include "starling_integration.h"
#include "timing/timing.h"

/* Minimum interval between two received observations in seconds.
 * Intermediate messages are discarded.
 */
#define TOR_THRESHOLD_SOLN_MODE_LOW_LATENCY 0.95
#define TOR_THRESHOLD_SOLN_MODE_TIMEMATCHED 0.15

/** \defgroup base_obs Base station observation handling
 * \{ */

static u32 base_obs_msg_counter = 0;
static u16 base_sender_id = 0;

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

  if (base_sender_id != 0 && base_sender_id != sender_id) {
    pvt_driver_reset_time_matched_filter(pvt_driver);
    pvt_driver_reset_low_latency_filter(pvt_driver);
    log_info(
        "Differential filters reset due to changing base position ID: old: %d, "
        "new: %d.",
        base_sender_id,
        sender_id);
  }
  base_sender_id = sender_id;

  pvt_driver_set_known_ref_pos(pvt_driver, base_pos);
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

  if (base_sender_id != 0 && base_sender_id != sender_id) {
    pvt_driver_reset_time_matched_filter(pvt_driver);
    pvt_driver_reset_low_latency_filter(pvt_driver);
    log_info(
        "Differential filters reset due to changing base position ID: old: %d, "
        "new: %d.",
        base_sender_id,
        sender_id);
  }
  base_sender_id = sender_id;

  pvt_driver_set_known_ref_pos(pvt_driver, base_pos);
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
  sbp_unpack_glonass_biases_content(*(msg_glo_biases_t *)msg, &biases);

  pvt_driver_set_known_glonass_biases(pvt_driver, &biases);
  /* Relay base station GLONASS biases using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_GLO_BIASES, len, msg, MSG_FORWARD_SENDER_ID);
}

/* Check that a given time is aligned (within some tolerance) to the
 * local solution epoch. */
static bool is_time_aligned_to_local_epoch(const gps_time_t *t) {
  gps_time_t epoch = gps_time_round_to_epoch(t, soln_freq_setting);
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

/* Returns minimum intervals between to received observations, depending
 * on the Starling solution mode.
 */
static double tor_interval_limit(void) {
  pvt_driver_solution_mode_t mode = pvt_driver_get_solution_mode(pvt_driver);
  if (mode == PVT_DRIVER_SOLN_MODE_LOW_LATENCY) {
    return TOR_THRESHOLD_SOLN_MODE_LOW_LATENCY;
  }
  if (mode == PVT_DRIVER_SOLN_MODE_TIME_MATCHED) {
    return TOR_THRESHOLD_SOLN_MODE_TIMEMATCHED;
  }
  return 0.0;
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
 *
 * \note This function takes ownership of `obs_array`
 */
static void update_obss(obs_array_t *obs_array) {
  /* Warn on receiving observations which are very old. This may be indicative
   * of a connectivity problem. Obviously, if we don't have a good local time
   * estimate, then we can't perform this check. */
  gps_time_t now = get_current_time();
  if (get_time_quality() > TIME_UNKNOWN &&
      gpsdifftime(&now, &obs_array->t) > BASE_LATENCY_TIMEOUT) {
    log_info("Communication latency exceeds 15 seconds");
  }

  bool ret = pvt_driver_send_base_obs(pvt_driver, obs_array);
  if (!ret) {
    log_error("BASE: Unable to send observations.");
  }
}

typedef void (*unpack_all_f)(const u8 msg[], u8 len, obs_array_t *obs_array);
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
static void generic_obs_callback(u16 relay_msg_type,
                                 u16 sender_id,
                                 u8 len,
                                 u8 msg[],
                                 void *context,
                                 unpack_all_f unpack) {
  (void)context;

  /* An SBP sender ID of zero means that the messages are relayed observations
   * from the console, not from the base station. We don't want to use them and
   * we don't want to create an infinite loop by forwarding them again so just
   * ignore them. */
  if (MSG_FORWARD_SENDER_ID == sender_id) {
    return;
  }

  /* Relay observations using sender_id = 0. */
  sbp_send_msg_(relay_msg_type, len, msg, MSG_FORWARD_SENDER_ID);

  /* No use processing base observations if receiver time is still unknown */
  if (TIME_UNKNOWN == get_time_quality()) {
    return;
  }

  /* GPS time of observation. */
  gps_time_t tor = GPS_TIME_UNKNOWN;
  /* Total number of messages in the observation set / sequence. */
  u8 total;
  /* The current message number in the sequence. */
  u8 count;

  /* Decode the message header to get the time and how far through the sequence
   * we are. */
  unpack_obs_header((observation_header_t *)msg, &tor, &total, &count);

  /* if tor is invalid this obs message was sent before a time was solved, so we
   * should ignore */
  if (!gps_time_valid(&tor)) {
    return;
  }

  /* Check that the base station's messages align well enough to our local
   * processing epochs - this will discard *.[1-9]00 if nav is at 1 Hz */
  if (!is_time_aligned_to_local_epoch(&tor)) {
    if (is_first_message_in_obs_sequence(count)) {
      log_info("Unaligned observation from base ignored, tow = %.3f", tor.tow);
    }
    return;
  }

  /* Check that messages are in chronological order and not too close in time */
  static gps_time_t tor_old = GPS_TIME_UNKNOWN;
  if (gps_time_valid(&tor_old)) {
    double tor_diff = gpsdifftime(&tor, &tor_old);
    bool same_epoch = (fabs(tor_diff) < FLOAT_EQUALITY_EPS);
    bool too_early = (tor_diff <= tor_interval_limit());
    /* if too close in time skip this message */
    if (!same_epoch && too_early) {
      return;
    }
    /* enough time has lapsed, update `tor_old` on the first message */
    if (is_first_message_in_obs_sequence(count)) {
      tor_old = tor;
    }
  } else {
    tor_old = tor;
  }

  /* Keep track of where in the sequence of messages we were last time around
   * so we can verify we haven't dropped a message. */
  static s16 prev_count = 0;
  static gps_time_t prev_tor = GPS_TIME_UNKNOWN;
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

  static obs_array_t *obs_array = NULL;
  /* Make sure we have a valid array before operating on it */
  if (NULL == obs_array) {
    obs_array = pvt_driver_alloc_base_obs(pvt_driver);
    if (NULL == obs_array) {
      log_error(
          "Unable to allocate an array to store base obs, dropping one packet");
      return;
    }
  }

  /* If this is the first packet in the sequence then reset the base_obss_rx
   * state. */
  if (is_first_message_in_obs_sequence(count)) {
    obs_array->n = 0;
    obs_array->t = tor;
  }
  obs_array->sender = sender_id;

  unpack(msg, len, obs_array);

  if (base_sender_id != 0 && base_sender_id != sender_id) {
    pvt_driver_reset_time_matched_filter(pvt_driver);
    pvt_driver_reset_low_latency_filter(pvt_driver);
    log_info(
        "Differential filters reset due to changing base observation ID: old: "
        "%d, new: %d.",
        base_sender_id,
        sender_id);
  }
  base_sender_id = sender_id;

  /* If we can, and all the obs have been received, update to using the new
   * obss. */
  if (is_final_message_in_obs_sequence(count, total)) {
    update_obss(obs_array); /* Transferring ownership of obs_array here */
    obs_array = NULL;

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

static void unpack_obs(const u8 msg[], u8 len, obs_array_t *obs_array) {
  if (obs_array->n != 0 && obs_array->is_osr) {
    log_error(
        "Receiving regular obs when we have previously processed OSR messages, "
        "ignoring regular obs");
    return;
  }
  obs_array->is_osr = false;

  /* Calculate the number of observations in this message by looking at the SBP
   * `len` field. */
  u8 obs_in_msg =
      (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

  /* Pull out the contents of the message. */
  packed_obs_content_t *msg_packed_obs =
      (packed_obs_content_t *)(msg + sizeof(observation_header_t));

  /* Copy into local array. */
  for (size_t i = 0;
       i < obs_in_msg && obs_array->n < MAX_INPUT_OBSERVATION_COUNT;
       ++i) {
    starling_obs_t *current_obs = &obs_array->observations[obs_array->n++];
    unpack_obs_content_into_starling_obs(&msg_packed_obs[i], current_obs);
  }
}

static void unpack_osr(const u8 msg[], u8 len, obs_array_t *obs_array) {
  if (obs_array->n != 0 && !obs_array->is_osr) {
    log_error(
        "Receiving OSR messages when we have previously processed regular obs, "
        "ignoring OSR messages");
    return;
  }
  obs_array->is_osr = true;

  /* Calculate the number of observations in this message by looking at the SBP
   * `len` field. */
  u8 obs_in_msg =
      (len - sizeof(observation_header_t)) / sizeof(packed_osr_content_t);

  /* Pull out the contents of the message. */
  packed_osr_content_t *msg_packed_osr =
      (packed_osr_content_t *)(msg + sizeof(observation_header_t));

  /* Copy into local array. */
  for (size_t i = 0;
       i < obs_in_msg && obs_array->n < STARLING_MAX_CHANNEL_COUNT;
       ++i) {
    starling_obs_t *current_obs = &obs_array->observations[obs_array->n++];
    unpack_osr_content(&msg_packed_osr[i], current_obs);
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
  generic_obs_callback(SBP_MSG_OBS, sender_id, len, msg, context, unpack_obs);
}

static void osr_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  generic_obs_callback(SBP_MSG_OSR, sender_id, len, msg, context, unpack_osr);
}

/** SBP callback for the old style observation messages.
 * Just logs a deprecation warning. */
static void deprecated_callback(u16 sender_id,
                                u8 len,
                                u8 msg[], /* NOLINT */
                                void *context) {
  (void)context;
  (void)len;
  (void)msg;
  (void)sender_id;
  log_warn(
      "Received a deprecated obs msg. Verify firmware version on remote "
      "Piksi.");
}

/** SBP callback for Group delay message */
static void ics_msg_callback(u16 sender_id,
                             u8 len,
                             u8 msg[], /* NOLINT */
                             void *context) {
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

  /* relay message */
  sbp_send_msg_(SBP_MSG_GROUP_DELAY, len, msg, MSG_FORWARD_SENDER_ID);
}

/** Setup the base station observation handling subsystem. */
void base_obs_setup() {
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

  static sbp_msg_callbacks_node_t osr_packed_node;
  sbp_register_cbk(SBP_MSG_OSR, &osr_callback, &osr_packed_node);

  static sbp_msg_callbacks_node_t deprecated_node;
  sbp_register_cbk(SBP_MSG_OBS_DEP_A, &deprecated_callback, &deprecated_node);

  static sbp_msg_callbacks_node_t deprecated_node_2;
  sbp_register_cbk(SBP_MSG_OBS_DEP_B, &deprecated_callback, &deprecated_node_2);

  static sbp_msg_callbacks_node_t deprecated_node_3;
  sbp_register_cbk(SBP_MSG_OBS_DEP_C, &deprecated_callback, &deprecated_node_3);

  static sbp_msg_callbacks_node_t ics_node;
  sbp_register_cbk(SBP_MSG_GROUP_DELAY, &ics_msg_callback, &ics_node);
}

/* \} */
