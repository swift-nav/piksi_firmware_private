/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/signal.h>

#include "peripherals/leds.h"
#include "position.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "solution.h"
#include "manage.h"
#include "simulator.h"
#include "settings.h"
#include "timing.h"
#include "base_obs.h"
#include "ephemeris.h"
#include "signal.h"
#include "ndb.h"
#include "shm.h"
#include "cnav_msg_storage.h"

extern bool disable_raim;

/** \defgroup base_obs Base station observation handling
 * \{ */

/** Mutex to control access to the base station observations. */
MUTEX_DECL(base_obs_lock);
/** Semaphore that is flagged when a new set of observations are received. */
BSEMAPHORE_DECL(base_obs_received, TRUE);
/** Most recent observations from the base station. */
obss_t base_obss;

/** Mutex to control access to the base station position state.
 * (#base_pos_ecef and #base_pos_known) */
MUTEX_DECL(base_pos_lock);
/** Is the base station position known? i.e. is #base_pos_ecef valid? */
bool base_pos_known = false;
/** Base station known position in ECEF as sent to us by the base station in
 * the BASE_POS message.  */
double base_pos_ecef[3];

static bool old_base_pos_known = false;
static double old_base_pos_ecef[3] = {0, 0, 0};

static u32 base_obs_msg_counter = 0;

void check_base_position_change(void)
{
  /* Check if the base position has changed and reset the RTK filter if
   * it has.
   */
  if (old_base_pos_known &&
     ((fabs(old_base_pos_ecef[0] - base_pos_ecef[0]) > 1e-3) ||
      (fabs(old_base_pos_ecef[1] - base_pos_ecef[1]) > 1e-3) ||
      (fabs(old_base_pos_ecef[2] - base_pos_ecef[2]) > 1e-3))) {
    log_warn("Base station position changed. Resetting RTK filter.");
    reset_rtk_filter();
    base_pos_known = false;
    memset(&base_pos_ecef, 0, sizeof(base_pos_ecef));
  }
  old_base_pos_known = base_pos_known;
  memcpy(&old_base_pos_ecef, &base_pos_ecef, sizeof(base_pos_ecef));
}

/** SBP callback for when the base station sends us a message containing its
 * known location in LLH coordinates.
 * Stores the base station position in the global #base_pos_ecef variable and
 * sets #base_pos_known to `true`.
 */
static void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len;
  // Skip forwarded sender_ids. See note in obs_callback about echo'ing
  // sender_id.
  if (sender_id == 0) {
    return;
  }
  /*TODO: keep track of sender_id to store multiple base positions?*/
  double llh_degrees[3];
  double llh[3];
  memcpy(llh_degrees, msg, 3 * sizeof(double));
  llh[0] = llh_degrees[0] * D2R;
  llh[1] = llh_degrees[1] * D2R;
  llh[2] = llh_degrees[2];
  chMtxLock(&base_pos_lock);
  wgsllh2ecef(llh, base_pos_ecef);
  base_pos_known = true;
  check_base_position_change();
  /* Relay base station position using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_BASE_POS_LLH, len, msg, 0);
  chMtxUnlock(&base_pos_lock);
}

/** SBP callback for when the base station sends us a message containing its
 * known location in ECEF coordinates.
 * Stores the base station position in the global #base_pos_ecef variable and
 * sets #base_pos_known to `true`.
 */
static void base_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len;
  // Skip forwarded sender_ids. See note in obs_callback about echo'ing
  // sender_id.
  if (sender_id == 0) {
    return;
  }
  chMtxLock(&base_pos_lock);
  memcpy(base_pos_ecef, msg, 3 * sizeof(double));
  base_pos_known = true;
  check_base_position_change();
  /* Relay base station position using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_BASE_POS_ECEF, len, msg, 0);
  chMtxUnlock(&base_pos_lock);
}

static inline bool not_l2p_sid(navigation_measurement_t a) {
  return a.sid.code != CODE_GPS_L2P;
}

static inline bool shm_suitable_wrapper(navigation_measurement_t meas) {
  return shm_navigation_suitable(meas.sid);
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
static void update_obss(obss_t *new_obss)
{
  static gps_time_t tor_old = {.wn = 0, .tow = 0};

  /* We don't want to allow observations that have the same or earlier time
   * stamp than the last received */
  if ( gpsdifftime(&new_obss->tor, &tor_old) <= 0) {
    log_info("Observation received with equal or earlier time stamp, ignoring");
    return;
  }

  /* Ensure observations sorted by PRN. */
  qsort(new_obss->nm, new_obss->n,
        sizeof(navigation_measurement_t), nav_meas_cmp);
  /** Precheck any base station observations and filter if needed. This is not a
   *  permanent solution for actually correcting GPS L2 base station
   *  observations that have mixed tracking modes in a signal epoch. For more
   *  details, see:
   *  https://github.com/swift-nav/estimation_team_planning/issues/215.
   */
  if (new_obss->n > 0 && has_mixed_l2_obs(new_obss->n, new_obss->nm)) {
    log_warn("Base observations have mixed L2 tracking types. Discarding L2P!");
    new_obss->n = filter_nav_meas(new_obss->n, new_obss->nm, not_l2p_sid);
  }

  /* Filter out any observation without a valid pseudorange observation. */
  if (new_obss->n > 0) {
    new_obss->n = filter_nav_meas(new_obss->n, new_obss->nm, pseudorange_valid);
  }

  /* Filter out any observation not marked healthy by the ndb. */
  if (new_obss->n > 0) {
    new_obss->n =
        filter_nav_meas(new_obss->n, new_obss->nm, shm_suitable_wrapper);
  }

  /* Lock mutex before modifying base_obss.
   * NOTE: We didn't need to lock it before reading in THIS context as this
   * is the only thread that writes to base_obss. */
  bool have_obs = true;
  chMtxLock(&base_obs_lock);

  /* Create a set of navigation measurements to store the previous
   * observations. */
  static u8 n_old = 0;
  static navigation_measurement_t nm_old[MAX_CHANNELS];

  /* Fill in the navigation measurements in base_obss, using TDCP method to
   * calculate the Doppler shift. */
  base_obss.n = tdcp_doppler(new_obss->n, new_obss->nm,
                             n_old, nm_old, base_obss.nm,
                             gpsdifftime(&new_obss->tor, &tor_old));

  /* Assume that we don't know the known, surveyed base position for now. */
  base_obss.has_known_pos_ecef = false;

  /* Copy over sender ID. */
  base_obss.sender_id = new_obss->sender_id;

  /* Copy the current observations over to nm_old so we can difference
   * against them next time around. */
  memcpy(nm_old, new_obss->nm,
         new_obss->n * sizeof(navigation_measurement_t));
  n_old = new_obss->n;
  tor_old = new_obss->tor;

  /* Copy over the time. */
  base_obss.tor = new_obss->tor;

  if (base_obss.n >= 4) {
    gnss_solution soln;
    dops_t dops;

    /* check if we have fix, if yes, calculate iono and tropo correction */
    if(base_obss.has_pos) {
      double llh[3];
      wgsecef2llh(base_obss.pos_ecef, llh);
      log_debug("Base: IONO/TROPO correction");
      ionosphere_t i_params;
      ionosphere_t *p_i_params = &i_params;
      /* get iono parameters if available */
      if(ndb_iono_corr_read(p_i_params) != NDB_ERR_NONE) {
        p_i_params = NULL;
      }
      calc_iono_tropo(base_obss.n, base_obss.nm, base_obss.pos_ecef, llh,
                      p_i_params);
    }

    /* Calculate a position solution. */
    /* disable_raim controlled by external setting (see solution.c). */
    /* Skip velocity solving for the base incase we have bad doppler values
     * due to a cycle slip. */
    s32 ret = calc_PVT(base_obss.n, base_obss.nm, disable_raim, true,
                       get_solution_elevation_mask(), &soln, &dops, NULL);

    if (ret >= 0 && soln.valid) {
      memcpy(base_obss.pos_ecef, soln.pos_ecef, sizeof(soln.pos_ecef));
      base_obss.has_pos = 1;

      chMtxLock(&base_pos_lock);
      if (base_pos_known) {
        double base_distance = vector_distance(3, soln.pos_ecef, base_pos_ecef);

        if (base_distance > BASE_STATION_RESET_THRESHOLD) {
          log_warn("Received base observation with SPP position %f m from the"
                   " surveyed position. Resetting RTK filter.",
                   base_distance);
         reset_rtk_filter();
         base_pos_known = false;
         memset(&base_pos_ecef, 0, sizeof(base_pos_ecef));
        } else {
          /* If our known base position is consistent with our calculated base
             SPP, save the known base position. This will be used for
             calculating the pseudo-absolute position later. However, if the
             difference between the two positions is larger than expected, log
             a warning, but still use the known base position. */
          if (base_distance > BASE_STATION_DISTANCE_THRESHOLD) {
            log_warn("Received base observation with SPP position %f m from the"
                     " surveyed position. Check the base station position setting.",
                     base_distance);
          }
          memcpy(base_obss.known_pos_ecef, base_pos_ecef, sizeof(base_pos_ecef));
          base_obss.has_known_pos_ecef = true;
        }
      } else {
        base_obss.has_known_pos_ecef = false;
      }
      chMtxUnlock(&base_pos_lock);
    } else {
      base_obss.has_pos = 0;
      /* TODO(dsk) check for repair failure */
      /* There was an error calculating the position solution. */
      log_warn("Error calculating base station position: (%s).", pvt_err_msg[-ret-1]);
    }
  } else {
    base_obss.has_pos = 0;
  }

  /* If the surveyed base station position is known then use it to calculate
     the satellite ranges. Otherwise, use the SPP. This calculation will be
     used later by the propagation functions. */
  if (base_obss.has_pos) {
    for (u8 i=0; i < base_obss.n; i++) {
      /* The nominal initial "sat_dist" contains the distance
         from the base position to the satellite position as well as the
         satellite clock error. */
      base_obss.sat_dists[i] = vector_distance(3, base_obss.nm[i].sat_pos,
                                               base_obss.has_known_pos_ecef ?
                                               base_obss.known_pos_ecef :
                                               base_obss.pos_ecef)
                                               - base_obss.nm[i].sat_clock_err * GPS_C;
    }
  }
  /* Unlock base_obss mutex. */
  chMtxUnlock(&base_obs_lock);

  /* Signal that a complete base observation has been received. */
  if (have_obs) {
    chBSemSignal(&base_obs_received);
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
static void obs_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context;

  /* Keep track of where in the sequence of messages we were last time around
   * so we can verify we haven't dropped a message. */
  static s16 prev_count = 0;

  static gps_time_t prev_tor = {.tow = 0.0, .wn = 0};

  /* As we receive observation messages we assemble them into a working
   * `obss_t` (`base_obss_rx`) so as not to disturb the global `base_obss`
   * state that may be in use. */
  static obss_t base_obss_rx = {.has_pos = 0};

  /* An SBP sender ID of zero means that the messages are relayed observations
   * from the console, not from the base station. We don't want to use them and
   * we don't want to create an infinite loop by forwarding them again so just
   * ignore them. */
  if (sender_id == 0) {
    return;
  }

  /* We set the sender_id */
  base_obss_rx.sender_id = sender_id;

  /* Relay observations using sender_id = 0. */
  sbp_send_msg_(SBP_MSG_OBS, len, msg, 0);

  /* GPS time of observation. */
  gps_time_t tor;
  /* Total number of messages in the observation set / sequence. */
  u8 total;
  /* The current message number in the sequence. */
  u8 count;

  /* Decode the message header to get the time and how far through the sequence
   * we are. */
  unpack_obs_header((observation_header_t*)msg, &tor, &total, &count);
  /* Check to see if the observation is aligned with our internal observations,
   * i.e. is it going to time match one of our local obs. */
  u32 obs_freq = soln_freq / obs_output_divisor;
  double epoch_count = tor.tow * obs_freq;
  double dt = fabs(epoch_count - round(epoch_count)) / obs_freq;
  if (dt > TIME_MATCH_THRESHOLD) {
    log_warn("Unaligned observation from base station ignored, "
             "tow = %.3f, dt = %.3f", tor.tow, dt);
    return;
  }

  /* Verify sequence integrity */
  if (count == 0) {
    prev_tor = tor;
    prev_count = 0;
  } else if (prev_tor.tow != tor.tow ||
             prev_tor.wn != tor.wn ||
             prev_count + 1 != count) {
    log_info("Dropped one of the observation packets! Skipping this sequence.");
    prev_count = -1;
    return;
  } else {
    prev_count = count;
  }

  /* Calculate the number of observations in this message by looking at the SBP
   * `len` field. */
  u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);

  /* If this is the first packet in the sequence then reset the base_obss_rx
   * state. */
  if (count == 0) {
    base_obss_rx.n = 0;
    base_obss_rx.tor = tor;
  }

  /* Pull out the contents of the message. */
  packed_obs_content_t *obs = (packed_obs_content_t *)(msg + sizeof(observation_header_t));
  for (u8 i=0; i<obs_in_msg; i++) {
    gnss_signal_t sid = sid_from_sbp16(obs[i].sid);
    if (!sid_supported(sid)) {
      continue;
    }
    /* Flag this as visible/viable to acquisition/search */
    manage_set_obs_hint(sid);

    navigation_measurement_t *nm = &base_obss_rx.nm[base_obss_rx.n];

    /* Unpack the observation into a navigation_measurement_t. */
    unpack_obs_content(&obs[i], &nm->raw_pseudorange, &nm->raw_carrier_phase,
                       &nm->raw_measured_doppler, &nm->cn0, &nm->lock_time,
                       &nm->flags, &nm->sid);

    /* Set the time */
    nm->tot = tor;
    nm->tot.tow -= nm->raw_pseudorange / GPS_C;
    normalize_gps_time(&nm->tot);

    /* Calculate satellite parameters using the ephemeris. */
    ephemeris_t ephe;
    ndb_ephemeris_read(nm->sid, &ephe);
    u8 eph_valid;
    s8 cscc_ret, css_ret;
    double clock_rate_err;
    const ephemeris_t *ephe_p = &ephe;

    eph_valid = ephemeris_valid(&ephe, &nm->tot);
    if (eph_valid) {
      /* Apply corrections to the pseudorange, carrier phase and Doppler. */
      cscc_ret = calc_sat_clock_corrections(1, &nm, &ephe_p);

      /* After correcting the time of transmission for the satellite clock error,
         recalculate the satellite position. */
      css_ret = calc_sat_state(&ephe, &nm->tot, nm->sat_pos, nm->sat_vel,
                               &nm->sat_clock_err, &clock_rate_err);
    }

    if (!eph_valid || (cscc_ret != 0) || (css_ret != 0)) {
      continue;
    }

    base_obss_rx.n++;
  }

  /* If we can, and all the obs have been received, update to using the new
   * obss. */
  if (count == total - 1) {
    update_obss(&base_obss_rx);
    /* Calculate packet latency. */
    if (time_quality >= TIME_COARSE) {
      gps_time_t now = get_current_time();
      float latency_ms = (float) ((now.tow - tor.tow) * 1000.0);
      log_obs_latency(latency_ms);
    }
    /* Update message counter */
    base_obs_msg_counter++;
  }
}

/** SBP callback for the old style observation messages.
 * Just logs a deprecation warning. */
static void deprecated_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void) context; (void) len; (void) msg; (void) sender_id;
  log_warn("Received a deprecated obs msg. Verify firmware version on remote Piksi.");
}

/** SBP callback for Group delay message
 */
static void ics_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  log_info("Group delay received from peer");

  cnav_msg_t cnav;
  memset(&cnav, 0, sizeof(cnav));

  /* unpack received message */
  cnav.data.type_30.isc_l1ca = ((msg_group_delay_t*)msg)->isc_l1ca;
  cnav.data.type_30.tgd = ((msg_group_delay_t*)msg)->tgd;
  cnav.prn = ((msg_group_delay_t*)msg)->prn;
  cnav.data.type_30.isc_l1ca_valid = (((msg_group_delay_t*)msg)->valid >> 2) & 0x1;
  cnav.data.type_30.isc_l2c_valid = (((msg_group_delay_t*)msg)->valid >> 1) & 0x1;
  cnav.data.type_30.tgd_valid = (((msg_group_delay_t*)msg)->valid) & 0x1;
  cnav.tow = ((msg_group_delay_t*)msg)->t_op.tow;
  cnav.alert = 0;
  cnav.bit_polarity = 0;
  cnav.msg_id = CNAV_MSG_TYPE_30;

  /* store received CNAV message */
  cnav_msg_put(&cnav);
}

/** Setup the base station observation handling subsystem. */
void base_obs_setup()
{
  /* Register callbacks on base station messages. */

  static sbp_msg_callbacks_node_t base_pos_llh_node;
  sbp_register_cbk(
    SBP_MSG_BASE_POS_LLH,
    &base_pos_llh_callback,
    &base_pos_llh_node
  );

  static sbp_msg_callbacks_node_t base_pos_ecef_node;
  sbp_register_cbk(
    SBP_MSG_BASE_POS_ECEF,
    &base_pos_ecef_callback,
    &base_pos_ecef_node
  );

  static sbp_msg_callbacks_node_t obs_packed_node;
  sbp_register_cbk(
    SBP_MSG_OBS,
    &obs_callback,
    &obs_packed_node
  );

  static sbp_msg_callbacks_node_t deprecated_node;
  sbp_register_cbk(
    SBP_MSG_OBS_DEP_A,
    &deprecated_callback,
    &deprecated_node
  );

  static sbp_msg_callbacks_node_t deprecated_node_2;
  sbp_register_cbk(
    SBP_MSG_OBS_DEP_B,
    &deprecated_callback,
    &deprecated_node_2
  );

  static sbp_msg_callbacks_node_t deprecated_node_3;
  sbp_register_cbk(
    SBP_MSG_OBS_DEP_C,
    &deprecated_callback,
    &deprecated_node_3
  );

  static sbp_msg_callbacks_node_t ics_node;
  sbp_register_cbk(
    SBP_MSG_GROUP_DELAY,
    &ics_msg_callback,
    &ics_node
  );
}

/** Get a rolling count of received base observations. */
u32 base_obs_msg_counter_get(void)
{
  return base_obs_msg_counter;
}

/* \} */
