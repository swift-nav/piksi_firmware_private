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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/sbas_raw_data.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/single_epoch_solver.h>
#include <libswiftnav/troposphere.h>

#include "calc_base_obs.h"
#include "calc_pvt_common.h"
//#include "calc_pvt_starling.h"
#include "calc_pvt_thread_time_matched.h"

#include <starling/platform/mutex.h>
#include <starling/platform/thread.h>

/**
 * Type dependencies.
 */
typedef struct sbp_messages_t sbp_messages_t;

/**
 * Function dependencies.
 */
extern void init_filters(void);
extern void solution_send_pos_messages(u8 base_sender_id,
                                       const sbp_messages_t *sbp_messages,
                                       u8 n_meas,
                                       const navigation_measurement_t nav_meas[]);
extern void process_matched_obs(const obss_t *rover_channel_meass,
                                const obss_t *reference_obss,
                                sbp_messages_t *sbp_messages); 
extern bool update_time_matched(gps_time_t *last_update_time,
                                gps_time_t *current_time,
                                u8 num_obs);
void sbp_messages_init(sbp_messages_t *sbp_messages, gps_time_t *t);
bool spp_timeout(const gps_time_t *_last_spp,
                 const gps_time_t *_last_dgnss,
                 dgnss_solution_mode_t _dgnss_soln_mode);

/**
 * Global variable dependencies.
 */
extern starling_mutex_t *time_matched_filter_manager_lock;
extern FilterManager    *time_matched_filter_manager;
extern gps_time_t last_dgnss;
extern gps_time_t last_spp;
extern gps_time_t last_time_matched_rover_obs_post;
extern double starling_frequency;
extern float glonass_downweight_factor;
extern memory_pool_t time_matched_obs_buff_pool;
extern mailbox_t time_matched_obs_mailbox;
extern dgnss_solution_mode_t dgnss_soln_mode;
extern bool enable_glonass;

/**
 * Time matched thread implementation.
 */
void time_matched_obs_thread(void *arg) {
  (void)arg;
  starling_thread_set_name("time matched obs");

  obss_t *base_obs;
  static obss_t base_obss_copy;
  init_filters();

  // Declare all SBP messages
  sbp_messages_t sbp_messages;

  while (1) {
    base_obs = NULL;
    const msg_t fetch_ret =
        chMBFetch(&base_obs_mailbox, (msg_t *)&base_obs, DGNSS_TIMEOUT_MS);

    if (fetch_ret != MSG_OK) {
      if (NULL != base_obs) {
        log_error("Base obs mailbox fetch failed with %" PRIi32, fetch_ret);
        chPoolFree(&base_obs_buff_pool, base_obs);
      }
      continue;
    }

    if (gps_time_valid(&last_time_matched_rover_obs_post) &&
        gpsdifftime(&last_time_matched_rover_obs_post, &base_obs->tor) >
            BASE_LATENCY_TIMEOUT) {
      log_info("Communication Latency exceeds 15 seconds");
    }

    base_obss_copy = *base_obs;
    chPoolFree(&base_obs_buff_pool, base_obs);

    // Check if the el mask has changed and update
    starling_mutex_lock(time_matched_filter_manager_lock);
    set_pvt_engine_elevation_mask(time_matched_filter_manager,
                                  get_solution_elevation_mask());
    set_pvt_engine_enable_glonass(time_matched_filter_manager, enable_glonass);
    set_pvt_engine_glonass_downweight_factor(time_matched_filter_manager,
                                             glonass_downweight_factor);
    set_pvt_engine_update_frequency(time_matched_filter_manager,
                                    starling_frequency);
    starling_mutex_unlock(time_matched_filter_manager_lock);

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (chMBFetch(&time_matched_obs_mailbox,
                     (msg_t *)&obss,
                     TIME_IMMEDIATE) == MSG_OK) {
      if (dgnss_soln_mode == SOLN_MODE_NO_DGNSS) {
        // Not doing any DGNSS.  Toss the obs away.
        chPoolFree(&time_matched_obs_buff_pool, obss);
        continue;
      }

      double dt = gpsdifftime(&obss->tor, &base_obss_copy.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD && base_obss_copy.has_pos == 1) {
        // We need to form the SBP messages derived from the SPP at this
        // solution time before we
        // do the differential solution so that the various messages can be
        // overwritten as appropriate,
        // the exception is the DOP messages, as we don't have the SPP DOP and
        // it will always be overwritten by the differential
        pvt_engine_result_t soln_copy = obss->soln;

        /* Init the messages we want to send */
        gps_time_t epoch_time = base_obss_copy.tor;
        sbp_messages_init(&sbp_messages, &epoch_time);

        solution_make_sbp(&soln_copy, NULL, &sbp_messages);

        static gps_time_t last_update_time = {.wn = 0, .tow = 0.0};
        if (update_time_matched(&last_update_time, &obss->tor, obss->n) ||
            dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
          process_matched_obs(obss, &base_obss_copy, &sbp_messages);
          last_update_time = obss->tor;
        }

        if (spp_timeout(&last_spp, &last_dgnss, dgnss_soln_mode)) {
          solution_send_pos_messages(
              base_obss_copy.sender_id, &sbp_messages, obss->n, obss->nm);
        }
        chPoolFree(&time_matched_obs_buff_pool, obss);
        break;
      } else {
        if (dt > 0) {
          /* Time of base obs before time of local obs, we must not have a local
           * observation matching this base observation, break and wait for a
           * new base observation. */

          /* In practice this should only happen when we initially start
           * receiving corrections, or if the ntrip/skylark corrections are old
           * due to connection problems.
           */
          log_warn(
              "Obs Matching: t_base < t_rover "
              "(dt=%f obss.t={%d,%f} base_obss.t={%d,%f})",
              dt,
              obss->tor.wn,
              obss->tor.tow,
              base_obss_copy.tor.wn,
              base_obss_copy.tor.tow);
          /* Return the buffer to the mailbox so we can try it again later. */
          const msg_t post_ret = chMBPostAhead(
              &time_matched_obs_mailbox, (msg_t)obss, TIME_IMMEDIATE);
          if (post_ret != MSG_OK) {
            /* Something went wrong with returning it to the buffer, better just
             * free it and carry on. */
            log_warn("Obs Matching: mailbox full, discarding observation!");
            chPoolFree(&time_matched_obs_buff_pool, obss);
          }
          break;
        } else {
          /* Time of base obs later than time of local obs,
           * keep moving through the mailbox. */
          chPoolFree(&time_matched_obs_buff_pool, obss);
        }
      }
    }
  }
}
