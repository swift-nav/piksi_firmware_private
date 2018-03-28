/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/ephemeris.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>

#include <string.h>

#include "starling_platform_shim.h"
#include "starling_threads.h"

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////
#define STARLING_THREAD_PRIORITY (HIGHPRIO - 4)
#define STARLING_THREAD_STACK (6 * 1024 * 1024)

#define TIME_MATCHED_OBS_THREAD_PRIORITY (NORMALPRIO - 3)
#define TIME_MATCHED_OBS_THREAD_STACK (6 * 1024 * 1024)

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////
static PLATFORM_THD_WORKING_AREA(wa_starling_thread, STARLING_THREAD_STACK);
static PLATFORM_THD_WORKING_AREA(wa_time_matched_obs_thread,
                                 TIME_MATCHED_OBS_THREAD_STACK);

static FilterManager *time_matched_filter_manager;
static FilterManager *low_latency_filter_manager;
static FilterManager *spp_filter_manager;

static PLATFORM_MUTEX_DECL(time_matched_filter_manager_lock);
static PLATFORM_MUTEX_DECL(low_latency_filter_manager_lock);
static PLATFORM_MUTEX_DECL(spp_filter_manager_lock);

static PLATFORM_MUTEX_DECL(time_matched_iono_params_lock);
static bool has_time_matched_iono_params = false;
static ionosphere_t time_matched_iono_params;

////////////////////////////////////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////////////////////////////////////

static void init_filters(void) {
  platform_mutex_lock(&time_matched_filter_manager_lock);
  time_matched_filter_manager = create_filter_manager_rtk();
  platform_mutex_unlock(&time_matched_filter_manager_lock);

  platform_mutex_lock(&low_latency_filter_manager_lock);
  low_latency_filter_manager = create_filter_manager_rtk();
  platform_mutex_unlock(&low_latency_filter_manager_lock);
}

static PVT_ENGINE_INTERFACE_RC get_baseline(
    const FilterManager *filter_manager,
    const bool use_time_matched_baseline,
    dops_t *dops,
    pvt_engine_result_t *result) {
  PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;

  get_baseline_ret = filter_manager_get_result(
      filter_manager, use_time_matched_baseline, result);

  if (get_baseline_ret == PVT_ENGINE_SUCCESS) {
    *dops = filter_manager_get_dop_values(filter_manager);
  }

  return get_baseline_ret;
}

static void reset_filters_callback(u16 sender_id,
                                   u8 len,
                                   u8 msg[],
                                   void *context) {
  (void)sender_id;
  (void)len;
  (void)context;
  switch (msg[0]) {
    case 0:
      log_info("Filter reset requested");
      reset_rtk_filter();
      break;
    default:
      break;
  }
}

static PVT_ENGINE_INTERFACE_RC update_filter(FilterManager *filter_manager) {
  PVT_ENGINE_INTERFACE_RC ret = PVT_ENGINE_FAILURE;
  if (filter_manager_is_initialized(filter_manager)) {
    ret = filter_manager_update(filter_manager);
  } else {
    log_info("Starling Filter not initialized.");
  }
  return ret;
}

static bool update_time_matched(gps_time_t *last_update_time,
                                gps_time_t *current_time,
                                u8 num_obs) {
  double update_dt = gpsdifftime(current_time, last_update_time);
  double update_rate_limit = 0.99;
  if (num_obs > 16) {
    update_rate_limit = 1.99;
  }
  if (update_dt < update_rate_limit) {
    return false;
  }
  return true;
}

static PVT_ENGINE_INTERFACE_RC call_pvt_engine_filter(
    FilterManager *filter_manager,
    const gps_time_t *obs_time,
    const u8 num_obs,
    const navigation_measurement_t *nav_meas,
    const ephemeris_t *ephemerides[MAX_CHANNELS],
    const double solution_frequency,
    pvt_engine_result_t *result,
    dops_t *dops) {
  PVT_ENGINE_INTERFACE_RC update_rov_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_filter_ret = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;

  bool is_initialized = filter_manager_is_initialized(filter_manager);

  if (is_initialized) {
    set_pvt_engine_elevation_mask(filter_manager,
                                  get_solution_elevation_mask());
    set_pvt_engine_enable_glonass(filter_manager, enable_glonass);
    set_pvt_engine_glonass_downweight_factor(filter_manager,
                                             glonass_downweight_factor);
    set_pvt_engine_update_frequency(filter_manager, solution_frequency);

    filter_manager_overwrite_ephemerides(filter_manager, ephemerides);

    update_rov_obs = filter_manager_update_rov_obs(
        filter_manager, obs_time, num_obs, nav_meas);
  }

  if (update_rov_obs == PVT_ENGINE_SUCCESS) {
    update_filter_ret = update_filter(filter_manager);
  }

  if (update_filter_ret == PVT_ENGINE_SUCCESS) {
    get_baseline_ret = get_baseline(filter_manager, false, dops, result);
  }

  return get_baseline_ret;
}

static void process_matched_obs(const obss_t *rover_channel_meass,
                                const obss_t *reference_obss,
                                sbp_messages_t *sbp_messages) {
  PVT_ENGINE_INTERFACE_RC update_rov_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_ref_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_filter_ret = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;
  dops_t RTK_dops;
  pvt_engine_result_t result;

  ephemeris_t ephs[MAX_CHANNELS];
  const ephemeris_t *stored_ephs[MAX_CHANNELS];
  memset(stored_ephs, 0, sizeof(stored_ephs));

  for (u8 i = 0; i < rover_channel_meass->n; i++) {
    const navigation_measurement_t *nm = &rover_channel_meass->nm[i];
    if (platform_try_read_ephemeris(nm->sid, &ephs[i])) {
      if (1 == ephemeris_valid(&ephs[i], &nm->tot)) {
        stored_ephs[i] = &ephs[i];
      }
    }
  }

  platform_mutex_lock(&time_matched_filter_manager_lock);

  if (!filter_manager_is_initialized(time_matched_filter_manager)) {
    filter_manager_init(time_matched_filter_manager);
  }

  if (filter_manager_is_initialized(time_matched_filter_manager)) {
    filter_manager_overwrite_ephemerides(time_matched_filter_manager,
                                         stored_ephs);

    platform_mutex_lock(&time_matched_iono_params_lock);
    if (has_time_matched_iono_params) {
      filter_manager_update_iono_parameters(time_matched_filter_manager,
                                            &time_matched_iono_params,
                                            disable_klobuchar);
    }
    platform_mutex_unlock(&time_matched_iono_params_lock);

    update_rov_obs = filter_manager_update_rov_obs(time_matched_filter_manager,
                                                   &rover_channel_meass->tor,
                                                   rover_channel_meass->n,
                                                   rover_channel_meass->nm);
    update_ref_obs = filter_manager_update_ref_obs(
        (FilterManagerRTK *)time_matched_filter_manager,
        &reference_obss->tor,
        reference_obss->n,
        reference_obss->nm,
        reference_obss->pos_ecef);

    if ((update_rov_obs == PVT_ENGINE_SUCCESS ||
         update_rov_obs == PVT_ENGINE_NO_OBS ||
         update_rov_obs == PVT_ENGINE_INSUFFICIENT_OBS) &&
        update_ref_obs == PVT_ENGINE_SUCCESS) {
      update_filter_ret = update_filter(time_matched_filter_manager);
    }

    if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY &&
        update_filter_ret == PVT_ENGINE_SUCCESS) {
      /* If we're in low latency mode we need to copy/update the low latency
         filter manager from the time matched filter manager. */
      platform_mutex_lock(&low_latency_filter_manager_lock);
      platform_mutex_lock(&base_pos_lock);
      platform_mutex_lock(&base_glonass_biases_lock);
      u32 begin = NAP->TIMING_COUNT;
      copy_filter_manager_rtk(
          (FilterManagerRTK *)low_latency_filter_manager,
          (const FilterManagerRTK *)time_matched_filter_manager);
      u32 end = NAP->TIMING_COUNT;
      log_debug("copy_filter_manager_rtk DST %p   SRC %p in %" PRIu32 " ticks",
                low_latency_filter_manager,
                time_matched_filter_manager,
                (end > begin) ? (end - begin) : (begin + (4294967295U - end)));
      current_base_sender_id = reference_obss->sender_id;
      platform_mutex_unlock(&base_glonass_biases_lock);
      platform_mutex_unlock(&base_pos_lock);
      platform_mutex_unlock(&low_latency_filter_manager_lock);
    }
  }

  /* If we are in time matched mode then calculate and output the baseline
   * for this observation. */
  if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED && !simulation_enabled() &&
      update_filter_ret == PVT_ENGINE_SUCCESS) {
    /* Note: in time match mode we send the physically incorrect time of the
     * observation message (which can be receiver clock time, or rounded GPS
     * time) instead of the true GPS time of the solution. */
    result.time = rover_channel_meass->tor;
    result.propagation_time = 0;
    get_baseline_ret =
        get_baseline(time_matched_filter_manager, true, &RTK_dops, &result);
  }

  platform_mutex_unlock(&time_matched_filter_manager_lock);

  if (get_baseline_ret == PVT_ENGINE_SUCCESS) {
    solution_make_baseline_sbp(
        &result, rover_channel_meass->pos_ecef, &RTK_dops, sbp_messages);
  }
}

/** Determine if we have had a SPP timeout.
 *
 * \param _last_spp. Last time of SPP solution
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
static bool spp_timeout(const gps_time_t *_last_spp,
                        const gps_time_t *_last_dgnss,
                        dgnss_solution_mode_t _dgnss_soln_mode) {
  // No timeout needed in low latency mode;
  if (_dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
    return false;
  }
  platform_mutex_lock(&last_sbp_lock);
  double time_diff = gpsdifftime(_last_dgnss, _last_spp);
  platform_mutex_unlock(&last_sbp_lock);

  // Need to compare timeout threshold in MS to system time elapsed (in system
  // ticks)
  return (time_diff > 0.0);
}

////////////////////////////////////////////////////////////////////////////////
// Interface
////////////////////////////////////////////////////////////////////////////////

static void starling_main_thread(void *arg) {
  (void)arg;
  msg_t ret;

  platform_thread_set_name("starling");

  sbp_messages_t sbp_messages;

  static navigation_measurement_t nav_meas[MAX_CHANNELS];
  static ephemeris_t e_meas[MAX_CHANNELS];
  static gps_time_t obs_time;

  platform_mutex_lock(&spp_filter_manager_lock);
  spp_filter_manager = create_filter_manager_spp();
  filter_manager_init(spp_filter_manager);
  platform_mutex_unlock(&spp_filter_manager_lock);

  u8 base_station_sender_id = 0;

  while (TRUE) {
    platform_watchdog_notify_starling_main_thread();

    me_msg_t *me_msg = NULL;
    ret = chMBFetch(&me_msg_mailbox, (msg_t *)&me_msg, DGNSS_TIMEOUT_MS);
    if (ret != MSG_OK) {
      if (NULL != me_msg) {
        log_error("STARLING: mailbox fetch failed with %" PRIi32, ret);
        platform_pool_free(&me_msg_buff_pool, me_msg);
      }
      continue;
    }

    /* forward SBAS raw message to SPP filter manager*/
    if (ME_MSG_SBAS_RAW == me_msg->id) {
      const gps_time_t current_time = get_current_time();
      if (gps_time_valid(&current_time)) {
        sbas_raw_data_t sbas_data;
        unpack_sbas_raw_data(&me_msg->msg.sbas, &sbas_data);

        /* fill the week number from current time */
        gps_time_match_weeks(&sbas_data.time_of_transmission, &current_time);

        sbas_system_t sbas_system = get_sbas_system(sbas_data.sid);

        platform_mutex_lock(&spp_filter_manager_lock);
        if (sbas_system != current_sbas_system &&
            SBAS_UNKNOWN != current_sbas_system) {
          /* clear existing SBAS corrections when provider changes */
          filter_manager_reinitialize_sbas(spp_filter_manager);
        }
        filter_manager_process_sbas_message(spp_filter_manager, &sbas_data);
        platform_mutex_unlock(&spp_filter_manager_lock);

        current_sbas_system = sbas_system;
      }
      platform_pool_free(&me_msg_buff_pool, me_msg);
      continue;
    }

    if (ME_MSG_OBS != me_msg->id) {
      platform_pool_free(&me_msg_buff_pool, me_msg);
      continue;
    }

    me_msg_obs_t *rover_channel_epoch = &me_msg->msg.obs;

    /* Init the messages we want to send */

    gps_time_t epoch_time = rover_channel_epoch->obs_time;
    if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= get_time_quality()) {
      /* observations do not have valid time, but we have a reasonable estimate
       * of current GPS time, so round that to nearest epoch and use it
       */
      epoch_time = get_current_time();
      epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
    }

    sbp_messages_init(&sbp_messages, &epoch_time);

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation(&sbp_messages);
      const u8 fake_base_sender_id = 1;
      solution_send_low_latency_output(fake_base_sender_id,
                                       &sbp_messages,
                                       rover_channel_epoch->size,
                                       rover_channel_epoch->obs);
      platform_pool_free(&me_msg_buff_pool, me_msg);
      continue;
    }

    if (rover_channel_epoch->size == 0 ||
        !gps_time_valid(&rover_channel_epoch->obs_time)) {
      platform_pool_free(&me_msg_buff_pool, me_msg);
      solution_send_low_latency_output(0, &sbp_messages, 0, nav_meas);
      continue;
    }

    if (gpsdifftime(&rover_channel_epoch->obs_time, &obs_time) <= 0.0) {
      /* When we change the solution rate down, we sometimes can round the
       * time to an epoch earlier than the previous one processed, in that
       * case we want to ignore any epochs with an earlier timestamp */
      platform_pool_free(&me_msg_buff_pool, me_msg);
      continue;
    }

    starling_frequency = soln_freq_setting;

    u8 n_ready = rover_channel_epoch->size;
    memset(nav_meas, 0, sizeof(nav_meas));

    if (n_ready) {
      MEMCPY_S(nav_meas,
               sizeof(nav_meas),
               rover_channel_epoch->obs,
               n_ready * sizeof(navigation_measurement_t));
    }

    memset(e_meas, 0, sizeof(e_meas));

    if (n_ready) {
      MEMCPY_S(e_meas,
               sizeof(e_meas),
               rover_channel_epoch->ephem,
               n_ready * sizeof(ephemeris_t));
    }

    obs_time = rover_channel_epoch->obs_time;

    platform_pool_free(&me_msg_buff_pool, me_msg);

    ionosphere_t i_params;
    /* get iono parameters if available, otherwise use default ones */
    if (!platform_try_read_iono_corr(&i_params)) {
      i_params = DEFAULT_IONO_PARAMS;
    }
    platform_mutex_lock(&time_matched_iono_params_lock);
    has_time_matched_iono_params = true;
    time_matched_iono_params = i_params;
    platform_mutex_unlock(&time_matched_iono_params_lock);
    platform_mutex_lock(&spp_filter_manager_lock);
    filter_manager_update_iono_parameters(spp_filter_manager, &i_params, false);
    platform_mutex_unlock(&spp_filter_manager_lock);

    dops_t dops;

    // This will duplicate pointers to satellites with mutliple frequencies,
    // but this scenario is expected and handled
    const ephemeris_t *stored_ephs[MAX_CHANNELS];
    memset(stored_ephs, 0, sizeof(stored_ephs));
    for (u8 i = 0; i < n_ready; i++) {
      navigation_measurement_t *nm = &nav_meas[i];
      ephemeris_t *e = NULL;

      /* Find the original index of this measurement in order to point to
       * the correct ephemeris. (Do not load it again from NDB because it may
       * have changed meanwhile.) */
      for (u8 j = 0; j < n_ready; j++) {
        if (sid_is_equal(nm->sid, e_meas[j].sid)) {
          e = &e_meas[j];
          break;
        }
      }

      if (e == NULL || 1 != ephemeris_valid(e, &nm->tot)) {
        continue;
      }

      stored_ephs[i] = e;
    }

    pvt_engine_result_t result_spp;
    result_spp.valid = false;
    bool successful_spp = false;

    platform_mutex_lock(&spp_filter_manager_lock);
    const PVT_ENGINE_INTERFACE_RC spp_call_filter_ret =
        call_pvt_engine_filter(spp_filter_manager,
                               &obs_time,
                               n_ready,
                               nav_meas,
                               stored_ephs,
                               starling_frequency,
                               &result_spp,
                               &dops);
    platform_mutex_unlock(&spp_filter_manager_lock);

    if (spp_call_filter_ret == PVT_ENGINE_SUCCESS) {
      solution_make_sbp(&result_spp, &dops, &sbp_messages);
      successful_spp = true;
    } else {
      if (dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        /* If we can't report a SPP position, something is wrong and no point
         * continuing to process this epoch - send out solution and
         * observation failed messages if not in time matched mode.
         */
        solution_send_low_latency_output(0, &sbp_messages, n_ready, nav_meas);
      }
      continue;
    }

    if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY && successful_spp) {
      platform_mutex_lock(&low_latency_filter_manager_lock);

      pvt_engine_result_t result_rtk;
      result_rtk.valid = false;
      const PVT_ENGINE_INTERFACE_RC rtk_call_filter_ret =
          call_pvt_engine_filter(low_latency_filter_manager,
                                 &obs_time,
                                 n_ready,
                                 nav_meas,
                                 stored_ephs,
                                 starling_frequency,
                                 &result_rtk,
                                 &dops);
      base_station_sender_id = current_base_sender_id;
      platform_mutex_unlock(&low_latency_filter_manager_lock);

      if (rtk_call_filter_ret == PVT_ENGINE_SUCCESS) {
        solution_make_baseline_sbp(
            &result_rtk, result_spp.baseline, &dops, &sbp_messages);
      }
    }

    /* This is posting the rover obs to the mailbox to the time matched thread,
     * we want these sent on at full rate */
    /* Post the observations to the mailbox. */
    post_observations(n_ready, nav_meas, &obs_time, &result_spp);

    solution_send_low_latency_output(
        base_station_sender_id, &sbp_messages, n_ready, nav_meas);
  }
}

static void time_matched_obs_thread(void *arg) {
  (void)arg;
  platform_thread_set_name("time matched obs");

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
        platform_pool_free(&base_obs_buff_pool, base_obs);
      }
      continue;
    }

    if (gps_time_valid(&last_time_matched_rover_obs_post) &&
        gpsdifftime(&last_time_matched_rover_obs_post, &base_obs->tor) >
            BASE_LATENCY_TIMEOUT) {
      log_info("Communication Latency exceeds 15 seconds");
    }

    base_obss_copy = *base_obs;
    platform_pool_free(&base_obs_buff_pool, base_obs);

    // Check if the el mask has changed and update
    platform_mutex_lock(&time_matched_filter_manager_lock);
    set_pvt_engine_elevation_mask(time_matched_filter_manager,
                                  get_solution_elevation_mask());
    set_pvt_engine_enable_glonass(time_matched_filter_manager, enable_glonass);
    set_pvt_engine_glonass_downweight_factor(time_matched_filter_manager,
                                             glonass_downweight_factor);
    set_pvt_engine_update_frequency(time_matched_filter_manager,
                                    starling_frequency);
    platform_mutex_unlock(&time_matched_filter_manager_lock);

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (chMBFetch(&time_matched_obs_mailbox,
                     (msg_t *)&obss,
                     TIME_IMMEDIATE) == MSG_OK) {
      if (dgnss_soln_mode == SOLN_MODE_NO_DGNSS) {
        // Not doing any DGNSS.  Toss the obs away.
        platform_pool_free(&time_matched_obs_buff_pool, obss);
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
        platform_pool_free(&time_matched_obs_buff_pool, obss);
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
            platform_pool_free(&time_matched_obs_buff_pool, obss);
          }
          break;
        } else {
          /* Time of base obs later than time of local obs,
           * keep moving through the mailbox. */
          platform_pool_free(&time_matched_obs_buff_pool, obss);
        }
      }
    }
  }
}

void starling_calc_pvt_setup() {
  /* Set time of last differential solution in the past. */
  last_dgnss = GPS_TIME_UNKNOWN;
  last_spp = GPS_TIME_UNKNOWN;
  last_time_matched_rover_obs_post = GPS_TIME_UNKNOWN;

  platform_initialize_settings();
  platform_initialize_memory_pools();

  /* Need to init filters here so they exist before we setup SBP callbacks */
  spp_filter_manager = NULL;
  time_matched_filter_manager = NULL;
  low_latency_filter_manager = NULL;

  /* Start solution thread */
  platform_thread_create_static(wa_starling_thread,
                                sizeof(wa_starling_thread),
                                STARLING_THREAD_PRIORITY,
                                starling_main_thread,
                                NULL);
  platform_thread_create_static(wa_time_matched_obs_thread,
                                sizeof(wa_time_matched_obs_thread),
                                TIME_MATCHED_OBS_THREAD_PRIORITY,
                                time_matched_obs_thread,
                                NULL);

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
      SBP_MSG_RESET_FILTERS, &reset_filters_callback, &reset_filters_node);
}

void reset_rtk_filter(void) {
  platform_mutex_lock(&time_matched_filter_manager_lock);
  filter_manager_init(time_matched_filter_manager);
  platform_mutex_unlock(&time_matched_filter_manager_lock);
}

void set_known_ref_pos(const double base_pos[3]) {
  if (time_matched_filter_manager) {
    filter_manager_set_known_ref_pos(
        (FilterManagerRTK *)time_matched_filter_manager, base_pos);
  }
}

void set_known_glonass_biases(const glo_biases_t biases) {
  if (time_matched_filter_manager) {
    filter_manager_set_known_glonass_biases(
        (FilterManagerRTK *)time_matched_filter_manager, biases);
  }
}

void starling_threads_set_enable_fix_mode(bool enable_fix) {
  platform_mutex_lock(&time_matched_filter_manager_lock);
  if (time_matched_filter_manager) {
    set_pvt_engine_enable_fix_mode(time_matched_filter_manager, enable_fix);
  }
  platform_mutex_unlock(&time_matched_filter_manager_lock);
  platform_mutex_lock(&low_latency_filter_manager_lock);
  if (low_latency_filter_manager) {
    set_pvt_engine_enable_fix_mode(low_latency_filter_manager, enable_fix);
  }
  platform_mutex_unlock(&low_latency_filter_manager_lock);
}

void starling_threads_set_max_age(int value) {
  platform_mutex_lock(&low_latency_filter_manager_lock);
  if (low_latency_filter_manager) {
    set_max_correction_age(low_latency_filter_manager, value);
  }
  platform_mutex_unlock(&low_latency_filter_manager_lock);
  platform_mutex_lock(&time_matched_filter_manager_lock);
  if (time_matched_filter_manager) {
    set_max_correction_age(time_matched_filter_manager, value);
  }
  platform_mutex_unlock(&time_matched_filter_manager_lock);
}
