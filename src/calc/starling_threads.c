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

#include <libsbp/sbp.h>
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

#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "me_msg/me_msg.h"
#include "starling_platform_shim.h"
#include "starling_threads.h"

extern void starling_integration_solution_send_low_latency_output(
    u8 base_sender_id,
    const sbp_messages_t *sbp_messages,
    u8 n_meas,
    const navigation_measurement_t nav_meas[]);

extern void starling_integration_sbp_messages_init(sbp_messages_t *sbp_messages,
                                                   const gps_time_t *t);

extern void starling_integration_solution_simulation(
    sbp_messages_t *sbp_messages);

#define TIME_MATCHED_OBS_THREAD_PRIORITY (NORMALPRIO - 3)
#define TIME_MATCHED_OBS_THREAD_STACK (6 * 1024 * 1024)

/* Settings which control the filter behavior of the Starling engine. */
typedef struct StarlingSettings {
  bool is_glonass_enabled;
  bool is_time_matched_klobuchar_enabled;
  float glonass_downweight_factor;
  float elevation_mask;
  dgnss_solution_mode_t solution_output_mode;
} StarlingSettings;

/* Initial settings values (internal to Starling). */
#define INIT_IS_GLONASS_ENABLED true
#define INIT_IS_TIME_MATCHED_KLOBUCHAR_ENABLED true
#define INIT_GLONASS_DOWNWEIGHT_FACTOR 4.0f
#define INIT_ELEVATION_MASK 10.0f
#define INIT_SOLUTION_OUTPUT_MODE STARLING_SOLN_MODE_LOW_LATENCY

/* Local settings object and mutex protection. */
static MUTEX_DECL(global_settings_lock);
static StarlingSettings global_settings = {
    .is_glonass_enabled = INIT_IS_GLONASS_ENABLED,
    .is_time_matched_klobuchar_enabled = INIT_IS_TIME_MATCHED_KLOBUCHAR_ENABLED,
    .glonass_downweight_factor = INIT_GLONASS_DOWNWEIGHT_FACTOR,
    .elevation_mask = INIT_ELEVATION_MASK,
    .solution_output_mode = INIT_SOLUTION_OUTPUT_MODE,
};

static FilterManager *time_matched_filter_manager = NULL;
static FilterManager *low_latency_filter_manager = NULL;
static FilterManager *spp_filter_manager = NULL;

static MUTEX_DECL(time_matched_filter_manager_lock);
static MUTEX_DECL(low_latency_filter_manager_lock);
static MUTEX_DECL(spp_filter_manager_lock);

static MUTEX_DECL(time_matched_iono_params_lock);
static bool has_time_matched_iono_params = false;
static ionosphere_t time_matched_iono_params;

static gps_time_t last_time_matched_rover_obs_post;

static double starling_frequency;

static sbas_system_t current_sbas_system = SBAS_UNKNOWN;

/**
 * Use this to update the settings for the provided filter manager
 * from any thread. This will apply the current value for
 * each of the settings to the given filter manager.
 *
 * The caller should take care to correctly synchronize access
 * to the provided filter manager. It is an error to call with
 * an invalid Filter Manager pointer.
 *
 * TODO(kevin) Once this lives inside LSNP, it may be better
 * to avoid synchronizing (and copying) the entire settings
 * struct every time and instead use C++ atomics for each
 * individual setting.
 */
static void update_filter_manager_settings(FilterManager *fm) {
  platform_mutex_lock(&global_settings_lock);
  StarlingSettings settings = global_settings;
  platform_mutex_unlock(&global_settings_lock);

  /* Apply the most recent settings values to the Filter Manager. */
  assert(fm);
  set_pvt_engine_enable_glonass(fm, settings.is_glonass_enabled);
  set_pvt_engine_obs_downweight_factor(
      fm, settings.glonass_downweight_factor, CODE_GLO_L1OF);
  set_pvt_engine_obs_downweight_factor(
      fm, settings.glonass_downweight_factor, CODE_GLO_L2OF);
  set_pvt_engine_elevation_mask(fm, settings.elevation_mask);
}

static void post_observations(u8 n,
                              const navigation_measurement_t m[],
                              const gps_time_t *t,
                              const pvt_engine_result_t *soln) {
  /* TODO: use a buffer from the pool from the start instead of
   * allocating nav_meas_tdcp as well. Downside, if we don't end up
   * pushing the message into the mailbox then we just wasted an
   * observation from the mailbox for no good reason. */

  obss_t *obs = platform_time_matched_obs_alloc();
  msg_t ret;
  if (obs == NULL) {
    /* Pool is empty, grab a buffer from the mailbox instead, i.e.
     * overwrite the oldest item in the queue. */
    ret =
        platform_time_matched_obs_mailbox_fetch((msg_t *)&obs, TIME_IMMEDIATE);
    if (ret != MSG_OK) {
      log_error("Pool full and mailbox empty!");
    }
  }
  if (NULL != obs) {
    obs->tor = *t;
    obs->n = n;
    for (u8 i = 0, cnt = 0; i < n; ++i) {
      obs->nm[cnt++] = m[i];
    }
    if (soln->valid) {
      obs->pos_ecef[0] = soln->baseline[0];
      obs->pos_ecef[1] = soln->baseline[1];
      obs->pos_ecef[2] = soln->baseline[2];
      obs->has_pos = true;
    } else {
      obs->has_pos = false;
    }

    if (soln) {
      obs->soln = *soln;
    } else {
      obs->soln.valid = 0;
      obs->soln.velocity_valid = 0;
    }

    ret = platform_time_matched_obs_mailbox_post((msg_t)obs, TIME_IMMEDIATE);
    if (ret != MSG_OK) {
      /* We could grab another item from the mailbox, discard it and then
       * post our obs again but if the size of the mailbox and the pool
       * are equal then we should have already handled the case where the
       * mailbox is full when we handled the case that the pool was full.
       * */
      log_error("Mailbox should have space!");
      platform_time_matched_obs_free(obs);
    } else {
      last_time_matched_rover_obs_post = *t;
    }
  }
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

void reset_rtk_filter(void) {
  platform_mutex_lock(&time_matched_filter_manager_lock);
  if (time_matched_filter_manager) {
    filter_manager_init(time_matched_filter_manager);
  }
  platform_mutex_unlock(&time_matched_filter_manager_lock);
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
    update_filter_manager_settings(filter_manager);

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

/**
 * Processed a time-matched pair of rover and reference
 * observations. Return code indicates the status of
 * the computation. If successful, the calculated
 * DOPS and result are returned via the output parameters.
 */
static PVT_ENGINE_INTERFACE_RC process_matched_obs(
    const obss_t *rover_channel_meass,
    const obss_t *reference_obss,
    dops_t *rtk_dops,
    pvt_engine_result_t *rtk_result) {
  PVT_ENGINE_INTERFACE_RC update_rov_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_ref_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_filter_ret = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;

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

  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();

  platform_mutex_lock(&time_matched_filter_manager_lock);

  if (!filter_manager_is_initialized(time_matched_filter_manager)) {
    filter_manager_init(time_matched_filter_manager);
  }

  if (filter_manager_is_initialized(time_matched_filter_manager)) {
    filter_manager_overwrite_ephemerides(time_matched_filter_manager,
                                         stored_ephs);

    /* Grab the most recent klobuchar enable setting. */
    platform_mutex_lock(&global_settings_lock);
    bool disable_klobuchar = !global_settings.is_time_matched_klobuchar_enabled;
    platform_mutex_unlock(&global_settings_lock);

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

    if (dgnss_soln_mode == STARLING_SOLN_MODE_LOW_LATENCY &&
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
      platform_mutex_unlock(&base_glonass_biases_lock);
      platform_mutex_unlock(&base_pos_lock);
      platform_mutex_unlock(&low_latency_filter_manager_lock);
    }
  }

  /* If we are in time matched mode then calculate and output the baseline
   * for this observation. */
  if (dgnss_soln_mode == STARLING_SOLN_MODE_TIME_MATCHED &&
      !platform_simulation_enabled() &&
      update_filter_ret == PVT_ENGINE_SUCCESS) {
    /* Note: in time match mode we send the physically incorrect time of the
     * observation message (which can be receiver clock time, or rounded GPS
     * time) instead of the true GPS time of the solution. */
    rtk_result->time = rover_channel_meass->tor;
    rtk_result->propagation_time = 0;
    get_baseline_ret =
        get_baseline(time_matched_filter_manager, true, rtk_dops, rtk_result);
  }

  platform_mutex_unlock(&time_matched_filter_manager_lock);

  return get_baseline_ret;
}

bool update_time_matched(gps_time_t *last_update_time,
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

static THD_WORKING_AREA(wa_time_matched_obs_thread,
                        TIME_MATCHED_OBS_THREAD_STACK);
static void time_matched_obs_thread(void *arg) {
  (void)arg;
  platform_thread_set_name("time matched obs");

  obss_t *base_obs;
  static obss_t base_obss_copy;

  while (1) {
    base_obs = NULL;
    const msg_t fetch_ret =
        platform_base_obs_mailbox_fetch((msg_t *)&base_obs, DGNSS_TIMEOUT_MS);

    if (fetch_ret != MSG_OK) {
      if (NULL != base_obs) {
        log_error("Base obs mailbox fetch failed with %" PRIi32, fetch_ret);
        platform_base_obs_free(base_obs);
      }
      continue;
    }

    if (gps_time_valid(&last_time_matched_rover_obs_post) &&
        gpsdifftime(&last_time_matched_rover_obs_post, &base_obs->tor) >
            BASE_LATENCY_TIMEOUT) {
      log_info("Communication Latency exceeds 15 seconds");
    }

    base_obss_copy = *base_obs;
    platform_base_obs_free(base_obs);

    /* Check if the el mask has changed and update */
    platform_mutex_lock(&time_matched_filter_manager_lock);
    /* Grab the latest settings. */
    update_filter_manager_settings(time_matched_filter_manager);

    set_pvt_engine_update_frequency(time_matched_filter_manager,
                                    starling_frequency);
    platform_mutex_unlock(&time_matched_filter_manager_lock);

    dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (platform_time_matched_obs_mailbox_fetch((msg_t *)&obss,
                                                   TIME_IMMEDIATE) == MSG_OK) {
      if (dgnss_soln_mode == STARLING_SOLN_MODE_NO_DGNSS) {
        /* Not doing any DGNSS.  Toss the obs away. */
        platform_time_matched_obs_free(obss);
        continue;
      }

      double dt = gpsdifftime(&obss->tor, &base_obss_copy.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD && base_obss_copy.has_pos == 1) {
        /* Local variables to capture the filter result. */
        PVT_ENGINE_INTERFACE_RC time_matched_rc = PVT_ENGINE_FAILURE;
        StarlingFilterSolution solution = {0};

        /* Perform the time-matched filter update. */
        static gps_time_t last_update_time = {.wn = 0, .tow = 0.0};
        if (update_time_matched(&last_update_time, &obss->tor, obss->n) ||
            dgnss_soln_mode == STARLING_SOLN_MODE_TIME_MATCHED) {
          time_matched_rc = process_matched_obs(
              obss, &base_obss_copy, &solution.dops, &solution.result);
          last_update_time = obss->tor;
        }

        /* Pass on NULL for the solution if it wasn't successful. */
        StarlingFilterSolution *p_solution = NULL;
        if (PVT_ENGINE_SUCCESS == time_matched_rc) {
          p_solution = &solution;
        }
        send_solution_time_matched(p_solution, &base_obss_copy, obss);

        platform_time_matched_obs_free(obss);
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
          const msg_t post_ret = platform_time_matched_obs_mailbox_post_ahead(
              (msg_t)obss, TIME_IMMEDIATE);
          if (post_ret != MSG_OK) {
            /* Something went wrong with returning it to the buffer, better just
             * free it and carry on. */
            log_warn("Obs Matching: mailbox full, discarding observation!");
            platform_time_matched_obs_free(obss);
          }
          break;
        } else {
          /* Time of base obs later than time of local obs,
           * keep moving through the mailbox. */
          platform_time_matched_obs_free(obss);
        }
      }
    }
  }
}

static void init_filters_and_settings(void) {
  last_time_matched_rover_obs_post = GPS_TIME_UNKNOWN;

  platform_time_matched_obs_mailbox_init();

  platform_mutex_lock(&time_matched_filter_manager_lock);
  time_matched_filter_manager = create_filter_manager_rtk();
  assert(time_matched_filter_manager);
  platform_mutex_unlock(&time_matched_filter_manager_lock);

  platform_mutex_lock(&low_latency_filter_manager_lock);
  low_latency_filter_manager = create_filter_manager_rtk();
  assert(low_latency_filter_manager);
  platform_mutex_unlock(&low_latency_filter_manager_lock);
}

/**
 * Perform the appropriate processing and FilterManager
 * updates for a single SBAS message.
 */
static void process_sbas_message(const msg_sbas_raw_t *sbas_msg) {
  const gps_time_t current_time = get_current_time();
  if (!gps_time_valid(&current_time)) {
    return;
  }
  sbas_raw_data_t sbas_data;
  unpack_sbas_raw_data(sbas_msg, &sbas_data);

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

/**
 * Try and fetch available SBAS messages from the SBAS mailbox.
 *
 * NOTE: This function should not block, so we use TIME_IMMEDIATE for
 * the fetch operation. If a message is there, we take it, otherwise
 * nevermind.
 */
static void process_any_sbas_messages(void) {
  msg_t ret = MSG_OK;
  while (MSG_OK == ret) {
    msg_sbas_raw_t *sbas_msg = NULL;
    ret = platform_sbas_msg_mailbox_fetch((msg_t *)&sbas_msg, TIME_IMMEDIATE);
    if (MSG_OK == ret) {
      /* We have successfully received an SBAS message, forward on to the
       * filter managers. */
      process_sbas_message(sbas_msg);
    } else if (NULL != sbas_msg) {
      /* If the fetch operation failed after assigning to the message pointer,
       * something has gone unexpectedly wrong. */
      log_error("STARLING: sbas mailbox fetch failed with %" PRIi32, ret);
    }
    /* Under any circumstances, if the message pointer was assigned to, it
     * must be released back to the pool. */
    if (NULL != sbas_msg) {
      platform_sbas_msg_free(sbas_msg);
    }
  }
}

static void starling_thread(void) {
  msg_t ret;

  /* Initialize all filters, settings, and SBP callbacks. */
  init_filters_and_settings();

  /* Spawn the time_matched thread. */
  platform_thread_create_static(wa_time_matched_obs_thread,
                                sizeof(wa_time_matched_obs_thread),
                                TIME_MATCHED_OBS_THREAD_PRIORITY,
                                time_matched_obs_thread,
                                NULL);

  sbp_messages_t sbp_messages;

  static navigation_measurement_t nav_meas[MAX_CHANNELS];
  static ephemeris_t e_meas[MAX_CHANNELS];
  static gps_time_t obs_time;

  platform_mutex_lock(&spp_filter_manager_lock);
  spp_filter_manager = create_filter_manager_spp();
  assert(spp_filter_manager);
  filter_manager_init(spp_filter_manager);
  platform_mutex_unlock(&spp_filter_manager_lock);

  while (TRUE) {
    platform_watchdog_notify_starling_main_thread();

    process_any_sbas_messages();

    me_msg_obs_t *me_msg = NULL;
    ret = platform_me_obs_msg_mailbox_fetch((msg_t *)&me_msg, DGNSS_TIMEOUT_MS);
    if (ret != MSG_OK) {
      if (NULL != me_msg) {
        log_error("STARLING: mailbox fetch failed with %" PRIi32, ret);
        platform_me_obs_msg_free(me_msg);
      }
      continue;
    }

    /* Init the messages we want to send */

    gps_time_t epoch_time = me_msg->obs_time;
    if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= get_time_quality()) {
      /* observations do not have valid time, but we have a reasonable estimate
       * of current GPS time, so round that to nearest epoch and use it
       */
      epoch_time = get_current_time();
      epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
    }

    starling_integration_sbp_messages_init(&sbp_messages, &epoch_time);

    /* Here we do all the nice simulation-related stuff. */
    if (platform_simulation_enabled()) {
      starling_integration_solution_simulation(&sbp_messages);
      const u8 fake_base_sender_id = 1;
      starling_integration_solution_send_low_latency_output(
          fake_base_sender_id, &sbp_messages, me_msg->size, me_msg->obs);
      platform_me_obs_msg_free(me_msg);
      continue;
    }

    if (me_msg->size == 0 || !gps_time_valid(&me_msg->obs_time)) {
      platform_me_obs_msg_free(me_msg);
      starling_integration_solution_send_low_latency_output(
          0, &sbp_messages, 0, nav_meas);
      continue;
    }

    if (gpsdifftime(&me_msg->obs_time, &obs_time) <= 0.0) {
      /* When we change the solution rate down, we sometimes can round the
       * time to an epoch earlier than the previous one processed, in that
       * case we want to ignore any epochs with an earlier timestamp */
      platform_me_obs_msg_free(me_msg);
      continue;
    }

    starling_frequency = soln_freq_setting;

    u8 n_ready = me_msg->size;
    memset(nav_meas, 0, sizeof(nav_meas));

    if (n_ready) {
      MEMCPY_S(nav_meas,
               sizeof(nav_meas),
               me_msg->obs,
               n_ready * sizeof(navigation_measurement_t));
    }

    memset(e_meas, 0, sizeof(e_meas));

    if (n_ready) {
      MEMCPY_S(
          e_meas, sizeof(e_meas), me_msg->ephem, n_ready * sizeof(ephemeris_t));
    }

    obs_time = me_msg->obs_time;

    platform_me_obs_msg_free(me_msg);

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

    /* This will duplicate pointers to satellites with mutliple frequencies,
     * but this scenario is expected and handled */
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

    /* Here we do the following:
     * 1. Call SPP filter.
     * 2. With SPP success call RTK filter.
     *
     * The outputs should be as follows.
     * 1. SPP success & RTK success & LOW_LATENCY enabled = SPP overwritten with
     * RTK output.
     * 2. SPP success & RTK failure                       = SPP output.
     * 3. SPP failure & not TIME_MATCHED enabled          = default output.
     */
    PVT_ENGINE_INTERFACE_RC spp_rc = PVT_ENGINE_FAILURE;
    PVT_ENGINE_INTERFACE_RC rtk_rc = PVT_ENGINE_FAILURE;
    StarlingFilterSolution spp_solution = {0};
    StarlingFilterSolution rtk_solution = {0};
    spp_solution.result.valid = false;
    rtk_solution.result.valid = false;
    dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();

    /* Always try and run the SPP filter. */
    platform_mutex_lock(&spp_filter_manager_lock);
    spp_rc = call_pvt_engine_filter(spp_filter_manager,
                                    &obs_time,
                                    n_ready,
                                    nav_meas,
                                    stored_ephs,
                                    starling_frequency,
                                    &spp_solution.result,
                                    &spp_solution.dops);
    platform_mutex_unlock(&spp_filter_manager_lock);

    /* We only post to time-matched thread on SPP success. */
    if (PVT_ENGINE_SUCCESS == spp_rc) {
      post_observations(n_ready, nav_meas, &obs_time, &spp_solution.result);
    }

    /* Figure out if we want to run the RTK filter. */
    const bool should_do_low_latency_rtk =
        (PVT_ENGINE_SUCCESS == spp_rc) &&
        (STARLING_SOLN_MODE_LOW_LATENCY == dgnss_soln_mode);

    /* Run the RTK filter when desired. */
    if (should_do_low_latency_rtk) {
      platform_mutex_lock(&low_latency_filter_manager_lock);
      rtk_rc = call_pvt_engine_filter(low_latency_filter_manager,
                                      &obs_time,
                                      n_ready,
                                      nav_meas,
                                      stored_ephs,
                                      starling_frequency,
                                      &rtk_solution.result,
                                      &rtk_solution.dops);
      platform_mutex_unlock(&low_latency_filter_manager_lock);
    }

    /* Generate the output based on which filters ran successfully. */
    StarlingFilterSolution *p_spp_solution = NULL;
    if (PVT_ENGINE_SUCCESS == spp_rc) {
      p_spp_solution = &spp_solution;
    }
    StarlingFilterSolution *p_rtk_solution = NULL;
    if (PVT_ENGINE_SUCCESS == rtk_rc) {
      p_rtk_solution = &rtk_solution;
    }

    /* Forward solutions to outside world. */
    send_solution_low_latency(
        p_spp_solution, p_rtk_solution, &epoch_time, nav_meas, n_ready);
  }
}

/* Run the starling engine on the current thread. Blocks indefinitely. */
void starling_run(void) { starling_thread(); }

/*******************************************************************************
 * Settings Update Functions
 * -------------------------
 * Take care to never hold a FilterManager lock inside the scope of a
 * global_settings_lock. Doing so may result in deadlock.
 ******************************************************************************/

/* Enable glonass constellation in the Starling engine. */
void starling_set_is_glonass_enabled(bool is_glonass_enabled) {
  platform_mutex_lock(&global_settings_lock);
  global_settings.is_glonass_enabled = is_glonass_enabled;
  platform_mutex_unlock(&global_settings_lock);
}

/* Enable klobuchar corrections in the time-matched filter. */
void starling_set_is_time_matched_klobuchar_enabled(bool is_klobuchar_enabled) {
  platform_mutex_lock(&global_settings_lock);
  global_settings.is_time_matched_klobuchar_enabled = is_klobuchar_enabled;
  platform_mutex_unlock(&global_settings_lock);
}

/* Modify the relative weighting of glonass observations. */
void starling_set_glonass_downweight_factor(float factor) {
  platform_mutex_lock(&global_settings_lock);
  global_settings.glonass_downweight_factor = factor;
  platform_mutex_unlock(&global_settings_lock);
}

/* Set the elevation mask used to filter satellites from the solution. */
void starling_set_elevation_mask(float elevation_mask) {
  platform_mutex_lock(&global_settings_lock);
  global_settings.elevation_mask = elevation_mask;
  platform_mutex_unlock(&global_settings_lock);
}

/* Enable fixed RTK mode in the Starling engine. */
void starling_set_is_fix_enabled(bool is_fix_enabled) {
  platform_mutex_lock(&low_latency_filter_manager_lock);
  if (low_latency_filter_manager) {
    set_pvt_engine_enable_fix_mode(low_latency_filter_manager, is_fix_enabled);
  }
  platform_mutex_unlock(&low_latency_filter_manager_lock);

  platform_mutex_lock(&time_matched_filter_manager_lock);
  if (time_matched_filter_manager) {
    set_pvt_engine_enable_fix_mode(time_matched_filter_manager, is_fix_enabled);
  }
  platform_mutex_unlock(&time_matched_filter_manager_lock);
}

/* Indicate for how long corrections should persist. */
void starling_set_max_correction_age(int max_age) {
  platform_mutex_lock(&low_latency_filter_manager_lock);
  if (low_latency_filter_manager) {
    set_max_correction_age(low_latency_filter_manager, max_age);
  }
  platform_mutex_unlock(&low_latency_filter_manager_lock);

  platform_mutex_lock(&time_matched_filter_manager_lock);
  if (time_matched_filter_manager) {
    set_max_correction_age(time_matched_filter_manager, max_age);
  }
  platform_mutex_unlock(&time_matched_filter_manager_lock);
}

/* Set the desired solution mode for the Starling engine. */
void starling_set_solution_mode(dgnss_solution_mode_t mode) {
  platform_mutex_lock(&global_settings_lock);
  global_settings.solution_output_mode = mode;
  platform_mutex_unlock(&global_settings_lock);
}

/* Get the current solution mode for the Starling engine. */
dgnss_solution_mode_t starling_get_solution_mode(void) {
  platform_mutex_lock(&global_settings_lock);
  dgnss_solution_mode_t mode = global_settings.solution_output_mode;
  platform_mutex_unlock(&global_settings_lock);
  return mode;
}
