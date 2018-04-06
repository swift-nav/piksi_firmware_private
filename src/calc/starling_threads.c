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
#include "ndb/ndb.h"
#include "settings/settings.h"
#include "starling_platform_shim.h"
#include "starling_threads.h"

/* Maximum CPU time the solution thread is allowed to use. */
#define SOLN_THD_CPU_MAX (0.60f)

#define STARLING_THREAD_PRIORITY (HIGHPRIO - 4)
#define STARLING_THREAD_STACK (6 * 1024 * 1024)

#define TIME_MATCHED_OBS_THREAD_PRIORITY (NORMALPRIO - 3)
#define TIME_MATCHED_OBS_THREAD_STACK (6 * 1024 * 1024)

/* Initial settings values. */
#define INIT_ENABLE_FIX_MODE FILTER_FIXED
#define INIT_MAX_AGE_DIFFERENTIAL 30

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;

static FilterManager *time_matched_filter_manager = NULL;
static FilterManager *low_latency_filter_manager = NULL;
static FilterManager *spp_filter_manager = NULL;

MUTEX_DECL(time_matched_filter_manager_lock);
MUTEX_DECL(low_latency_filter_manager_lock);
MUTEX_DECL(spp_filter_manager_lock);

MUTEX_DECL(time_matched_iono_params_lock);
bool has_time_matched_iono_params = false;
static ionosphere_t time_matched_iono_params;

MUTEX_DECL(last_sbp_lock);
static gps_time_t last_dgnss;
static gps_time_t last_spp;
static gps_time_t last_time_matched_rover_obs_post;

static double starling_frequency;

bool send_heading = false;

double heading_offset = 0.0;

static bool disable_klobuchar = false;

bool enable_glonass = true;

static float glonass_downweight_factor = 4;

static u8 current_base_sender_id;

static soln_pvt_stats_t last_pvt_stats = {.systime = PIKSI_SYSTIME_INIT,
                                          .signals_used = 0};
static soln_dgnss_stats_t last_dgnss_stats = {.systime = PIKSI_SYSTIME_INIT,
                                              .mode = 0};
static sbas_system_t current_sbas_system = SBAS_UNKNOWN;

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
  filter_manager_init(time_matched_filter_manager);
  platform_mutex_unlock(&time_matched_filter_manager_lock);
}

/** Determine if we have had a DGNSS timeout.
 *
 * \param _last_dgnss. Last time of DGNSS solution
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
bool dgnss_timeout(piksi_systime_t *_last_dgnss,
                   dgnss_solution_mode_t _dgnss_soln_mode) {
  /* No timeout needed in low latency mode */
  if (SOLN_MODE_LOW_LATENCY == _dgnss_soln_mode) {
    return false;
  }

  /* Need to compare timeout threshold in MS to system time elapsed (in system
   * ticks) */
  return (piksi_systime_elapsed_since_ms(_last_dgnss) > DGNSS_TIMEOUT_MS);
}

/** Determine if we have had a SPP timeout.
 *
 * \param _last_spp. Last time of SPP solution
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
bool spp_timeout(const gps_time_t *_last_spp,
                 const gps_time_t *_last_dgnss,
                 dgnss_solution_mode_t _dgnss_soln_mode) {
  /* No timeout needed in low latency mode; */
  if (_dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
    return false;
  }
  platform_mutex_lock(&last_sbp_lock);
  double time_diff = gpsdifftime(_last_dgnss, _last_spp);
  platform_mutex_unlock(&last_sbp_lock);

  /* Need to compare timeout threshold in MS to system time elapsed (in system
   * ticks) */
  return (time_diff > 0.0);
}

void solution_make_sbp(const pvt_engine_result_t *soln,
                       dops_t *dops,
                       sbp_messages_t *sbp_messages) {
  if (soln && soln->valid) {
    /* Send GPS_TIME message first. */
    sbp_make_gps_time(&sbp_messages->gps_time, &soln->time, SPP_POSITION);
    sbp_make_utc_time(&sbp_messages->utc_time, &soln->time, SPP_POSITION);

    /* In SPP, `baseline` is actually absolute position in ECEF. */
    double pos_ecef[3], pos_llh[3];
    memcpy(pos_ecef, soln->baseline, 3 * sizeof(double));
    wgsecef2llh(pos_ecef, pos_llh);

    double accuracy, h_accuracy, v_accuracy;
    double pos_ecef_cov[6], pos_ned_cov[6];
    pvt_engine_covariance_to_accuracy(soln->baseline_covariance,
                                      pos_ecef,
                                      &accuracy,
                                      &h_accuracy,
                                      &v_accuracy,
                                      pos_ecef_cov,
                                      pos_ned_cov);

    double vel_accuracy, vel_h_accuracy, vel_v_accuracy;
    double vel_ecef_cov[6], vel_ned_cov[6];
    pvt_engine_covariance_to_accuracy(soln->velocity_covariance,
                                      pos_ecef,
                                      &vel_accuracy,
                                      &vel_h_accuracy,
                                      &vel_v_accuracy,
                                      vel_ecef_cov,
                                      vel_ned_cov);

    sbp_make_pos_llh_vect(&sbp_messages->pos_llh,
                          pos_llh,
                          h_accuracy,
                          v_accuracy,
                          &soln->time,
                          soln->num_sats_used,
                          soln->flags);

    sbp_make_pos_llh_cov(&sbp_messages->pos_llh_cov,
                         pos_llh,
                         pos_ned_cov,
                         &soln->time,
                         soln->num_sats_used,
                         soln->flags);

    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef,
                           pos_ecef,
                           accuracy,
                           &soln->time,
                           soln->num_sats_used,
                           soln->flags);

    sbp_make_pos_ecef_cov(&sbp_messages->pos_ecef_cov,
                          pos_ecef,
                          pos_ecef_cov,
                          &soln->time,
                          soln->num_sats_used,
                          soln->flags);

    if (soln->velocity_valid) {
      double vel_ned[3];
      wgsecef2ned(soln->velocity, pos_ecef, vel_ned);
      sbp_make_vel_ned(&sbp_messages->vel_ned,
                       vel_ned,
                       vel_h_accuracy,
                       vel_v_accuracy,
                       &soln->time,
                       soln->num_sats_used,
                       soln->flags);

      sbp_make_vel_ned_cov(&sbp_messages->vel_ned_cov,
                           vel_ned,
                           vel_ned_cov,
                           &soln->time,
                           soln->num_sats_used,
                           soln->flags);

      sbp_make_vel_ecef(&sbp_messages->vel_ecef,
                        soln->velocity,
                        vel_accuracy,
                        &soln->time,
                        soln->num_sats_used,
                        soln->flags);

      sbp_make_vel_ecef_cov(&sbp_messages->vel_ecef_cov,
                            soln->velocity,
                            vel_ecef_cov,
                            &soln->time,
                            soln->num_sats_used,
                            soln->flags);
    }

    /* DOP message can be sent even if solution fails to compute */
    if (dops) {
      sbp_make_dops(&sbp_messages->sbp_dops,
                    dops,
                    sbp_messages->pos_llh.tow,
                    soln->flags);
    }

    /* Update stats */
    piksi_systime_get(&last_pvt_stats.systime);
    last_pvt_stats.signals_used = soln->num_sigs_used;
  }
}

/**
 *
 * @param base_sender_id sender id of base obs
 * @param sbp_messages struct of sbp messages
 * @param n_meas nav_meas len
 * @param nav_meas Valid navigation measurements
 */
static void solution_send_pos_messages(
    u8 base_sender_id,
    const sbp_messages_t *sbp_messages,
    u8 n_meas,
    const navigation_measurement_t nav_meas[]) {
  if (sbp_messages) {
    sbp_send_msg(SBP_MSG_GPS_TIME,
                 sizeof(sbp_messages->gps_time),
                 (u8 *)&sbp_messages->gps_time);
    sbp_send_msg(SBP_MSG_UTC_TIME,
                 sizeof(sbp_messages->utc_time),
                 (u8 *)&sbp_messages->utc_time);
    sbp_send_msg(SBP_MSG_POS_LLH,
                 sizeof(sbp_messages->pos_llh),
                 (u8 *)&sbp_messages->pos_llh);
    sbp_send_msg(SBP_MSG_POS_ECEF,
                 sizeof(sbp_messages->pos_ecef),
                 (u8 *)&sbp_messages->pos_ecef);
    sbp_send_msg(SBP_MSG_VEL_NED,
                 sizeof(sbp_messages->vel_ned),
                 (u8 *)&sbp_messages->vel_ned);
    sbp_send_msg(SBP_MSG_VEL_ECEF,
                 sizeof(sbp_messages->vel_ecef),
                 (u8 *)&sbp_messages->vel_ecef);
    sbp_send_msg(SBP_MSG_DOPS,
                 sizeof(sbp_messages->sbp_dops),
                 (u8 *)&sbp_messages->sbp_dops);
    sbp_send_msg(SBP_MSG_POS_ECEF_COV,
                 sizeof(sbp_messages->pos_ecef_cov),
                 (u8 *)&sbp_messages->pos_ecef_cov);
    sbp_send_msg(SBP_MSG_VEL_ECEF_COV,
                 sizeof(sbp_messages->vel_ecef_cov),
                 (u8 *)&sbp_messages->vel_ecef_cov);
    sbp_send_msg(SBP_MSG_POS_LLH_COV,
                 sizeof(sbp_messages->pos_llh_cov),
                 (u8 *)&sbp_messages->pos_llh_cov);
    sbp_send_msg(SBP_MSG_VEL_NED_COV,
                 sizeof(sbp_messages->vel_ned_cov),
                 (u8 *)&sbp_messages->vel_ned_cov);

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_ECEF,
                   sizeof(sbp_messages->baseline_ecef),
                   (u8 *)&sbp_messages->baseline_ecef);
    }

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_NED,
                   sizeof(sbp_messages->baseline_ned),
                   (u8 *)&sbp_messages->baseline_ned);
    }

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_AGE_CORRECTIONS,
                   sizeof(sbp_messages->age_corrections),
                   (u8 *)&sbp_messages->age_corrections);
    }

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_DGNSS_STATUS,
                   sizeof(sbp_messages->dgnss_status),
                   (u8 *)&sbp_messages->dgnss_status);
    }

    if (send_heading && dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_HEADING,
                   sizeof(sbp_messages->baseline_heading),
                   (u8 *)&sbp_messages->baseline_heading);
    }
  }

  utc_params_t utc_params;
  utc_params_t *p_utc_params = &utc_params;
  /* read UTC parameters from NDB if they exist*/
  if (NDB_ERR_NONE != ndb_utc_params_read(&utc_params, NULL)) {
    p_utc_params = NULL;
  }

  /* Send NMEA alongside the sbp */
  double propagation_time = sbp_messages->age_corrections.age * 0.1;
  nmea_send_msgs(&sbp_messages->pos_llh,
                 &sbp_messages->vel_ned,
                 &sbp_messages->sbp_dops,
                 &sbp_messages->gps_time,
                 propagation_time,
                 base_sender_id,
                 p_utc_params,
                 &sbp_messages->baseline_heading,
                 n_meas,
                 nav_meas);
}

static void solution_send_low_latency_output(
    u8 base_sender_id,
    const sbp_messages_t *sbp_messages,
    u8 n_meas,
    const navigation_measurement_t nav_meas[]) {
  /* Work out if we need to wait for a certain period of no time matched
   * positions before we output a SBP position */
  bool wait_for_timeout = false;
  if (!(dgnss_timeout(&last_dgnss_stats.systime, dgnss_soln_mode)) &&
      SOLN_MODE_TIME_MATCHED == dgnss_soln_mode) {
    wait_for_timeout = true;
  }

  if (!wait_for_timeout) {
    solution_send_pos_messages(base_sender_id, sbp_messages, n_meas, nav_meas);
    platform_mutex_lock(&last_sbp_lock);
    last_spp.wn = sbp_messages->gps_time.wn;
    last_spp.tow = sbp_messages->gps_time.tow * 0.001;
    platform_mutex_unlock(&last_sbp_lock);
  }
}

double calc_heading(const double b_ned[3]) {
  double heading = atan2(b_ned[1], b_ned[0]);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading * R2D;
}

#define SPP_ECEF_SIZE 3

void solution_make_baseline_sbp(const pvt_engine_result_t *result,
                                const double spp_ecef[SPP_ECEF_SIZE],
                                const dops_t *dops,
                                sbp_messages_t *sbp_messages) {
  double ecef_pos[3];
  if (result->has_known_reference_pos) {
    vector_add(3, result->known_reference_pos, result->baseline, ecef_pos);
  } else {
    MEMCPY_S(
        ecef_pos, sizeof(ecef_pos), spp_ecef, SPP_ECEF_SIZE * sizeof(double));
  }

  double b_ned[3];
  wgsecef2ned(result->baseline, ecef_pos, b_ned);

  double accuracy, h_accuracy, v_accuracy, pos_ecef_cov[6], pos_ned_cov[6];
  pvt_engine_covariance_to_accuracy(result->baseline_covariance,
                                    ecef_pos,
                                    &accuracy,
                                    &h_accuracy,
                                    &v_accuracy,
                                    pos_ecef_cov,
                                    pos_ned_cov);

  sbp_make_baseline_ecef(&sbp_messages->baseline_ecef,
                         &result->time,
                         result->num_sats_used,
                         result->baseline,
                         accuracy,
                         result->flags);

  sbp_make_baseline_ned(&sbp_messages->baseline_ned,
                        &result->time,
                        result->num_sats_used,
                        b_ned,
                        h_accuracy,
                        v_accuracy,
                        result->flags);

  sbp_make_age_corrections(
      &sbp_messages->age_corrections, &result->time, result->propagation_time);

  sbp_make_dgnss_status(&sbp_messages->dgnss_status,
                        result->num_sats_used,
                        result->propagation_time,
                        result->flags);

  if (result->flags == FIXED_POSITION &&
      dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    double heading = calc_heading(b_ned);
    sbp_make_heading(&sbp_messages->baseline_heading,
                     &result->time,
                     heading + heading_offset,
                     result->num_sats_used,
                     result->flags);
  }

  if (result->has_known_reference_pos ||
      (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
       simulation_enabled_for(SIMULATION_MODE_RTK))) {
    double pseudo_absolute_ecef[3];
    double pseudo_absolute_llh[3];

    vector_add(
        3, result->known_reference_pos, result->baseline, pseudo_absolute_ecef);
    wgsecef2llh(pseudo_absolute_ecef, pseudo_absolute_llh);

    /* now send pseudo absolute sbp message */
    sbp_make_pos_llh_vect(&sbp_messages->pos_llh,
                          pseudo_absolute_llh,
                          h_accuracy,
                          v_accuracy,
                          &result->time,
                          result->num_sats_used,
                          result->flags);
    sbp_make_pos_llh_cov(&sbp_messages->pos_llh_cov,
                         pseudo_absolute_llh,
                         pos_ned_cov,
                         &result->time,
                         result->num_sats_used,
                         result->flags);
    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef,
                           pseudo_absolute_ecef,
                           accuracy,
                           &result->time,
                           result->num_sats_used,
                           result->flags);
    sbp_make_pos_ecef_cov(&sbp_messages->pos_ecef_cov,
                          pseudo_absolute_ecef,
                          pos_ecef_cov,
                          &result->time,
                          result->num_sats_used,
                          result->flags);
  }
  sbp_make_dops(
      &sbp_messages->sbp_dops, dops, sbp_messages->pos_llh.tow, result->flags);

  platform_mutex_lock(&last_sbp_lock);
  last_dgnss.wn = result->time.wn;
  last_dgnss.tow = result->time.tow;
  platform_mutex_unlock(&last_sbp_lock);

  /* Update stats */
  piksi_systime_get(&last_dgnss_stats.systime);
  last_dgnss_stats.mode =
      (result->flags == FIXED_POSITION) ? FILTER_FIXED : FILTER_FLOAT;
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

static void solution_simulation(sbp_messages_t *sbp_messages) {
  simulation_step();

  /* TODO: The simulator's handling of time is a bit crazy. This is a hack
   * for now but the simulator should be refactored so that it can give the
   * exact correct solution time output without this nonsense. */
  pvt_engine_result_t *soln = simulation_current_pvt_engine_result_t();

  if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
    solution_make_sbp(soln, simulation_current_dops_solution(), sbp_messages);
  }

  if (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK)) {
    u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK) ? FIXED_POSITION
                                                           : FLOAT_POSITION;

    pvt_engine_result_t result = {
        .time = soln->time,
        .num_sats_used = simulation_current_num_sats(),
        .num_sigs_used = 0,
        .flags = flags,
        .has_known_reference_pos = true,
        .propagation_time = 0.0,
    };
    MEMCPY_S(result.baseline,
             sizeof(result.baseline),
             simulation_current_baseline_ecef(),
             sizeof(result.baseline));
    MEMCPY_S(result.baseline_covariance,
             sizeof(result.baseline_covariance),
             simulation_current_covariance_ecef(),
             sizeof(result.baseline_covariance));
    MEMCPY_S(result.known_reference_pos,
             sizeof(result.known_reference_pos),
             simulation_ref_ecef(),
             sizeof(result.known_reference_pos));

    solution_make_baseline_sbp(&result,
                               simulation_ref_ecef(),
                               simulation_current_dops_solution(),
                               sbp_messages);

    double t_check = soln->time.tow * (starling_frequency / obs_output_divisor);
    if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
      /* RFT_TODO *
       * SBP_FRAMING_MAX_PAYLOAD_SIZE replaces the setting for now, but
       * this function will completely go away */
      send_observations(simulation_current_num_sats(),
                        SBP_FRAMING_MAX_PAYLOAD_SIZE,
                        simulation_current_navigation_measurements(),
                        &(soln->time));
    }
  }
}

void sbp_messages_init(sbp_messages_t *sbp_messages, gps_time_t *t) {
  sbp_init_gps_time(&sbp_messages->gps_time, t);
  sbp_init_utc_time(&sbp_messages->utc_time, t);
  sbp_init_pos_llh(&sbp_messages->pos_llh, t);
  sbp_init_pos_ecef(&sbp_messages->pos_ecef, t);
  sbp_init_vel_ned(&sbp_messages->vel_ned, t);
  sbp_init_vel_ecef(&sbp_messages->vel_ecef, t);
  sbp_init_sbp_dops(&sbp_messages->sbp_dops, t);
  sbp_init_age_corrections(&sbp_messages->age_corrections, t);
  sbp_init_dgnss_status(&sbp_messages->dgnss_status);
  sbp_init_baseline_ecef(&sbp_messages->baseline_ecef, t);
  sbp_init_baseline_ned(&sbp_messages->baseline_ned, t);
  sbp_init_baseline_heading(&sbp_messages->baseline_heading, t);
  sbp_init_pos_ecef_cov(&sbp_messages->pos_ecef_cov, t);
  sbp_init_vel_ecef_cov(&sbp_messages->vel_ecef_cov, t);
  sbp_init_pos_llh_cov(&sbp_messages->pos_llh_cov, t);
  sbp_init_vel_ned_cov(&sbp_messages->vel_ned_cov, t);
}

void process_matched_obs(const obss_t *rover_channel_meass,
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

void init_filters(void) {
  platform_mutex_lock(&time_matched_filter_manager_lock);
  time_matched_filter_manager = create_filter_manager_rtk();
  platform_mutex_unlock(&time_matched_filter_manager_lock);

  platform_mutex_lock(&low_latency_filter_manager_lock);
  low_latency_filter_manager = create_filter_manager_rtk();
  platform_mutex_unlock(&low_latency_filter_manager_lock);

  /* We also need to be careful to set any initial values which may
   * later be updated by settings changes. */
  starling_set_enable_fix_mode(INIT_ENABLE_FIX_MODE);
  starling_set_max_correction_age(INIT_MAX_AGE_DIFFERENTIAL);
}

static THD_WORKING_AREA(wa_time_matched_obs_thread,
                        TIME_MATCHED_OBS_THREAD_STACK);
static void time_matched_obs_thread(void *arg) {
  (void)arg;
  platform_thread_set_name("time matched obs");

  obss_t *base_obs;
  static obss_t base_obss_copy;

  /* Declare all SBP messages */
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

    /* Check if the el mask has changed and update */
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
    while (platform_time_matched_obs_mailbox_fetch((msg_t *)&obss,
                                                   TIME_IMMEDIATE) == MSG_OK) {
      if (dgnss_soln_mode == SOLN_MODE_NO_DGNSS) {
        /* Not doing any DGNSS.  Toss the obs away. */
        platform_time_matched_obs_free(obss);
        continue;
      }

      double dt = gpsdifftime(&obss->tor, &base_obss_copy.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD && base_obss_copy.has_pos == 1) {
        /* We need to form the SBP messages derived from the SPP at this
         * solution time before we
         * do the differential solution so that the various messages can be
         * overwritten as appropriate,
         * the exception is the DOP messages, as we don't have the SPP DOP and
         * it will always be overwritten by the differential */
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

void reset_filters_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
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

soln_dgnss_stats_t solution_last_dgnss_stats_get(void) {
  return last_dgnss_stats;
}

soln_pvt_stats_t solution_last_pvt_stats_get(void) { return last_pvt_stats; }

/* Check that -180.0 <= new heading_offset setting value <= 180.0. */
static bool heading_offset_changed(struct setting *s, const char *val) {
  double offset = 0;
  bool ret = s->type->from_string(s->type->priv, &offset, s->len, val);
  if (!ret) {
    return ret;
  }

  if (fabs(offset) > 180.0) {
    log_error(
        "Invalid heading offset setting of %3.1f, max is %3.1f, min is %3.1f, "
        "leaving heading offset at %3.1f",
        offset,
        180.0,
        -180.0,
        heading_offset);
    ret = false;
  }
  *(double *)s->addr = offset;
  return ret;
}

static void init_filters_and_settings(void) {
  /* Set time of last differential solution in the past. */
  last_dgnss = GPS_TIME_UNKNOWN;
  last_spp = GPS_TIME_UNKNOWN;
  last_time_matched_rover_obs_post = GPS_TIME_UNKNOWN;

  static const char *const dgnss_soln_mode_enum[] = {
      "Low Latency", "Time Matched", "No DGNSS", NULL};
  static struct setting_type dgnss_soln_mode_setting;
  int TYPE_GNSS_SOLN_MODE = settings_type_register_enum(
      dgnss_soln_mode_enum, &dgnss_soln_mode_setting);
  SETTING(
      "solution", "dgnss_solution_mode", dgnss_soln_mode, TYPE_GNSS_SOLN_MODE);

  SETTING("solution", "send_heading", send_heading, TYPE_BOOL);
  SETTING_NOTIFY("solution",
                 "heading_offset",
                 heading_offset,
                 TYPE_FLOAT,
                 heading_offset_changed);

  SETTING(
      "solution", "disable_klobuchar_correction", disable_klobuchar, TYPE_BOOL);
  SETTING("solution", "enable_glonass", enable_glonass, TYPE_BOOL);
  SETTING("solution",
          "glonass_measurement_std_downweight_factor",
          glonass_downweight_factor,
          TYPE_FLOAT);

  platform_time_matched_obs_mailbox_init();

  platform_mutex_lock(&time_matched_filter_manager_lock);
  time_matched_filter_manager = create_filter_manager_rtk();
  platform_mutex_unlock(&time_matched_filter_manager_lock);

  platform_mutex_lock(&low_latency_filter_manager_lock);
  low_latency_filter_manager = create_filter_manager_rtk();
  platform_mutex_unlock(&low_latency_filter_manager_lock);

  /* We also need to be careful to set any initial values which may
   * later be updated by settings changes. */
  starling_set_enable_fix_mode(INIT_ENABLE_FIX_MODE);
  starling_set_max_correction_age(INIT_MAX_AGE_DIFFERENTIAL);

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
      SBP_MSG_RESET_FILTERS, &reset_filters_callback, &reset_filters_node);
}

static THD_WORKING_AREA(wa_starling_thread, STARLING_THREAD_STACK);
static void starling_thread(void *arg) {
  (void)arg;
  msg_t ret;

  platform_thread_set_name("starling");

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
    if (platform_simulation_enabled()) {
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

void starling_setup() {
  /* Start solution thread */
  platform_thread_create_static(wa_starling_thread,
                                sizeof(wa_starling_thread),
                                STARLING_THREAD_PRIORITY,
                                starling_thread,
                                NULL);
}

/* Enable fixed RTK mode in the Starling engine. */
void starling_set_enable_fix_mode(bool is_fix_enabled) {
  platform_mutex_lock(&time_matched_filter_manager_lock);
  if (time_matched_filter_manager) {
    set_pvt_engine_enable_fix_mode(time_matched_filter_manager, is_fix_enabled);
  }
  platform_mutex_unlock(&time_matched_filter_manager_lock);

  platform_mutex_lock(&low_latency_filter_manager_lock);
  if (low_latency_filter_manager) {
    set_pvt_engine_enable_fix_mode(low_latency_filter_manager, is_fix_enabled);
  }
  platform_mutex_unlock(&low_latency_filter_manager_lock);
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
