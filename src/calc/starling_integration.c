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

#include <assert.h>
#include <ch.h>
#include <string.h>

#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/memcpy_s.h>

#include "calc/calc_pvt_common.h"
#include "calc/calc_pvt_me.h"
#include "calc/starling_integration.h"
#include "calc/starling_threads.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings.h"
#include "simulator/simulator.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define STARLING_THREAD_PRIORITY (HIGHPRIO - 4)
#define STARLING_THREAD_STACK (6 * 1024 * 1024)

#define STARLING_BASE_SENDER_ID_DEFAULT 0

/*******************************************************************************
 * Globals
 ******************************************************************************/
bool enable_glonass = true;
bool send_heading = false;
double heading_offset = 0.0;

/*******************************************************************************
 * Locals
 ******************************************************************************/

/* Working area for the main starling thread. */
static THD_WORKING_AREA(wa_starling_thread, STARLING_THREAD_STACK);

static MUTEX_DECL(last_sbp_lock);
static gps_time_t last_dgnss;
static gps_time_t last_spp;

static soln_pvt_stats_t last_pvt_stats = {.systime = PIKSI_SYSTIME_INIT,
                                          .signals_used = 0};
static soln_dgnss_stats_t last_dgnss_stats = {.systime = PIKSI_SYSTIME_INIT,
                                              .mode = 0};

/* Keeps track of the which base sent us the most recent base
 * observations so we can appropriately identify who was involved
 * in any RTK solutions. */
static MUTEX_DECL(current_base_sender_id_lock);
static u8 current_base_sender_id = STARLING_BASE_SENDER_ID_DEFAULT;

/*******************************************************************************
 * Output Callback Helpers
 ******************************************************************************/

static double calc_heading(const double b_ned[3]) {
  double heading = atan2(b_ned[1], b_ned[0]);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading * R2D;
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
  /* No timeout needed in low latency mode; */
  if (_dgnss_soln_mode == STARLING_SOLN_MODE_LOW_LATENCY) {
    return false;
  }
  chMtxLock(&last_sbp_lock);
  double time_diff = gpsdifftime(_last_dgnss, _last_spp);
  chMtxUnlock(&last_sbp_lock);

  /* Need to compare timeout threshold in MS to system time elapsed (in system
   * ticks) */
  return (time_diff > 0.0);
}

/** Determine if we have had a DGNSS timeout.
 *
 * \param _last_dgnss. Last time of DGNSS solution
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
static bool dgnss_timeout(piksi_systime_t *_last_dgnss,
                          dgnss_solution_mode_t _dgnss_soln_mode) {
  /* No timeout needed in low latency mode */
  if (STARLING_SOLN_MODE_LOW_LATENCY == _dgnss_soln_mode) {
    return false;
  }

  /* Need to compare timeout threshold in MS to system time elapsed (in system
   * ticks) */
  return (piksi_systime_elapsed_since_ms(_last_dgnss) > DGNSS_TIMEOUT_MS);
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
  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();
  /* Check to see if we have waited long enough since the last SPP update. */
  if (!spp_timeout(&last_spp, &last_dgnss, dgnss_soln_mode)) {
    return;
  }

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

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_ECEF,
                   sizeof(sbp_messages->baseline_ecef),
                   (u8 *)&sbp_messages->baseline_ecef);
    }

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_NED,
                   sizeof(sbp_messages->baseline_ned),
                   (u8 *)&sbp_messages->baseline_ned);
    }

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_AGE_CORRECTIONS,
                   sizeof(sbp_messages->age_corrections),
                   (u8 *)&sbp_messages->age_corrections);
    }

    if (dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_DGNSS_STATUS,
                   sizeof(sbp_messages->dgnss_status),
                   (u8 *)&sbp_messages->dgnss_status);
    }

    if (send_heading && dgnss_soln_mode != STARLING_SOLN_MODE_NO_DGNSS) {
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

void starling_integration_sbp_messages_init(sbp_messages_t *sbp_messages,
                                            const gps_time_t *epoch_time) {
  /* Necessary because some of these functions strip the const qualifier. */
  gps_time_t *t = (gps_time_t *)epoch_time;

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

/**
 * Accessed externally from starling_threads.c
 * TODO(kevin) fix this.
 */
void starling_integration_solution_send_low_latency_output(
    u8 base_sender_id,
    const sbp_messages_t *sbp_messages,
    u8 n_meas,
    const navigation_measurement_t nav_meas[]) {
  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();
  /* Work out if we need to wait for a certain period of no time matched
   * positions before we output a SBP position */
  bool wait_for_timeout = false;
  if (!(dgnss_timeout(&last_dgnss_stats.systime, dgnss_soln_mode)) &&
      STARLING_SOLN_MODE_TIME_MATCHED == dgnss_soln_mode) {
    wait_for_timeout = true;
  }

  if (!wait_for_timeout) {
    solution_send_pos_messages(base_sender_id, sbp_messages, n_meas, nav_meas);
    chMtxLock(&last_sbp_lock);
    last_spp.wn = sbp_messages->gps_time.wn;
    last_spp.tow = sbp_messages->gps_time.tow * 0.001;
    chMtxUnlock(&last_sbp_lock);
  }
}

static void solution_make_sbp(const pvt_engine_result_t *soln,
                              const dops_t *dops,
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

static void solution_make_baseline_sbp(const pvt_engine_result_t *result,
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

  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();

  if (result->flags == FIXED_POSITION &&
      dgnss_soln_mode == STARLING_SOLN_MODE_TIME_MATCHED) {
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

  chMtxLock(&last_sbp_lock);
  last_dgnss.wn = result->time.wn;
  last_dgnss.tow = result->time.tow;
  chMtxUnlock(&last_sbp_lock);

  /* Update stats */
  piksi_systime_get(&last_dgnss_stats.systime);
  last_dgnss_stats.mode =
      (result->flags == FIXED_POSITION) ? FILTER_FIXED : FILTER_FLOAT;
}

void starling_integration_solution_simulation(sbp_messages_t *sbp_messages) {
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

    double t_check = soln->time.tow * (soln_freq_setting / obs_output_divisor);
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

/*******************************************************************************
 * Settings Update Helpers
 ******************************************************************************/

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

static bool enable_fix_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool is_fix_enabled = (value != 0);
  starling_set_is_fix_enabled(is_fix_enabled);
  *(dgnss_filter_t *)s->addr = value;
  return ret;
}

static bool set_dgnss_soln_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  dgnss_solution_mode_t dgnss_soln_mode = value;
  starling_set_solution_mode(dgnss_soln_mode);
  *(dgnss_solution_mode_t *)s->addr = dgnss_soln_mode;
  return ret;
}

static bool set_max_age(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  starling_set_max_correction_age(value);
  *(int *)s->addr = value;
  return ret;
}

static bool set_is_glonass_enabled(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool is_glonass_enabled = (value != 0);
  starling_set_is_glonass_enabled(is_glonass_enabled);
  *(bool *)s->addr = is_glonass_enabled;
  return ret;
}

static bool set_glonass_downweight_factor(struct setting *s, const char *val) {
  float value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  starling_set_glonass_downweight_factor(value);
  *(float *)s->addr = value;
  return ret;
}

static bool set_disable_klobuchar(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool disable_klobuchar = (value != 0);
  starling_set_is_time_matched_klobuchar_enabled(!disable_klobuchar);
  *(bool *)s->addr = disable_klobuchar;
  return ret;
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

static void initialize_starling_settings(void) {
  static const char *const dgnss_filter_enum[] = {"Float", "Fixed", NULL};
  static struct setting_type dgnss_filter_setting;
  static dgnss_filter_t dgnss_filter_mode = FILTER_FIXED;
  int TYPE_GNSS_FILTER =
      settings_type_register_enum(dgnss_filter_enum, &dgnss_filter_setting);

  SETTING_NOTIFY("solution",
                 "dgnss_filter",
                 dgnss_filter_mode,
                 TYPE_GNSS_FILTER,
                 enable_fix_mode);

  static u32 max_age_of_differential = 0;
  SETTING_NOTIFY("solution",
                 "correction_age_max",
                 max_age_of_differential,
                 TYPE_INT,
                 set_max_age);

  SETTING_NOTIFY("solution",
                 "enable_glonass",
                 enable_glonass,
                 TYPE_BOOL,
                 set_is_glonass_enabled);

  static float glonass_downweight_factor = 4.0;
  SETTING_NOTIFY("solution",
                 "glonass_measurement_std_downweight_factor",
                 glonass_downweight_factor,
                 TYPE_FLOAT,
                 set_glonass_downweight_factor);

  static bool disable_klobuchar = false;
  SETTING_NOTIFY("solution",
                 "disable_klobuchar_correction",
                 disable_klobuchar,
                 TYPE_BOOL,
                 set_disable_klobuchar);

  static const char *const dgnss_soln_mode_enum[] = {
      "Low Latency", "Time Matched", "No DGNSS", NULL};
  static struct setting_type dgnss_soln_mode_setting;
  int TYPE_GNSS_SOLN_MODE = settings_type_register_enum(
      dgnss_soln_mode_enum, &dgnss_soln_mode_setting);
  static dgnss_solution_mode_t dgnss_soln_mode = STARLING_SOLN_MODE_LOW_LATENCY;
  SETTING_NOTIFY("solution",
                 "dgnss_solution_mode",
                 dgnss_soln_mode,
                 TYPE_GNSS_SOLN_MODE,
                 set_dgnss_soln_mode);

  SETTING("solution", "send_heading", send_heading, TYPE_BOOL);
  SETTING_NOTIFY("solution",
                 "heading_offset",
                 heading_offset,
                 TYPE_FLOAT,
                 heading_offset_changed);
}

static THD_FUNCTION(initialize_and_run_starling, arg) {
  (void)arg;
  chRegSetThreadName("starling");

  initialize_starling_settings();

  /* Set time of last differential solution in the past. */
  last_dgnss = GPS_TIME_UNKNOWN;
  last_spp = GPS_TIME_UNKNOWN;

  /* Register a reset callback. */
  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
      SBP_MSG_RESET_FILTERS, &reset_filters_callback, &reset_filters_node);

  /* This runs forever. */
  starling_run();

  /* Never get here. */
  log_error("Starling Engine has unexpectedly terminated.");
  assert(0);
  for (;;) {
  }
}

/*******************************************************************************
 * Starling Output Callbacks
 ******************************************************************************/

/**
 * Pass along a time-matched solution to the outside world.
 *
 * The solution pointer may optionally be NULL if there was no
 * valid solution for this epoch of processing. The observation
 * pointers are expected to always be valid.
 *
 * NOTE: The pointers are only valid within the enclosing scope.
 *       Any copies of the data must be deep copies.
 */
void send_solution_time_matched(const StarlingFilterSolution *solution,
                                const obss_t *obss_base,
                                const obss_t *obss_rover) {
  assert(obss_base);
  assert(obss_rover);
  /* Fill in the output messages. We always use the SPP message first.
   * Then if there is a successful time-matched result, we will
   * overwrite the relevant messages. */
  sbp_messages_t sbp_messages;
  starling_integration_sbp_messages_init(&sbp_messages, &obss_base->tor);

  pvt_engine_result_t soln_copy = obss_rover->soln;
  solution_make_sbp(&soln_copy, NULL, &sbp_messages);

  if (solution) {
    solution_make_baseline_sbp(&solution->result,
                               obss_rover->pos_ecef,
                               &solution->dops,
                               &sbp_messages);
  }

  solution_send_pos_messages(
      obss_base->sender_id, &sbp_messages, obss_rover->n, obss_rover->nm);

  /* Always keep track of which base station is sending in the
   * base observations. */
  chMtxLock(&current_base_sender_id_lock);
  current_base_sender_id = obss_base->sender_id;
  chMtxUnlock(&current_base_sender_id_lock);
}

/**
 * Pass along a low-latency solution to the outside world.
 *
 * At every processing epoch, possible solutions include both
 * an SPP solution, and an RTK solution. Either one may be invalid,
 * (indicated by NULL pointer), although existence of an RTK
 * solution implies existence of an SPP solution.
 *
 * NOTE: The pointers are only valid within the enclosing scope.
 *       Any copies of the data must be deep copies.
 */
void send_solution_low_latency(const StarlingFilterSolution *spp_solution,
                               const StarlingFilterSolution *rtk_solution,
                               const gps_time_t *solution_epoch_time,
                               const navigation_measurement_t *nav_meas,
                               const size_t num_nav_meas) {
  assert(solution_epoch_time);
  assert(nav_meas);

  /* Initialize the output messages. If there is an SPP solution, we first
   * apply that. Then if there is an RTK solution, overwrite the relevant
   * messages with the RTK baseline result. When there are no valid
   * solutions, we simply pass on the set of default messages. */
  sbp_messages_t sbp_messages;
  starling_integration_sbp_messages_init(&sbp_messages, solution_epoch_time);

  u8 base_sender_id = STARLING_BASE_SENDER_ID_DEFAULT;
  if (spp_solution) {
    solution_make_sbp(
        &spp_solution->result, &spp_solution->dops, &sbp_messages);
    if (rtk_solution) {
      solution_make_baseline_sbp(&rtk_solution->result,
                                 spp_solution->result.baseline,
                                 &rtk_solution->dops,
                                 &sbp_messages);

      chMtxLock(&current_base_sender_id_lock);
      base_sender_id = current_base_sender_id;
      chMtxUnlock(&current_base_sender_id_lock);
    }
  }
  starling_integration_solution_send_low_latency_output(
      base_sender_id, &sbp_messages, num_nav_meas, nav_meas);
}

/*******************************************************************************
 * Starling Integration API
 ******************************************************************************/

void starling_calc_pvt_setup() {
  /* Start main starling thread. */
  chThdCreateStatic(wa_starling_thread,
                    sizeof(wa_starling_thread),
                    STARLING_THREAD_PRIORITY,
                    initialize_and_run_starling,
                    NULL);
}

soln_dgnss_stats_t solution_last_dgnss_stats_get(void) {
  return last_dgnss_stats;
}

soln_pvt_stats_t solution_last_pvt_stats_get(void) { return last_pvt_stats; }
