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
#include <libswiftnav/pvt_engine/vehicle_dynamics_filter.h>
#include <starling/starling.h>
#include <starling/starling_platform.h>

#include "calc/calc_pvt_common.h"
#include "calc/calc_pvt_me.h"
#include "calc/starling_integration.h"
#include "calc/starling_obs_converter.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings.h"
#include "simulator/simulator.h"
#include "utils/timing/timing.h"

#include "board/v3/nap/nap_hw.h"
#include "timing/timing.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/

/** Number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

/* Size of an spp solution in ECEF. */
#define SPP_ECEF_SIZE 3

#define STARLING_BASE_SENDER_ID_DEFAULT 0

/*******************************************************************************
 * Types
 ******************************************************************************/

/* Set of messages sent by the Piksi Multi integration of Starling. */
typedef struct {
  msg_gps_time_t gps_time;
  msg_utc_time_t utc_time;
  msg_pos_llh_t pos_llh;
  msg_pos_ecef_t pos_ecef;
  msg_vel_ned_t vel_ned;
  msg_vel_ecef_t vel_ecef;
  msg_dops_t sbp_dops;
  msg_age_corrections_t age_corrections;
  msg_dgnss_status_t dgnss_status;
  msg_baseline_ecef_t baseline_ecef;
  msg_baseline_ned_t baseline_ned;
  msg_baseline_heading_t baseline_heading;
  msg_pos_ecef_cov_t pos_ecef_cov;
  msg_vel_ecef_cov_t vel_ecef_cov;
  msg_pos_llh_cov_t pos_llh_cov;
  msg_vel_ned_cov_t vel_ned_cov;
} sbp_messages_t;

/*******************************************************************************
 * Globals
 ******************************************************************************/
bool enable_glonass = true;
bool enable_galileo = true;
bool enable_beidou = true;
bool send_heading = false;
double heading_offset = 0.0;

/*******************************************************************************
 * Locals
 ******************************************************************************/

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

/* We are going to keep around several of these, and switch between
 * them at runtime. */
static VehicleDynamicsFilter *default_dynamics_filter = NULL;
static VehicleDynamicsFilter *tractor_dynamics_filter = NULL;
static VehicleDynamicsFilter *dynamics_filter = NULL;
/* Settings for the dynamics filters. */
MUTEX_DECL(dynamics_filter_lock);
struct DynamicsFilterSettings {
  double lowpass_constant;
  double max_acceleration;
} dynamics_filter_settings = {.lowpass_constant = 3.0, .max_acceleration = 0.5};

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
  /* Because this function is used only on the time-matched thread,
   * the time-matched output can be considered "indefinitely timed-out"
   * when in low-latency mode. */
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

static void starling_integration_solution_send_low_latency_output(
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
    sbp_make_gps_time(&sbp_messages->gps_time, &soln->time, POSITION_MODE_SPP);
    sbp_make_utc_time(&sbp_messages->utc_time, &soln->time, POSITION_MODE_SPP);

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
                          soln->flags.position_mode);

    sbp_make_pos_llh_cov(&sbp_messages->pos_llh_cov,
                         pos_llh,
                         pos_ned_cov,
                         &soln->time,
                         soln->num_sats_used,
                         soln->flags.position_mode);

    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef,
                           pos_ecef,
                           accuracy,
                           &soln->time,
                           soln->num_sats_used,
                           soln->flags.position_mode);

    sbp_make_pos_ecef_cov(&sbp_messages->pos_ecef_cov,
                          pos_ecef,
                          pos_ecef_cov,
                          &soln->time,
                          soln->num_sats_used,
                          soln->flags.position_mode);

    if (soln->velocity_valid) {
      double vel_ned[3];
      wgsecef2ned(soln->velocity, pos_ecef, vel_ned);
      sbp_make_vel_ned(&sbp_messages->vel_ned,
                       vel_ned,
                       vel_h_accuracy,
                       vel_v_accuracy,
                       &soln->time,
                       soln->num_sats_used,
                       soln->flags.velocity_mode);

      sbp_make_vel_ned_cov(&sbp_messages->vel_ned_cov,
                           vel_ned,
                           vel_ned_cov,
                           &soln->time,
                           soln->num_sats_used,
                           soln->flags.velocity_mode);

      sbp_make_vel_ecef(&sbp_messages->vel_ecef,
                        soln->velocity,
                        vel_accuracy,
                        &soln->time,
                        soln->num_sats_used,
                        soln->flags.velocity_mode);

      sbp_make_vel_ecef_cov(&sbp_messages->vel_ecef_cov,
                            soln->velocity,
                            vel_ecef_cov,
                            &soln->time,
                            soln->num_sats_used,
                            soln->flags.velocity_mode);
    }

    /* DOP message can be sent even if solution fails to compute */
    if (dops) {
      sbp_make_dops(&sbp_messages->sbp_dops,
                    dops,
                    sbp_messages->pos_llh.tow,
                    soln->flags.position_mode);
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
                         result->flags.position_mode);

  sbp_make_baseline_ned(&sbp_messages->baseline_ned,
                        &result->time,
                        result->num_sats_used,
                        b_ned,
                        h_accuracy,
                        v_accuracy,
                        result->flags.position_mode);

  sbp_make_age_corrections(
      &sbp_messages->age_corrections, &result->time, result->propagation_time);

  sbp_make_dgnss_status(&sbp_messages->dgnss_status,
                        result->num_sats_used,
                        result->propagation_time,
                        result->flags.position_mode);

  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();

  if (result->flags.position_mode == POSITION_MODE_FIXED &&
      dgnss_soln_mode == STARLING_SOLN_MODE_TIME_MATCHED) {
    double heading = calc_heading(b_ned);
    sbp_make_heading(&sbp_messages->baseline_heading,
                     &result->time,
                     heading + heading_offset,
                     result->num_sats_used,
                     result->flags.position_mode);
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
                          result->flags.position_mode);
    sbp_make_pos_llh_cov(&sbp_messages->pos_llh_cov,
                         pseudo_absolute_llh,
                         pos_ned_cov,
                         &result->time,
                         result->num_sats_used,
                         result->flags.position_mode);
    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef,
                           pseudo_absolute_ecef,
                           accuracy,
                           &result->time,
                           result->num_sats_used,
                           result->flags.position_mode);
    sbp_make_pos_ecef_cov(&sbp_messages->pos_ecef_cov,
                          pseudo_absolute_ecef,
                          pos_ecef_cov,
                          &result->time,
                          result->num_sats_used,
                          result->flags.position_mode);
  }
  sbp_make_dops(&sbp_messages->sbp_dops,
                dops,
                sbp_messages->pos_llh.tow,
                result->flags.position_mode);

  chMtxLock(&last_sbp_lock);
  last_dgnss.wn = result->time.wn;
  last_dgnss.tow = result->time.tow;
  chMtxUnlock(&last_sbp_lock);

  /* Update stats */
  piksi_systime_get(&last_dgnss_stats.systime);
  last_dgnss_stats.mode = (result->flags.position_mode == POSITION_MODE_FIXED)
                              ? FILTER_FIXED
                              : FILTER_FLOAT;
}

/*******************************************************************************
 * Simulation Helpers
 ******************************************************************************/
static void starling_integration_solution_simulation(
    sbp_messages_t *sbp_messages) {
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
    u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK)
                   ? POSITION_MODE_FIXED
                   : POSITION_MODE_FLOAT;

    pvt_engine_result_t result = {
        .time = soln->time,
        .num_sats_used = simulation_current_num_sats(),
        .num_sigs_used = 0,
        .flags = {flags, 0, 0},
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

void starling_integration_simulation_run(const me_msg_obs_t *me_msg) {
  gps_time_t epoch_time = me_msg->obs_time;
  if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= get_time_quality()) {
    /* observations do not have valid time, but we have a reasonable estimate
     * of current GPS time, so round that to nearest epoch and use it
     */
    epoch_time = get_current_time();
    epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
  }
  sbp_messages_t sbp_messages;
  starling_integration_sbp_messages_init(&sbp_messages, &epoch_time);
  starling_integration_solution_simulation(&sbp_messages);
  const u8 fake_base_sender_id = 1;
  starling_integration_solution_send_low_latency_output(
      fake_base_sender_id, &sbp_messages, me_msg->size, me_msg->obs);
}

bool starling_integration_simulation_enabled(void) {
  return simulation_enabled();
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

static bool setting_notify_vehicle_dynamics_filter_mode(struct setting *s,
                                                        const char *val) {
  bool ret = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!ret) {
    return false;
  }
  VehicleDynamicsFilter *new_filter = NULL;
  switch (*(VehicleDynamicsFilterType *)s->addr) {
    case DYNAMICS_NONE:
      new_filter = default_dynamics_filter;
      break;
    case DYNAMICS_TRACTOR:
      new_filter = tractor_dynamics_filter;
      break;
    case DYNAMICS_DRONE:
    case DYNAMICS_INTERGALACTIC_VOYAGER:
    default:
      break;
  }
  if (NULL != new_filter) {
    /* Reset the new filter whenever it is changed. */
    chMtxLock(&dynamics_filter_lock);
    dynamics_filter = new_filter;
    vehicle_dynamics_filter_reset(dynamics_filter);
    chMtxUnlock(&dynamics_filter_lock);
    return true;
  } else {
    log_error(
        "Invalid Vehicle Dynamics Filter mode selection. No change made.");
    return false;
  }
}

static bool setting_notify_vehicle_dynamics_filter_param(struct setting *s,
                                                         const char *val) {
  chMtxLock(&dynamics_filter_lock);
  bool ret = s->type->from_string(s->type->priv, s->addr, s->len, val);
  chMtxUnlock(&dynamics_filter_lock);
  return ret;
}

static bool set_max_age(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, s->addr, s->len, val);
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

static bool set_is_galileo_enabled(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool is_galileo_enabled = (value != 0);
  starling_set_is_galileo_enabled(is_galileo_enabled);
  *(bool *)s->addr = is_galileo_enabled;
  return ret;
}

static bool set_is_beidou_enabled(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool is_beidou_enabled = (value != 0);
  starling_set_is_beidou_enabled(is_beidou_enabled);
  *(bool *)s->addr = is_beidou_enabled;
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
      starling_reset_rtk_filter();
      break;
    default:
      break;
  }
}

/* Add SBAS data to the Starling engine. */
void starling_add_sbas_data(const sbas_raw_data_t *sbas_data,
                            const size_t n_sbas_data) {
  for (size_t i = 0; i < n_sbas_data; ++i) {
    sbas_raw_data_t *sbas_data_msg =
        platform_mailbox_item_alloc(MB_ID_SBAS_DATA);
    if (NULL == sbas_data_msg) {
      log_error("platform_mailbox_item_alloc(MB_ID_SBAS_DATA) failed!");
      continue;
    }
    assert(sbas_data);
    *sbas_data_msg = *sbas_data;
    errno_t ret =
        platform_mailbox_post(MB_ID_SBAS_DATA, sbas_data_msg, MB_NONBLOCKING);
    if (ret != 0) {
      log_error("platform_mailbox_post(MB_ID_SBAS_DATA) failed!");
      platform_mailbox_item_free(MB_ID_SBAS_DATA, sbas_data_msg);
    }
  }
}

/**
 * Simply apply whatever the current settings are to the
 * given dynamics filter.
 */
static void update_dynamics_filter_settings(VehicleDynamicsFilter *filter) {
  vehicle_dynamics_filter_set_param(filter,
                                    VEHICLE_DYNAMICS_LOWPASS_TIME_CONSTANT_S,
                                    dynamics_filter_settings.lowpass_constant);
  vehicle_dynamics_filter_set_param(
      filter,
      VEHICLE_DYNAMICS_MAX_LINEAR_ACCELERATION_MS2,
      dynamics_filter_settings.max_acceleration);
}

/**
 * This function behaves in a somewhat unexpected way.
 * Given two (possibly NULL) solutions, this function chooses
 * the "better" of the two solutions and feeds it through the
 * dynamics filter. If they are both null, the output parameter
 * won't be touched.
 */
static void apply_dynamics_filter_to_solutions(
    const StarlingFilterSolution *spp_solution,
    const StarlingFilterSolution *rtk_solution,
    pvt_engine_result_t *output) {
  pvt_engine_result_t const *input = NULL;
  if (rtk_solution) {
    input = &rtk_solution->result;
  } else if (spp_solution) {
    input = &spp_solution->result;
  }
  if (input && output) {
    chMtxLock(&dynamics_filter_lock);
    update_dynamics_filter_settings(dynamics_filter);
    VehicleDynamicsFilter *current_filter = dynamics_filter;
    chMtxUnlock(&dynamics_filter_lock);

    vehicle_dynamics_filter_process(current_filter, input, output);
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

  /* Only send time-matched output if we are not in low-latency mode
   * and our current time-matched result occurs after the most recent
   * SPP output. */
  if (spp_timeout(&last_spp, &last_dgnss, starling_get_solution_mode())) {
    /* Notify the user that the vehicle dynamics filter has no effect in
     * time-matched mode (if applicable). */
    if (dynamics_filter != default_dynamics_filter) {
      log_warn(
          "Non-default Vehicle Dynamics Filter has no effect in Time-Matched "
          "mode.");
    }
    solution_send_pos_messages(
        obss_base->sender_id, &sbp_messages, obss_rover->n, obss_rover->nm);
  }

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

  /* Apply the vehicle dynamics filter when enabled.
   * We need to make sure to pass through the most accurate solution
   * available -- RTK is preferred over SPP. */
  pvt_engine_result_t filtered_pvt_result = {0};
  apply_dynamics_filter_to_solutions(
      spp_solution, rtk_solution, &filtered_pvt_result);

  /* (July 2018) We always output SPP velocity, so whatever
   * the filtered result is, we want to make sure we overwrite
   * the velocity with the SPP velocity. */
  if (spp_solution) {
    filtered_pvt_result.velocity_valid = spp_solution->result.velocity_valid;
    MEMCPY_S(&filtered_pvt_result.velocity,
             3 * sizeof(double),
             &spp_solution->result.velocity,
             3 * sizeof(double));
    MEMCPY_S(&filtered_pvt_result.velocity_covariance,
             9 * sizeof(double),
             &spp_solution->result.velocity_covariance,
             9 * sizeof(double));
  } else {
    filtered_pvt_result.velocity_valid = false;
  }

  /* Check if observations do not have valid time. We may have locally a
   * reasonable estimate of current GPS time, so we can round that to the
   * nearest epoch and use instead if necessary.
   */
  gps_time_t epoch_time = *solution_epoch_time;
  if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= get_time_quality()) {
    epoch_time = get_current_time();
    epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
  }

  /* Initialize the output messages. If there is an SPP solution, we first
   * apply that. Then if there is an RTK solution, overwrite the relevant
   * messages with the RTK baseline result. When there are no valid
   * solutions, we simply pass on the set of default messages. */
  sbp_messages_t sbp_messages;
  starling_integration_sbp_messages_init(&sbp_messages, &epoch_time);

  u8 base_sender_id = STARLING_BASE_SENDER_ID_DEFAULT;
  if (spp_solution) {
    solution_make_sbp(&filtered_pvt_result, &spp_solution->dops, &sbp_messages);
    if (rtk_solution) {
      solution_make_baseline_sbp(&filtered_pvt_result,
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
 * Starling Initialization
 ******************************************************************************/

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

  static u32 max_age_of_differential = 30;
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

  SETTING_NOTIFY("solution",
                 "enable_galileo",
                 enable_galileo,
                 TYPE_BOOL,
                 set_is_galileo_enabled);

  SETTING_NOTIFY("solution",
                 "enable_beidou",
                 enable_beidou,
                 TYPE_BOOL,
                 set_is_beidou_enabled);

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

static void initialize_vehicle_dynamics_filters(void) {
  default_dynamics_filter = vehicle_dynamics_filter_create(DYNAMICS_NONE);
  tractor_dynamics_filter = vehicle_dynamics_filter_create(DYNAMICS_TRACTOR);

  dynamics_filter = default_dynamics_filter;

  /* TODO(kevin) register settings. */
  static const char *const vehicle_dynamics_filter_mode_enum[] = {
      "None", "Tractor", NULL};
  static struct setting_type vehicle_dynamics_filter_mode_setting;
  int TYPE_VEHICLE_DYNAMICS_FILTER_MODE = settings_type_register_enum(
      vehicle_dynamics_filter_mode_enum, &vehicle_dynamics_filter_mode_setting);
  static VehicleDynamicsFilterType vehicle_dynamics_filter_mode = DYNAMICS_NONE;
  SETTING_NOTIFY("vehicle_dynamics_filter",
                 "mode",
                 vehicle_dynamics_filter_mode,
                 TYPE_VEHICLE_DYNAMICS_FILTER_MODE,
                 setting_notify_vehicle_dynamics_filter_mode);

  SETTING_NOTIFY("vehicle_dynamics_filter",
                 "lowpass_time_constant",
                 dynamics_filter_settings.lowpass_constant,
                 TYPE_FLOAT,
                 setting_notify_vehicle_dynamics_filter_param);

  SETTING_NOTIFY("vehicle_dynamics_filter",
                 "max_acceleration",
                 dynamics_filter_settings.max_acceleration,
                 TYPE_FLOAT,
                 setting_notify_vehicle_dynamics_filter_param);
}

static void profile_low_latency_thread(enum ProfileDirective directive) {
  static float avg_run_time_s = 0.1f;
  static float diff_run_time_s = 0.1f;
  static float avg_diff_run_time_s = 0.0f;
  static float std_run_time_s = 0.1f;
  const float smooth_factor = 0.01f;
  u32 nap_snapshot_begin = 0;
  switch (directive) {
    case PROFILE_BEGIN:
      nap_snapshot_begin = NAP->TIMING_COUNT;
      break;
    case PROFILE_END: {
      u32 nap_snapshot_diff = (u32)(NAP->TIMING_COUNT - nap_snapshot_begin);
      float time_snapshot_diff = RX_DT_NOMINAL * nap_snapshot_diff;
      avg_run_time_s = avg_run_time_s * (1 - smooth_factor) +
                       time_snapshot_diff * smooth_factor;
      diff_run_time_s = (time_snapshot_diff - avg_run_time_s);
      avg_diff_run_time_s = avg_diff_run_time_s * (1 - smooth_factor) +
                            (diff_run_time_s * diff_run_time_s) * smooth_factor;
      std_run_time_s = sqrtf(avg_diff_run_time_s);
      if (diff_run_time_s > 3.0f * std_run_time_s) {
        log_warn("time_snapshot_diff %f average %f std %f",
                 time_snapshot_diff,
                 avg_run_time_s,
                 std_run_time_s);
      }
    } break;
    default:
      log_warn("Bad profile directive.");
      break;
  }
}

/* Determines how long each read operation will block for. */
#define READ_OBS_ROVER_TIMEOUT DGNSS_TIMEOUT_MS
#define READ_OBS_BASE_TIMEOUT DGNSS_TIMEOUT_MS

/* TODO(kevin) refactor common code. */
static int read_obs_rover(int blocking, me_msg_obs_t *me_msg) {
  me_msg_obs_t *local_me_msg = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_ME_OBS, (void **)&local_me_msg, blocking);
  if (local_me_msg) {
    if (STARLING_READ_OK == ret) {
      *me_msg = *local_me_msg;
    } else {
      /* Erroneous behavior for fetch to return non-NULL pointer and indicate
       * read failure. */
      log_error("Rover obs mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_ME_OBS, local_me_msg);
  }
  return ret;
}

/* TODO(kevin) refactor common code. */
static int read_obs_base(int blocking, obss_t *obs) {
  obs_array_t *new_obs_array = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_BASE_OBS, (void **)&new_obs_array, blocking);
  if (new_obs_array) {
    if (STARLING_READ_OK == ret) {
      ret = convert_starling_obs_array_to_obss(new_obs_array, obs);
    } else {
      /* Erroneous behavior for fetch to return non-NULL pointer and indicate
       * read failure. */
      log_error("Base obs mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_BASE_OBS, new_obs_array);
  }
  return ret;
}

/* TODO(kevin) refactor common code. */
static int read_sbas_data(int blocking, sbas_raw_data_t *data) {
  sbas_raw_data_t *local_data = NULL;
  errno_t ret =
      platform_mailbox_fetch(MB_ID_SBAS_DATA, (void **)&local_data, blocking);
  if (local_data) {
    if (STARLING_READ_OK == ret) {
      *data = *local_data;
    } else {
      /* Erroneous behavior for fetch to return non-NULL pointer and indicate
       * read failure. */
      log_error("STARLING: sbas mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_SBAS_DATA, local_data);
  }
  return ret;
}

static int read_ephemeris_array(int blocking, ephemeris_array_t *eph_arr) {
  ephemeris_array_t *local_eph_arr = NULL;
  errno_t ret = platform_mailbox_fetch(
      MB_ID_EPHEMERIS, (void **)&local_eph_arr, blocking);
  if (local_eph_arr) {
    if (STARLING_READ_OK == ret) {
      eph_arr->n = local_eph_arr->n;
      if (local_eph_arr->n > 0) {
        MEMCPY_S(eph_arr->ephemerides,
                 sizeof(eph_arr->ephemerides),
                 local_eph_arr->ephemerides,
                 local_eph_arr->n * sizeof(ephemeris_t));
      }
    } else {
      log_error("STARLING: ephemeris mailbox fetch failed with %d", ret);
    }
    platform_mailbox_item_free(MB_ID_EPHEMERIS, local_eph_arr);
  }
  return ret;
}

static THD_FUNCTION(initialize_and_run_starling, arg) {
  (void)arg;
  chRegSetThreadName("starling");

  initialize_vehicle_dynamics_filters();
  initialize_starling_settings();

  /* Set time of last differential solution in the past. */
  last_dgnss = GPS_TIME_UNKNOWN;
  last_spp = GPS_TIME_UNKNOWN;

  /* Register a reset callback. */
  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
      SBP_MSG_RESET_FILTERS, &reset_filters_callback, &reset_filters_node);

  StarlingIoFunctionTable io_functions = {
      .read_obs_rover = read_obs_rover,
      .read_obs_base = read_obs_base,
      .read_sbas_data = read_sbas_data,
      .read_ephemeris_array = read_ephemeris_array,
      .read_imu = NULL,
      .handle_solution_low_latency = send_solution_low_latency,
      .handle_solution_time_matched = send_solution_time_matched,
  };

  StarlingDebugFunctionTable debug_functions = {
      .profile_low_latency_thread = profile_low_latency_thread,
  };

  /* This runs forever. */
  starling_run(&io_functions, &debug_functions);

  /* Never get here. */
  log_error("Starling Engine has unexpectedly terminated.");
  assert(0);
  for (;;) {
  }
}

/*******************************************************************************
 * Starling Integration API
 ******************************************************************************/

void starling_calc_pvt_setup() {
  /* Start main starling thread. */
  platform_thread_create(THREAD_ID_STARLING, initialize_and_run_starling);
}

soln_dgnss_stats_t solution_last_dgnss_stats_get(void) {
  return last_dgnss_stats;
}

soln_pvt_stats_t solution_last_pvt_stats_get(void) { return last_pvt_stats; }
