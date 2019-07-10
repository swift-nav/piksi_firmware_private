/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "starling_integration.h"

#include <assert.h>
#include <ch.h>
#include <starling/integration/starling_input_bridge.h>
#include <starling/platform/mutex.h>
#include <starling/platform/thread.h>
#include <starling/starling.h>
#include <starling/starling_external_dependencies.h>
#include <string.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/memcpy_s.h>

#include "board/v3/nap/nap_hw.h"
#include "calc/calc_pvt_common.h"
#include "calc/calc_pvt_me.h"
#include "calc/sbp_settings_client.h"
#include "calc/starling_sbp_link.h"
#include "calc/starling_sbp_output.h"
#include "calc/starling_sbp_settings.h"
#include "cfg/init.h"
#include "ndb/ndb.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "shm/shm.h"
#include "simulator/simulator.h"
#include "timing/timing.h"
#include "track/track_sid_db.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/

/* Number of seconds of no time-matched after which low-latency will resume. */
#define LOW_LATENCY_RESUME_AFTER_SEC 5.0

/* Size of an spp solution in ECEF. */
#define SPP_ECEF_SIZE 3

#define STARLING_BASE_SENDER_ID_DEFAULT 0

/*******************************************************************************
 * Locals
 ******************************************************************************/
static MUTEX_DECL(last_sbp_lock);
static gps_time_t last_sbp_dgnss = GPS_TIME_UNKNOWN;
static gps_time_t last_sbp_low_latency = GPS_TIME_UNKNOWN;

/*******************************************************************************/
static MUTEX_DECL(piksi_solution_info_lock);
static piksi_solution_info_t piksi_solution_info = {
    .last_time_spp = PIKSI_SYSTIME_INIT,
    .last_time_rtk = PIKSI_SYSTIME_INIT,
    .was_last_rtk_fix = false,
};

/*******************************************************************************/
void piksi_solution_info_get(piksi_solution_info_t *info) {
  chMtxLock(&piksi_solution_info_lock);
  *info = piksi_solution_info;
  chMtxUnlock(&piksi_solution_info_lock);
}

/*******************************************************************************
 * Output Callback Helpers
 ******************************************************************************/

static bool is_raim_disabled(void) { return disable_raim; }

static cache_ret_t cache_read_ephemeris(const gnss_signal_t sid,
                                        ephemeris_t *eph) {
  ndb_op_code_t ret = ndb_ephemeris_read(sid, eph);
  if (NDB_ERR_NONE == ret) {
    return CACHE_OK;
  }
  if (NDB_ERR_UNCONFIRMED_DATA == ret) {
    return CACHE_OK_UNCONFIRMED_DATA;
  }
  return CACHE_ERROR;
}

static cache_ret_t cache_read_iono_corr(ionosphere_t *iono) {
  ndb_op_code_t ret = ndb_iono_corr_read(iono);
  if (NDB_ERR_NONE == ret) {
    return CACHE_OK;
  }
  if (NDB_ERR_UNCONFIRMED_DATA == ret) {
    return CACHE_OK_UNCONFIRMED_DATA;
  }
  return CACHE_ERROR;
}

static double calc_heading(const double b_ned[3]) {
  double heading = atan2(b_ned[1], b_ned[0]);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading * R2D;
}

typedef struct {
  const utc_params_t *utc_params;
  u8 flags;
} utc_details;

// A helper function that attempts to read the UTC parameters from NDB
// In the return value it sets all of the flags needed to make an SBP UTC time
// message and a pointer to the read UTC params. If the reading fails the
// pointer in the return value is set to NULL.
static utc_details get_utc_info(utc_params_t *utc_params, const u8 time_qual) {
  utc_details result;
  bool is_nv;

  result.flags = (sbp_get_time_quality_flags(time_qual) & 0x7);
  result.utc_params = NULL;

  if (TIME_UNKNOWN != time_qual &&
      NDB_ERR_NONE == ndb_utc_params_read(utc_params, &is_nv)) {
    result.utc_params = utc_params;
    if (is_nv) {
      result.flags |= (NVM_UTC << 3);
    } else {
      result.flags |= (DECODED_UTC << 3);
    }
  } else {
    result.flags |= (DEFAULT_UTC << 3);
  }

  return result;
}

void starling_integration_sbp_messages_init(sbp_messages_t *sbp_messages,
                                            const gps_time_t *epoch_time,
                                            u8 time_qual) {
  /* Necessary because some of these functions strip the const qualifier. */
  gps_time_t *t = (gps_time_t *)epoch_time;
  /* if there is ANY time known here better than propagated,
   * initialize time_qual as time_propagated for SBP output.
   * If we have a GNSS solution, we will override with the sbp GNSS Solution
   * time quality */
  u8 sbp_time_qual = (TIME_PROPAGATED <= time_qual) ? TIME_PROPAGATED : 0;
  sbp_init_gps_time(&sbp_messages->gps_time, t, sbp_time_qual);

  utc_params_t utc_params;
  utc_details details = get_utc_info(&utc_params, time_qual);
  sbp_init_utc_time(
      &sbp_messages->utc_time, t, details.utc_params, details.flags);

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

static void send_low_latency_messages(const gps_time_t *time_of_solution,
                                      const sbp_messages_t *sbp_messages) {
  dgnss_solution_mode_t mode = starling_get_solution_mode();

  /* This is ridiculously confusing, allow me to explain:
   *
   * When in "TIME-MATCHED" mode, we want to recognize when the
   * solution hasn't been sent in a while and allow the "LOW-LATENCY"
   * messages through. On the other hand, if there has been a
   * sufficiently recent "TIME-MATCHED" output, we do not want
   * to send any "LOW-LATENCY" messages.
   *
   * In "LOW-LATENCY" mode, all of this is moot.
   */
  chMtxLock(&last_sbp_lock);
  if (!gps_time_valid(&last_sbp_dgnss)) {
    last_sbp_dgnss = *time_of_solution;
  }
  const gps_time_t last_dgnss_time = last_sbp_dgnss;
  chMtxUnlock(&last_sbp_lock);
  const double elapsed_time_sec =
      gpsdifftime(time_of_solution, &last_dgnss_time);
  if (STARLING_SOLN_MODE_TIME_MATCHED == mode &&
      elapsed_time_sec < LOW_LATENCY_RESUME_AFTER_SEC) {
    return;
  }

  solution_send_pos_messages(sbp_messages);
  chMtxLock(&last_sbp_lock);
  last_sbp_low_latency = *time_of_solution;
  chMtxUnlock(&last_sbp_lock);
}

static void solution_make_sbp(const pvt_engine_result_t *soln,
                              const dops_t *dops,
                              sbp_messages_t *sbp_messages,
                              u8 time_qual) {
  if (soln && soln->valid) {
    sbp_make_gps_time(&sbp_messages->gps_time, &soln->time, time_qual);

    utc_params_t utc_params;
    utc_details details = get_utc_info(&utc_params, time_qual);
    sbp_make_utc_time(&sbp_messages->utc_time,
                      &soln->time,
                      details.utc_params,
                      details.flags);

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
  }
}

static void solution_make_baseline_sbp(const pvt_engine_result_t *result,
                                       const double spp_ecef[SPP_ECEF_SIZE],
                                       const dops_t *dops,
                                       sbp_messages_t *sbp_messages) {
  double ecef_pos[3];
  if (result->has_known_reference_pos) {
    MEMCPY_S(ecef_pos,
             sizeof(ecef_pos),
             result->known_reference_pos,
             SPP_ECEF_SIZE * sizeof(double));
  } else {
    vector_subtract(3, spp_ecef, result->baseline, ecef_pos);
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

  dgnss_solution_mode_t mode = starling_get_solution_mode();

  if (result->flags.position_mode == POSITION_MODE_FIXED &&
      mode == STARLING_SOLN_MODE_TIME_MATCHED) {
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
  last_sbp_dgnss = result->time;
  chMtxUnlock(&last_sbp_lock);
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
    solution_make_sbp(
        soln, simulation_current_dops_solution(), sbp_messages, TIME_FINEST);
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
      send_observations(simulation_current_obs(), SBP_FRAMING_MAX_PAYLOAD_SIZE);
    }
  }
}

void starling_integration_simulation_run(const obss_t *obss) {
  gps_time_t epoch_time = obss->tor;
  u8 time_qual = get_time_quality();
  if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= time_qual) {
    /* observations do not have valid time, but we have a reasonable estimate
     * of current GPS time, so round that to nearest epoch and use it
     */
    epoch_time = get_current_time();
    epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
  }
  sbp_messages_t sbp_messages;
  starling_integration_sbp_messages_init(&sbp_messages, &epoch_time, time_qual);
  starling_integration_solution_simulation(&sbp_messages);
  send_low_latency_messages(&epoch_time, &sbp_messages);
}

bool starling_integration_simulation_enabled(void) {
  return simulation_enabled();
}

/*******************************************************************************
 * Settings Update Helpers
 ******************************************************************************/

static void reset_filters_callback(u16 sender_id,
                                   u8 len,
                                   u8 msg[], /* NOLINT */
                                   void *context) {
  (void)sender_id;
  (void)len;
  (void)context;
  switch (msg[0]) {
    case 0:
      log_info("Filter reset requested");
      starling_reset_time_matched_filter();
      break;
    default:
      break;
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
void handle_solution_time_matched(const StarlingFilterSolution *solution,
                                  const obs_core_t *obs_base,
                                  const pvt_engine_result_t *rover_soln,
                                  const double *rover_spp_ecef) {
  assert(obs_base);
  assert(rover_soln);
  assert(rover_spp_ecef);

  if (STARLING_SOLN_MODE_TIME_MATCHED != starling_get_solution_mode()) {
    return;
  }

  /* Fill in the output messages. We always use the SPP message first.
   * Then if there is a successful time-matched result, we will
   * overwrite the relevant messages. */
  sbp_messages_t sbp_messages;
  u8 time_qual = get_time_quality();
  starling_integration_sbp_messages_init(
      &sbp_messages, &obs_base->t, time_qual);

  pvt_engine_result_t soln_copy = (*rover_soln);

  /* TODO: Actually get the timing quality from ME / Starling
   * the time quality could have degraded or improved AFTER this
   * solution epoch was calculated. Thus time quality isn't strictly
   * true but it is good enough.  The plumbing exercise remains to remove
   * discrepancy between current time qual and this epoch's time qual */

  solution_make_sbp(&soln_copy, NULL, &sbp_messages, time_qual);

  if (solution) {
    solution_make_baseline_sbp(
        &solution->result, rover_spp_ecef, &solution->dops, &sbp_messages);
  }

  /* There is an edge case when switching into time-matched mode where
   * a solution may be posted after the most recent low-latency solution
   * was already transmitted. We detect this case and make sure to avoid
   * outputting anachronous solutions.
   *
   * If there was a valid solution this epoch, then `last_dgnss` will
   * have been set while making the baseline SBP messages. We check
   * this value to make sure it occurs after the most recent low latency
   * output.
   *
   * If either of the timestamps are invalid treat it as if this message
   * after a previous low latency message. This will only occur with the
   * first mesage.
   */
  chMtxLock(&last_sbp_lock);
  const bool is_after_last_low_latency =
      !gps_time_valid(&last_sbp_dgnss) ||
      !gps_time_valid(&last_sbp_low_latency) ||
      gpsdifftime(&last_sbp_dgnss, &last_sbp_low_latency) > 0.;
  chMtxUnlock(&last_sbp_lock);
  if (is_after_last_low_latency) {
    solution_send_pos_messages(&sbp_messages);
  }
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
static void handle_solution_low_latency(
    const StarlingFilterSolution *spp_solution,
    const StarlingFilterSolution *rtk_solution,
    const gps_time_t *solution_epoch_time) {
  assert(solution_epoch_time);

  /* Check if observations do not have valid time. We may have locally a
   * reasonable estimate of current GPS time, so we can round that to the
   * nearest epoch and use instead if necessary.
   */
  gps_time_t epoch_time = *solution_epoch_time;
  if (!gps_time_valid(&epoch_time)) {
    epoch_time = get_current_time();
    epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
  }

  /* Initialize the output messages. If there is an SPP solution, we first
   * apply that. Then if there is an RTK solution, overwrite the relevant
   * messages with the RTK baseline result. When there are no valid
   * solutions, we simply pass on the set of default messages. */
  u8 time_qual = get_time_quality();
  sbp_messages_t sbp_messages;
  starling_integration_sbp_messages_init(&sbp_messages, &epoch_time, time_qual);

  /* TODO: Actually get the timing quality from ME / Starling
   * the time quality could have degraded or improved AFTER this
   * solution epoch was calculated. Thus time quality isn't strictly
   * true but is good enough.  The plumbing exercise remains to remove
   * discrepancy between current time qual and this epoch's time qual */
  if (spp_solution) {
    solution_make_sbp(
        &spp_solution->result, &spp_solution->dops, &sbp_messages, time_qual);
    if (rtk_solution) {
      solution_make_baseline_sbp(&rtk_solution->result,
                                 spp_solution->result.baseline,
                                 &rtk_solution->dops,
                                 &sbp_messages);
    }
  }
  send_low_latency_messages(&epoch_time, &sbp_messages);
}

/*******************************************************************************
 * Starling Initialization
 ******************************************************************************/
static void profile_low_latency_thread(enum ProfileDirective directive) {
  static float avg_run_time_s = 0.1f;
  static float diff_run_time_s = 0.1f;
  static float avg_diff_run_time_s = 0.0f;
  static float std_run_time_s = 0.1f;
  const float smooth_factor = 0.01f;
  static u64 nap_snapshot_begin = 0;
  switch (directive) {
    case PROFILE_BEGIN:
      nap_snapshot_begin = nap_timing_count();
      break;
    case PROFILE_END: {
      u64 nap_snapshot_diff = (u64)(nap_timing_count() - nap_snapshot_begin);
      float time_snapshot_diff = (float)(RX_DT_NOMINAL * nap_snapshot_diff);
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

/******************************************************************************/
static THD_FUNCTION(initialize_and_run_starling, arg) { /* NOLINT */
  (void)arg;
  chRegSetThreadName("starling");

  /* Set time of last differential solution in the past. */
  last_sbp_dgnss = GPS_TIME_UNKNOWN;
  last_sbp_low_latency = GPS_TIME_UNKNOWN;

  /* Register a reset callback. */
  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
      SBP_MSG_RESET_FILTERS, &reset_filters_callback, &reset_filters_node);

  StarlingDebugFunctionTable debug_functions = {
      .profile_low_latency_thread = profile_low_latency_thread,
  };

  /* This runs forever. */
  starling_run(&debug_functions);

  /* Never get here. */
  log_error("Starling Engine has unexpectedly terminated.");
  assert(0);

  __builtin_unreachable();
}

/*******************************************************************************/
static void update_piksi_solution_info(const StarlingFilterSolution *soln) {
  /* Ignore non-existant or invalid solutions. */
  if (NULL == soln || !soln->result.valid) {
    return;
  }
  piksi_systime_t now_systime = PIKSI_SYSTIME_INIT;
  piksi_systime_get(&now_systime);
  /* Check the position mode and update the corresponding info. */
  const uint8_t pos_mode = soln->result.flags.position_mode;
  const uint8_t is_fixed = (pos_mode == POSITION_MODE_FIXED);
  chMtxLock(&piksi_solution_info_lock);
  switch (pos_mode) {
    case POSITION_MODE_SPP:  // fallthru
    case POSITION_MODE_SBAS:
      piksi_solution_info.last_time_spp = now_systime;
      piksi_solution_info.num_spp_signals = soln->result.num_sigs_used;
      break;
    case POSITION_MODE_DGNSS:  // fallthru
    case POSITION_MODE_FLOAT:  // fallthru
    case POSITION_MODE_FIXED:
      piksi_solution_info.last_time_rtk = now_systime;
      piksi_solution_info.was_last_rtk_fix = is_fixed;
      break;
    case POSITION_MODE_NONE:            // fallthru
    case POSITION_MODE_DEAD_RECKONING:  // fallthru
    default:
      break;
  }
  chMtxUnlock(&piksi_solution_info_lock);
}

/*******************************************************************************/
static void update_solution_info_low_latency(
    const StarlingFilterSolution *spp_solution,
    const StarlingFilterSolution *rtk_solution,
    const gps_time_t *solution_epoch_time) {
  (void)solution_epoch_time;
  update_piksi_solution_info(spp_solution);
  update_piksi_solution_info(rtk_solution);
}

/*******************************************************************************/
static void update_solution_info_time_matched(
    const StarlingFilterSolution *solution,
    const obs_core_t *obs_base,
    const pvt_engine_result_t *rover_soln,
    const double *rover_spp_ecef) {
  (void)obs_base;
  (void)rover_soln;
  (void)rover_spp_ecef;
  update_piksi_solution_info(solution);
}

/*******************************************************************************/
static void setup_solution_handlers(void) {
  /* This solution handler manages all of the SBP message transmission. */
  static SolutionHandler handler_sbp = {
      .handle_low_latency = handle_solution_low_latency,
      .handle_time_matched = handle_solution_time_matched,
  };
  starling_add_solution_handler(&handler_sbp);

  /* This one keeps stats on the solutions which are used by the LEDs. */
  static SolutionHandler handler_info = {
      .handle_low_latency = update_solution_info_low_latency,
      .handle_time_matched = update_solution_info_time_matched,
  };
  starling_add_solution_handler(&handler_info);
}

/*******************************************************************************
 * Starling Integration API
 ******************************************************************************/
void starling_calc_pvt_setup() {
  /* Setup the Starling SBP link. */
  starling_sbp_link_setup();
  assert(sbp_link);

  /* Let Starling register all of its settings over SBP. */
  starling_register_sbp_settings(sbp_link);

  /* Connect the missing external dependencies for the Starling engine. */
  external_functions_t extfns = {
      .cache_read_ephemeris = cache_read_ephemeris,
      .cache_read_iono_corr = cache_read_iono_corr,
      .track_sid_db_elevation_degrees_get = track_sid_db_elevation_degrees_get,
      .shm_navigation_unusable = shm_navigation_unusable,
      .starling_integration_simulation_enabled =
          starling_integration_simulation_enabled,
      .starling_integration_simulation_run =
          starling_integration_simulation_run,
      .disable_raim = is_raim_disabled,
  };
  starling_set_external_functions_implementation(&extfns);

  setup_solution_handlers();

  /* Start main starling thread. */
  platform_thread_create(THREAD_ID_STARLING, initialize_and_run_starling);
}
