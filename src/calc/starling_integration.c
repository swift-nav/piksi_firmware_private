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

#include <starling/integration/starling_input_bridge.h>
#include <starling/platform/starling_platform.h>
#include <starling/starling.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/memcpy_s.h>

#include "calc/calc_pvt_common.h"
#include "calc/calc_pvt_me.h"
#include "calc/starling_integration.h"
#include "calc/starling_sbp_output.h"
#include "ndb/ndb.h"
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
 * Globals
 ******************************************************************************/
bool send_heading = false;

/* TODO(kevin) what to do about this? */
bool disable_raim = false;

/*******************************************************************************
 * Locals
 ******************************************************************************/
static bool enable_glonass = true;
static bool enable_galileo = true;
static bool enable_beidou = true;

static double heading_offset = 0.0;

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
  sbp_init_utc_time(&sbp_messages->utc_time, t, sbp_time_qual);
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
    const sbp_messages_t *sbp_messages) {
  dgnss_solution_mode_t dgnss_soln_mode = starling_get_solution_mode();
  /* Work out if we need to wait for a certain period of no time matched
   * positions before we output a SBP position */
  bool wait_for_timeout = false;
  if (!(dgnss_timeout(&last_dgnss_stats.systime, dgnss_soln_mode)) &&
      STARLING_SOLN_MODE_TIME_MATCHED == dgnss_soln_mode) {
    wait_for_timeout = true;
  }

  if (!wait_for_timeout) {
    solution_send_pos_messages(sbp_messages);
    chMtxLock(&last_sbp_lock);
    last_spp.wn = sbp_messages->gps_time.wn;
    last_spp.tow = sbp_messages->gps_time.tow * 0.001;
    chMtxUnlock(&last_sbp_lock);
  }
}

static void solution_make_sbp(const pvt_engine_result_t *soln,
                              const dops_t *dops,
                              sbp_messages_t *sbp_messages,
                              u8 time_qual) {
  if (soln && soln->valid) {
    sbp_make_gps_time(&sbp_messages->gps_time, &soln->time, time_qual);
    sbp_make_utc_time(&sbp_messages->utc_time, &soln->time, time_qual);
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
      send_observations(simulation_current_num_sats(),
                        SBP_FRAMING_MAX_PAYLOAD_SIZE,
                        simulation_current_navigation_measurements(),
                        &(soln->time));
    }
  }
}

void starling_integration_simulation_run(const me_msg_obs_t *me_msg) {
  gps_time_t epoch_time = me_msg->obs_time;
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
  starling_integration_solution_send_low_latency_output(&sbp_messages);
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
  u8 time_qual = get_time_quality();
  starling_integration_sbp_messages_init(
      &sbp_messages, &obss_base->tor, time_qual);

  pvt_engine_result_t soln_copy = obss_rover->soln;

  /* TODO: Actually get the timing quality from ME / Starling
   * the time quality could have degraded or improved AFTER this
   * solution epoch was calculated. Thus time quality isn't strictly
   * true but it is good enough.  The plumbing exercise remains to remove
   * discrepancy between current time qual and this epoch's time qual */

  solution_make_sbp(&soln_copy, NULL, &sbp_messages, time_qual);

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
    solution_send_pos_messages(&sbp_messages);
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
                               const gps_time_t *solution_epoch_time) {
  assert(solution_epoch_time);

  /* Check if observations do not have valid time. We may have locally a
   * reasonable estimate of current GPS time, so we can round that to the
   * nearest epoch and use instead if necessary.
   */
  gps_time_t epoch_time = *solution_epoch_time;
  u8 time_qual = get_time_quality();
  if (!gps_time_valid(&epoch_time) && TIME_PROPAGATED <= time_qual) {
    epoch_time = get_current_time();
    epoch_time = gps_time_round_to_epoch(&epoch_time, soln_freq_setting);
  }

  /* Initialize the output messages. If there is an SPP solution, we first
   * apply that. Then if there is an RTK solution, overwrite the relevant
   * messages with the RTK baseline result. When there are no valid
   * solutions, we simply pass on the set of default messages. */
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
  starling_integration_solution_send_low_latency_output(&sbp_messages);
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

  /* The base obs can optionally enable RAIM exclusion algorithm. */
  SETTING("solution", "disable_raim", disable_raim, TYPE_BOOL);

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
  /* When Skylark or Orion are enabled, klobuchar corrections shall be disabled.
   * Note that
   * this setting is originally from piksi_system_daemon but since PFWP settings
   * module doesn't support setting_add_watch functionality, setting
   * registration was moved here and original registration in
   * piksi_system_daemon was replaced with watch trigger registration. If the
   * setting infrastructure is reworked so that watch trigger registration is
   * possible also from PFWP, the actual registration can be moved back to
   * piksi_system_daemon. */
  SETTING_NOTIFY(
      "skylark", "enable", disable_klobuchar, TYPE_BOOL, set_disable_klobuchar);
  SETTING_NOTIFY(
      "orion", "enable", disable_klobuchar, TYPE_BOOL, set_disable_klobuchar);

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

  StarlingIoFunctionTable io_functions = {
      .wait = starling_wait,
      .read_obs_rover = starling_receive_rover_obs,
      .read_obs_base = starling_receive_base_obs,
      .read_sbas_data = starling_receive_sbas_data,
      .read_ephemeris_array = starling_receive_ephemeris_array,
      .read_imu = starling_receive_imu_data,
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
