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

#include "calc_base_obs.h"
#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "calc_pvt_starling.h"
#include "main.h"
#include "manage.h"
#include "me_msg/me_msg.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "piksi_systime.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "starling_threads.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

/* Maximum CPU time the solution thread is allowed to use. */
#define SOLN_THD_CPU_MAX (0.60f)

memory_pool_t time_matched_obs_buff_pool;
mailbox_t time_matched_obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;
dgnss_filter_t dgnss_filter = FILTER_FIXED;

FilterManager *time_matched_filter_manager;
FilterManager *low_latency_filter_manager;
FilterManager *spp_filter_manager;

MUTEX_DECL(time_matched_filter_manager_lock);
MUTEX_DECL(low_latency_filter_manager_lock);
MUTEX_DECL(spp_filter_manager_lock);

MUTEX_DECL(time_matched_iono_params_lock);
bool has_time_matched_iono_params = false;
ionosphere_t time_matched_iono_params;

MUTEX_DECL(last_sbp_lock);
gps_time_t last_dgnss;
gps_time_t last_spp;
gps_time_t last_time_matched_rover_obs_post;

double starling_frequency;
u32 max_age_of_differential = 30;

bool disable_raim = false;
bool send_heading = false;

double heading_offset = 0.0;

bool disable_klobuchar = false;

bool enable_glonass = true;

float glonass_downweight_factor = 4;

u8 current_base_sender_id;

static soln_pvt_stats_t last_pvt_stats = {.systime = PIKSI_SYSTIME_INIT,
                                          .signals_used = 0};
static soln_dgnss_stats_t last_dgnss_stats = {.systime = PIKSI_SYSTIME_INIT,
                                              .mode = 0};
sbas_system_t current_sbas_system = SBAS_UNKNOWN;

void post_observations(u8 n,
                       const navigation_measurement_t m[],
                       const gps_time_t *t,
                       const pvt_engine_result_t *soln) {
  /* TODO: use a buffer from the pool from the start instead of
   * allocating nav_meas_tdcp as well. Downside, if we don't end up
   * pushing the message into the mailbox then we just wasted an
   * observation from the mailbox for no good reason. */

  obss_t *obs = chPoolAlloc(&time_matched_obs_buff_pool);
  msg_t ret;
  if (obs == NULL) {
    /* Pool is empty, grab a buffer from the mailbox instead, i.e.
     * overwrite the oldest item in the queue. */
    ret = chMBFetch(&time_matched_obs_mailbox, (msg_t *)&obs, TIME_IMMEDIATE);
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

    ret = chMBPost(&time_matched_obs_mailbox, (msg_t)obs, TIME_IMMEDIATE);
    if (ret != MSG_OK) {
      /* We could grab another item from the mailbox, discard it and then
       * post our obs again but if the size of the mailbox and the pool
       * are equal then we should have already handled the case where the
       * mailbox is full when we handled the case that the pool was full.
       * */
      log_error("Mailbox should have space!");
      chPoolFree(&time_matched_obs_buff_pool, obs);
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
void solution_send_pos_messages(
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

void solution_send_low_latency_output(
    u8 base_sender_id,
    const sbp_messages_t *sbp_messages,
    u8 n_meas,
    const navigation_measurement_t nav_meas[]) {
  // Work out if we need to wait for a certain period of no time matched
  // positions before we output a SBP position
  bool wait_for_timeout = false;
  if (!(dgnss_timeout(&last_dgnss_stats.systime, dgnss_soln_mode)) &&
      SOLN_MODE_TIME_MATCHED == dgnss_soln_mode) {
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

double calc_heading(const double b_ned[3]) {
  double heading = atan2(b_ned[1], b_ned[0]);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading * R2D;
}

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

  chMtxLock(&last_sbp_lock);
  last_dgnss.wn = result->time.wn;
  last_dgnss.tow = result->time.tow;
  chMtxUnlock(&last_sbp_lock);

  /* Update stats */
  piksi_systime_get(&last_dgnss_stats.systime);
  last_dgnss_stats.mode =
      (result->flags == FIXED_POSITION) ? FILTER_FIXED : FILTER_FLOAT;
}

void solution_simulation(sbp_messages_t *sbp_messages) {
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

static bool enable_fix_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }

  bool enable_fix = value == 0 ? false : true;
  chMtxLock(&time_matched_filter_manager_lock);
  set_pvt_engine_enable_fix_mode(time_matched_filter_manager, enable_fix);
  chMtxUnlock(&time_matched_filter_manager_lock);
  chMtxLock(&low_latency_filter_manager_lock);
  set_pvt_engine_enable_fix_mode(low_latency_filter_manager, enable_fix);
  chMtxUnlock(&low_latency_filter_manager_lock);
  *(dgnss_filter_t *)s->addr = value;
  return ret;
}

static bool set_max_age(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }

  chMtxLock(&low_latency_filter_manager_lock);
  set_max_correction_age(low_latency_filter_manager, value);
  chMtxUnlock(&low_latency_filter_manager_lock);
  chMtxLock(&time_matched_filter_manager_lock);
  set_max_correction_age(time_matched_filter_manager, value);
  chMtxUnlock(&time_matched_filter_manager_lock);
  *(int *)s->addr = value;
  return ret;
}

void init_filters(void) {
  chMtxLock(&time_matched_filter_manager_lock);
  time_matched_filter_manager = create_filter_manager_rtk();
  chMtxUnlock(&time_matched_filter_manager_lock);

  chMtxLock(&low_latency_filter_manager_lock);
  low_latency_filter_manager = create_filter_manager_rtk();
  chMtxUnlock(&low_latency_filter_manager_lock);

  static const char *const dgnss_filter_enum[] = {"Float", "Fixed", NULL};
  static struct setting_type dgnss_filter_setting;
  int TYPE_GNSS_FILTER =
      settings_type_register_enum(dgnss_filter_enum, &dgnss_filter_setting);

  SETTING_NOTIFY("solution",
                 "dgnss_filter",
                 dgnss_filter,
                 TYPE_GNSS_FILTER,
                 enable_fix_mode);
  SETTING_NOTIFY("solution",
                 "correction_age_max",
                 max_age_of_differential,
                 TYPE_INT,
                 set_max_age);
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

void platform_initialize_settings() {
  static const char *const dgnss_soln_mode_enum[] = {
      "Low Latency", "Time Matched", "No DGNSS", NULL};
  static struct setting_type dgnss_soln_mode_setting;
  int TYPE_GNSS_SOLN_MODE = settings_type_register_enum(
      dgnss_soln_mode_enum, &dgnss_soln_mode_setting);
  SETTING(
      "solution", "dgnss_solution_mode", dgnss_soln_mode, TYPE_GNSS_SOLN_MODE);

  SETTING("solution", "disable_raim", disable_raim, TYPE_BOOL);
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
}

void platform_initialize_memory_pools() {
  static msg_t time_matched_obs_mailbox_buff[STARLING_OBS_N_BUFF];
  chMBObjectInit(&time_matched_obs_mailbox,
                 time_matched_obs_mailbox_buff,
                 STARLING_OBS_N_BUFF);
  chPoolObjectInit(&time_matched_obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[STARLING_OBS_N_BUFF] _CCM;
  chPoolLoadArray(&time_matched_obs_buff_pool, obs_buff, STARLING_OBS_N_BUFF);
}


