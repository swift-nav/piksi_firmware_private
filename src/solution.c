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
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <libswiftnav/cnav_msg.h>
#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/pvt.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/troposphere.h>
#include <libswiftnav/sid_set.h>

#define memory_pool_t MemoryPool
#include <ch.h>
#undef memory_pool_t

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
#include "system_monitor.h"
#include "main.h"
#include "cnav_msg_storage.h"
#include "ndb.h"
#include "shm.h"

/* Maximum CPU time the solution thread is allowed to use. */
#define SOLN_THD_CPU_MAX (0.60f)

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

/** Mandatory flags filter for measurements */
#define MANAGE_TRACK_FLAGS_FILTER (MANAGE_TRACK_FLAG_ACTIVE | \
                                   MANAGE_TRACK_FLAG_NO_ERROR | \
                                   MANAGE_TRACK_FLAG_CONFIRMED | \
                                   MANAGE_TRACK_FLAG_CN0_SHORT | \
                                   MANAGE_TRACK_FLAG_ELEVATION | \
                                   MANAGE_TRACK_FLAG_HAS_EPHE | \
                                   MANAGE_TRACK_FLAG_HEALTHY | \
                                   MANAGE_TRACK_FLAG_NAV_SUITABLE | \
                                   MANAGE_TRACK_FLAG_TOW )
/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

MemoryPool obs_buff_pool;
mailbox_t obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;
dgnss_filter_t dgnss_filter = FILTER_FLOAT;

static FilterManager *time_matched_filter_manager;
static FilterManager *low_latency_filter_manager;

MUTEX_DECL(time_matched_filter_manager_lock);
MUTEX_DECL(low_latency_filter_manager_lock);

MUTEX_DECL(time_matched_iono_params_lock);
bool has_time_matched_iono_params = false;
static ionosphere_t time_matched_iono_params;

MUTEX_DECL(last_sbp_lock);
gps_time_t last_dgnss;
gps_time_t last_spp;

double soln_freq = 10.0;
u32 max_age_of_differential = 30;
u32 obs_output_divisor = 10;

double known_baseline[3] = {0, 0, 0};
s16 msg_obs_max_size = 102;

bool disable_raim = false;
bool send_heading = false;

double heading_offset = 0.0;

bool disable_klobuchar = false;

static u8 old_base_sender_id = 0;
static u8 low_latency_base_sender_id = 0;

static soln_stats_t last_stats = { .signals_tracked = 0, .signals_useable = 0 };
static soln_pvt_stats_t last_pvt_stats = { .systime = -1, .signals_used = 0 };
static soln_dgnss_stats_t last_dgnss_stats = { .systime = -1, .mode = 0 };

void reset_rtk_filter(void) {
  chMtxLock(&time_matched_filter_manager_lock);
  filter_manager_init(time_matched_filter_manager);
  chMtxUnlock(&time_matched_filter_manager_lock);
}

/** Determine if we have had a DGNSS timeout.
 *
 * \param _last_dgnss. Last time of DGNSS solution
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
bool dgnss_timeout(systime_t _last_dgnss, dgnss_solution_mode_t _dgnss_soln_mode) {

  // No timeout needed in low latency mode;
  if (_dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
    return false;
  }

  // Need to compare timeout threshold in MS to system time elapsed (in system ticks)
  return (chVTTimeElapsedSinceX(_last_dgnss) > MS2ST(DGNSS_TIMEOUT_MS) );
}

/** Determine if we have had a SPP timeout.
 *
 * \param _last_spp. Last time of SPP solution
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
bool spp_timeout(const gps_time_t *_last_spp, const gps_time_t *_last_dgnss, dgnss_solution_mode_t _dgnss_soln_mode) {

  // No timeout needed in low latency mode;
  if (_dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
    return false;
  }
  chMtxLock(&last_sbp_lock);
  double time_diff = gpsdifftime(_last_dgnss,_last_spp);
  chMtxUnlock(&last_sbp_lock);

  // Need to compare timeout threshold in MS to system time elapsed (in system ticks)
  return (time_diff > 0.0);
}

void solution_make_sbp(const gnss_solution *soln, dops_t *dops, bool clock_jump, sbp_messages_t *sbp_messages) {
  if (soln && soln->valid) {
    /* Send GPS_TIME message first. */
    sbp_make_gps_time(&sbp_messages->gps_time, &soln->time, SPP_POSITION);

    u8 utc_flags = SPP_POSITION;
    utc_params_t utc_params;
    /* try to read UTC parameters from NDB */
    if (NDB_ERR_NONE == ndb_utc_params_read(&utc_params)) {
      /* TODO: flag NVM_UTC when the params come from NV */
      utc_flags |= (DECODED_UTC << 3);
    } else {
      utc_flags |= (DEFAULT_UTC << 3);
    }

    sbp_make_utc_time(&sbp_messages->utc_time, &soln->time, utc_flags, &utc_params);

    /* Extract full covariance matrix from upper triangular in soln->err_cov */
    double full_covariance[9];
    extract_covariance(full_covariance, soln);

    /* Compute the accuracy figures from the covariance matrix */
    double accuracy, h_accuracy, v_accuracy;
    covariance_to_accuracy(full_covariance, soln->pos_ecef,
                           &accuracy, &h_accuracy, &v_accuracy);

    /* Estimate ballpark velocity accuracy from the position accuracy.
     * Since velocity uses the same system matrix as SPP position solution, the
     * accuracy estimate is just a scaled version of that.
     * TODO: implement proper computation of vel_err_cov matrix in LSNP */

    double vel_accuracy_multiplier = sqrt(DOPPLER_CN0_COEFFICIENT / CODE_CN0_COEFFICIENT)
                                     * code_to_lambda(CODE_GPS_L1CA);
    double vel_accuracy = vel_accuracy_multiplier * accuracy;
    double vel_h_accuracy = vel_accuracy_multiplier * h_accuracy;
    double vel_v_accuracy = vel_accuracy_multiplier * v_accuracy;

    const gps_time_t soln_time = soln->time;

    /* Position in LLH. */
    sbp_make_pos_llh_vect(&sbp_messages->pos_llh, soln->pos_llh, h_accuracy, v_accuracy,
                          &soln_time, soln->n_sats_used, SPP_POSITION);

    /* Position in ECEF. */
    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef, soln->pos_ecef, accuracy,
                          &soln_time, soln->n_sats_used, SPP_POSITION);

    /* Velocity in NED. */
    /* Do not send if there has been a clock jump. Velocity may be unreliable.*/
    if (!clock_jump && soln->velocity_valid) {
      sbp_make_vel_ned(&sbp_messages->vel_ned,
                       soln,
                       vel_h_accuracy, vel_v_accuracy,
                       SPP_POSITION); /* TODO replace with a Measured Doppler Flag #define */

      /* Velocity in ECEF. */
      sbp_make_vel_ecef(&sbp_messages->vel_ecef,
                        soln,
                        vel_accuracy,
                        SPP_POSITION); /* TODO replace with a Measured Doppler Flag #define */
    }

    /* DOP message can be sent even if solution fails to compute */
    if (dops) {
      sbp_make_dops(&sbp_messages->sbp_dops,dops,sbp_messages->pos_llh.tow, SPP_POSITION);
    }

    /* Update stats */
    last_pvt_stats.systime = chVTGetSystemTime();
    last_pvt_stats.signals_used = soln->n_sigs_used;
  } else {
    gps_time_t time_guess = get_current_time();
    sbp_make_gps_time(&sbp_messages->gps_time, &time_guess, 0);
  }
}

/** Extract the full covariance matrix from soln struct */
void extract_covariance(double full_covariance[9], const gnss_solution *soln) {

  assert(soln != NULL);
  assert(full_covariance != NULL);

/* soln->cov_err has the covariance in upper triangle covariance form, so
 * copy from
 *
 *    0  1  2       0  1  2
 *    _  3  4   to  3  4  5
 *    _  _  5       6  7  8  */

  full_covariance[0] = soln->err_cov[0];
  full_covariance[1] = soln->err_cov[1];
  full_covariance[2] = soln->err_cov[2];
  full_covariance[3] = soln->err_cov[1];
  full_covariance[4] = soln->err_cov[3];
  full_covariance[5] = soln->err_cov[4];
  full_covariance[6] = soln->err_cov[2];
  full_covariance[7] = soln->err_cov[4];
  full_covariance[8] = soln->err_cov[5];
}

/**
 *
 * @param propagation_time time sdiffs were propagated for
 * @param sender_id sender id of base obs
 * @param n_used number of measuremendt in nav_meas
 * @param nav_meas observations
 * @param gps_time sbp gps time message
 * @param pos_llh sbp position llh message
 * @param pos_ecef sbp ecef position message
 * @param vel_ned sbp ned velocity message
 * @param vel_ecef sbp ecef velcoity message
 * @param sbp_dops sbp DOP message
 * @param baseline_ned sbp ned baseline message
 * @param baseline_ecef sbp ecef baseline message
 * @param baseline_heading sbp baseline heading message
 */
static void solution_send_pos_messages(double propagation_time, u8 sender_id,
	                                   const sbp_messages_t *sbp_messages) {

  if (sbp_messages) {
    sbp_send_msg(SBP_MSG_GPS_TIME, sizeof(sbp_messages->gps_time), (u8 * ) &sbp_messages->gps_time);
    sbp_send_msg(SBP_MSG_UTC_TIME, sizeof(sbp_messages->utc_time), (u8 * ) &sbp_messages->utc_time);
    sbp_send_msg(SBP_MSG_POS_LLH, sizeof(sbp_messages->pos_llh), (u8 * ) &sbp_messages->pos_llh);
    sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(sbp_messages->pos_ecef), (u8 * ) &sbp_messages->pos_ecef);
    sbp_send_msg(SBP_MSG_VEL_NED, sizeof(sbp_messages->vel_ned), (u8 * ) &sbp_messages->vel_ned);
    sbp_send_msg(SBP_MSG_VEL_ECEF, sizeof(sbp_messages->vel_ecef), (u8 * ) &sbp_messages->vel_ecef);
    sbp_send_msg(SBP_MSG_DOPS, sizeof(sbp_messages->sbp_dops), (u8 *) &sbp_messages->sbp_dops);

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_ECEF, sizeof(sbp_messages->baseline_ecef), (u8 * ) &sbp_messages->baseline_ecef);
    }

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_NED, sizeof(sbp_messages->baseline_ned), (u8 * ) &sbp_messages->baseline_ned);
    }

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_AGE_CORRECTIONS, sizeof(sbp_messages->age_corrections), (u8 * ) &sbp_messages->age_corrections);
    }

    if (dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_DGNSS_STATUS, sizeof(sbp_messages->dgnss_status), (u8 * ) &sbp_messages->dgnss_status);
    }

    if (send_heading && dgnss_soln_mode != SOLN_MODE_NO_DGNSS) {
      sbp_send_msg(SBP_MSG_BASELINE_HEADING, sizeof(sbp_messages->baseline_heading),
                   (u8 * ) &sbp_messages->baseline_heading);
    }
  }

  utc_params_t utc_params;
  /* read UTC parameters from NDB if they exist*/
  ndb_utc_params_read(&utc_params);

  /* Send NMEA alongside the sbp */
  nmea_send_msgs(&sbp_messages->pos_llh, &sbp_messages->vel_ned,
                 &sbp_messages->sbp_dops, &sbp_messages->gps_time,
                 propagation_time, sender_id, &utc_params,
                 &sbp_messages->baseline_heading);

}

static void solution_send_low_latency_output(double propagation_time, u8 sender_id,
                                      		 const sbp_messages_t *sbp_messages) {
  // Work out if we need to wait for a certain period of no time matched positions before we output a SBP position
  bool wait_for_timeout = false;
  if (!(dgnss_timeout(last_dgnss_stats.systime, dgnss_soln_mode)) && dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    wait_for_timeout = true;
  }

  if (!wait_for_timeout) {
    solution_send_pos_messages(propagation_time, sender_id, sbp_messages);
    chMtxLock(&last_sbp_lock);
    last_spp.wn = sbp_messages->gps_time.wn;
    last_spp.tow = sbp_messages->gps_time.tow * 0.001;
    chMtxUnlock(&last_sbp_lock);

  }
}

double calc_heading(const double b_ned[3])
{
  double heading = atan2(b_ned[1], b_ned[0]);
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  return heading * R2D;
}

/** Creates and sends RTK solution.
 * If the base station position is known,
 * send the NMEA and SBP psuedo absolute msgs.
 *
 * If operating in simulation mode, it depends upon the simulation mode enabled
 * and the simulation base_ecef position (both available in the global struct
 * sim_settings and accessed via wrappers prototyped in simulator.h)
 *
 * \param t pointer to gps time struct representing gps time for solution
 * \param n_sats u8 representig the number of satellites
 * \param b_ecef size 3 vector of doubles representing ECEF position (meters)
 * \param ref_ecef size 3 vector of doubles representing reference position
 * for conversion from ECEF to local NED coordinates (meters)
 * \param flags u8 RTK solution flags. 1 if float, 0 if fixed
 */
void solution_make_baseline_sbp(const gps_time_t *t, u8 n_sats, double b_ecef[3],
                                double covariance_ecef[9], double ref_ecef[3],
                                bool has_known_base_pos_ecef, double known_base_pos[3],
                                u8 flags, dops_t *dops, double propagation_time,
                                sbp_messages_t *sbp_messages)
{
  double b_ned[3];
  wgsecef2ned(b_ecef, ref_ecef, b_ned);

  double accuracy, h_accuracy, v_accuracy;
  covariance_to_accuracy(covariance_ecef, ref_ecef, &accuracy, &h_accuracy, &v_accuracy);

  sbp_make_baseline_ecef(&sbp_messages->baseline_ecef, t, n_sats, b_ecef, accuracy, flags);

  sbp_make_baseline_ned(&sbp_messages->baseline_ned, t, n_sats, b_ned, h_accuracy, v_accuracy, flags);

  sbp_make_age_corrections(&sbp_messages->age_corrections, t, propagation_time);

  sbp_make_dgnss_status(&sbp_messages->dgnss_status, n_sats, propagation_time, flags);

  if(flags == FIXED_POSITION && dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    double heading = calc_heading(b_ned);
    sbp_make_heading(&sbp_messages->baseline_heading, t, heading + heading_offset, n_sats, flags);
  }

  if (has_known_base_pos_ecef || (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK))) {
    double pseudo_absolute_ecef[3];
    double pseudo_absolute_llh[3];

    vector_add(3, known_base_pos, b_ecef, pseudo_absolute_ecef);
    wgsecef2llh(pseudo_absolute_ecef, pseudo_absolute_llh);

    /* now send pseudo absolute sbp message */
    sbp_make_pos_llh_vect(&sbp_messages->pos_llh, pseudo_absolute_llh, h_accuracy, v_accuracy, t, n_sats, flags);
    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef, pseudo_absolute_ecef, accuracy, t, n_sats, flags);

  }
  sbp_make_dops(&sbp_messages->sbp_dops, dops, sbp_messages->pos_llh.tow, flags);

  if(t) {
    chMtxLock(&last_sbp_lock);
    last_dgnss.wn = t->wn;
    last_dgnss.tow = t->tow;
    chMtxUnlock(&last_sbp_lock);
  }

  /* Update stats */
  last_dgnss_stats.systime = chVTGetSystemTime();
  last_dgnss_stats.mode = (flags == FIXED_POSITION) ? FILTER_FIXED : FILTER_FLOAT;
}

static void output_baseline(u8 num_sdiffs, const sdiff_t *sdiffs, const gps_time_t *t,
                            dops_t *dops, double diff_time, double rover_pos[3],
                            bool has_known_base_pos_ecef, double known_base_pos[3],
                            sbp_messages_t *sbp_messages) {
  double baseline[3];
  double covariance[9];
  u8 num_sats_used = 0;
  u8 num_sigs_used = 0;
  u8 flags = 0;
  s8 ret = -1;
  bool send_baseline = false;

  if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    /* In time matched mode filter is already updated so no need to update
       filter again. Just get the baseline. */
    ret = get_baseline(time_matched_filter_manager, baseline, covariance,
                       &num_sats_used, &num_sigs_used, &flags);
    if (ret < 0) {
      log_warn("output_baseline: Time matched baseline calculation failed");
    } else if (ret == 0) {
      *dops = filter_manager_get_dop_values(time_matched_filter_manager);
      send_baseline = true;
    }
  } else if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
    /* Need to update filter with propagated obs before we can get the baseline. */
    chMtxLock(&low_latency_filter_manager_lock);
    if (filter_manager_is_initialized(low_latency_filter_manager)) {
      ret = filter_manager_update(low_latency_filter_manager, t,
                                  sdiffs, num_sdiffs, rover_pos,
                                  has_known_base_pos_ecef ? known_base_pos : NULL,
                                  diff_time);
      if (ret == 0) {
        ret = get_baseline(low_latency_filter_manager, baseline, covariance,
                           &num_sats_used, &num_sigs_used, &flags);
        if (ret < 0) {
          log_warn("output_baseline: Low latency baseline calculation failed");
        } else if (ret == 0) {
          *dops = filter_manager_get_dop_values(low_latency_filter_manager);
          send_baseline = true;
        }
      } else {
        log_info("Skipping low latency filter update this epoch.");
      }
    } else {
      log_info("Low latency filter not initialized.");
    }
    chMtxUnlock(&low_latency_filter_manager_lock);
  }

  if (send_baseline) {
    solution_make_baseline_sbp(t, num_sats_used, baseline, covariance,
                               rover_pos, has_known_base_pos_ecef, known_base_pos,
                               flags, dops, diff_time, sbp_messages);
  }
}

static void send_observations(u8 n, const navigation_measurement_t m[],
                              const gps_time_t *t)
{
  static u8 buff[256];

  /* Upper limit set by SBP framing size, preventing underflow */
  u16 msg_payload_size = MAX(
      MIN((u16)MAX(msg_obs_max_size, 0), SBP_FRAMING_MAX_PAYLOAD_SIZE),
      sizeof(observation_header_t)
    ) - sizeof(observation_header_t);

  /* Lower limit set by sending at least 1 observation */
  msg_payload_size = MAX(msg_payload_size, sizeof(packed_obs_content_t));

  /* Round down the number of observations per message */
  u16 obs_in_msg = msg_payload_size / sizeof(packed_obs_content_t);

  /* Round up the number of messages */
  u16 total = MIN((n + obs_in_msg - 1) / obs_in_msg,
                  MSG_OBS_HEADER_MAX_SIZE);

  u8 obs_i = 0;
  for (u8 count = 0; count < total && obs_i < n; count++) {
    u8 curr_n = MIN(n - obs_i, obs_in_msg);
    pack_obs_header(t, total, count, (observation_header_t*) buff);
    packed_obs_content_t *obs = (packed_obs_content_t *)&buff[sizeof(observation_header_t)];

    for (u8 i = 0; i < obs_in_msg && obs_i < n; i++) {
      if (pack_obs_content(m[obs_i].raw_pseudorange,
                           m[obs_i].raw_carrier_phase,
                           m[obs_i].raw_measured_doppler,
                           m[obs_i].cn0,
                           m[obs_i].lock_time,
                           m[obs_i].flags,
                           m[obs_i].sid,
                           &obs[i]) >= 0) {
        /* Packed. */
        obs_i++;
      }
    }

    sbp_send_msg(SBP_MSG_OBS,
          sizeof(observation_header_t) + curr_n * sizeof(packed_obs_content_t),
          buff);
  }
}

static void post_observations(u8 n, const navigation_measurement_t m[],
                              const gps_time_t *t, const gnss_solution *soln )
{
  /* TODO: use a buffer from the pool from the start instead of
   * allocating nav_meas_tdcp as well. Downside, if we don't end up
   * pushing the message into the mailbox then we just wasted an
   * observation from the mailbox for no good reason. */

  obss_t *obs = chPoolAlloc(&obs_buff_pool);
  msg_t ret;
  if (obs == NULL) {
    /* Pool is empty, grab a buffer from the mailbox instead, i.e.
     * overwrite the oldest item in the queue. */
    ret = chMBFetch(&obs_mailbox, (msg_t *)&obs, TIME_IMMEDIATE);
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
    if (soln->valid > 0) {
      obs->pos_ecef[0] = soln->pos_ecef[0];
      obs->pos_ecef[1] = soln->pos_ecef[1];
      obs->pos_ecef[2] = soln->pos_ecef[2];
      obs->has_pos = true;
    } else {
      obs->has_pos = false;
    }

    if (soln){
      obs->soln = *soln;
    } else {
      obs->soln.valid = 0;
      obs->soln.velocity_valid = 0;
    }

    ret = chMBPost(&obs_mailbox, (msg_t)obs, TIME_IMMEDIATE);
    if (ret != MSG_OK) {
      /* We could grab another item from the mailbox, discard it and then
       * post our obs again but if the size of the mailbox and the pool
       * are equal then we should have already handled the case where the
       * mailbox is full when we handled the case that the pool was full.
       * */
      log_error("Mailbox should have space!");
      chPoolFree(&obs_buff_pool, obs);
    }
  }
}

static void solution_simulation(sbp_messages_t *sbp_messages)
{
  simulation_step();

  /* TODO: The simulator's handling of time is a bit crazy. This is a hack
   * for now but the simulator should be refactored so that it can give the
   * exact correct solution time output without this nonsense. */
  gnss_solution *soln = simulation_current_gnss_solution();

  if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
    solution_make_sbp(soln, simulation_current_dops_solution(), FALSE, sbp_messages);
  }

  if (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK)) {

    u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK) ? FIXED_POSITION : FLOAT_POSITION;

    solution_make_baseline_sbp(&(soln->time), simulation_current_num_sats(), simulation_current_baseline_ecef(),
                               simulation_current_covariance_ecef(), simulation_ref_ecef(),
                               true, simulation_ref_ecef(), flags,
                               simulation_current_dops_solution(), 0.0, sbp_messages);

    double t_check = soln->time.tow * (soln_freq / obs_output_divisor);
    if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
      send_observations(simulation_current_num_sats(),
          simulation_current_navigation_measurements(), &(soln->time));
    }
  }
}

/** Update the satellite azimuth & elevation database with current angles
 * \param rcv_pos Approximate receiver position
 * \param t Approximate time
 */
static void update_sat_azel(const double rcv_pos[3], const gps_time_t t)
{
  ephemeris_t ephemeris;
  double sat_pos[3];
  double sat_vel[3];
  double clock_err;
  double clock_rate_err;
  u64 nap_count = gpstime2napcount(&t);

  /* compute elevation for any valid ephemeris we can pull from NDB */
  for (u16 idx = 0; idx < PLATFORM_SIGNAL_COUNT; idx++) {
    gnss_signal_t sid = sid_from_global_index(idx);
    ndb_op_code_t res = ndb_ephemeris_read(sid, &ephemeris);
    /* compute elevation also from unconfirmed ephemeris candidates */
    if (NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res ) {
      if (ephemeris_valid(&ephemeris, &t)
          && calc_sat_state(&ephemeris, &t,
                       sat_pos, sat_vel, &clock_err, &clock_rate_err) >= 0) {

        double az, el;
        wgsecef2azel(sat_pos, rcv_pos, &az, &el);
        /* update the elevation with the timestamp of the used position */
        sv_azel_degrees_set(sid, (float)az * R2D, (float)el * R2D, nap_count);
        log_debug_sid(sid, "Updated elevation %.1f", el * R2D);
      }
    }
    /* TODO: use almanac if there is no valid ephemeris */
  }
}

/** Sleep until the next solution deadline.
 *
 * \param deadline    Pointer to the current deadline, updated by this function.
 * \param interval    Interval by which the deadline should be advanced.
 */
static void sol_thd_sleep(systime_t *deadline, systime_t interval)
{
  *deadline += interval;

  chSysLock();
  while (1) {
    /* Sleep for at least (1-SOLN_THD_CPU_MAX) * interval ticks so that
     * execution time is limited to SOLN_THD_CPU_MAX. */
    systime_t systime = chVTGetSystemTimeX();
    systime_t delta = *deadline - systime;
    systime_t sleep_min = (systime_t)ceilf((1.0f-SOLN_THD_CPU_MAX) * interval);
    if ((systime_t)(delta - sleep_min) <= ((systime_t)-1) / 2) {
      chThdSleepS(delta);
      break;
    } else {
      chSysUnlock();
      if (delta <= ((systime_t)-1) / 2) {
        /* Deadline is in the future. Skipping due to high CPU usage. */
        log_warn("Solution thread skipping deadline, "
                 "time = %lu, deadline = %lu", systime, *deadline);
      } else {
        /* Deadline is in the past. */
        log_warn("Solution thread missed deadline, "
                 "time = %lu, deadline = %lu", systime, *deadline);
      }
      *deadline += interval;
      chSysLock();
    }
  }
  chSysUnlock();
}

/**
 * The method excludes measurements that might decrease accuracy.
 *
 * \param[in]     n_ready Number of available measurements.
 * \param[in,out] meas    Measurements data vector.
 * \param[in,out] ephe    Ephemeris array
 *
 * \return Number of available measurements.
 */
static u8 filter_out_measurements(u8 n_ready, channel_measurement_t meas[],
                                  ephemeris_t ephe[])
{
  static const chan_meas_flags_t required_flags =
    /* Any phase accuracy, high code accuracy */
    CHAN_MEAS_FLAG_CODE_VALID | CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID;

  /* Note: do not use measurements that do not have valid Doppler */

  for (u8 i = 0; i < n_ready; ) {
    if (required_flags != (meas[i].flags & required_flags)) {
      /* This measurement can't be used */
      meas[i] = meas[n_ready - 1];
      ephe[i] = ephe[n_ready - 1];
      --n_ready;
    } else {
      ++i;
    }
  }

  return n_ready;
}

/**
 * Collects channel measurements, ephemerides and auxiliary data.
 *
 * \param[in]  rec_tc    Timestamp [samples].
 * \param[out] meas      Destination measurement array.
 * \param[out] in_view   Destination in_view array.
 * \param[out] ephe      Destination ephemeris array.
 * \param[out] pn_ready  Destination for measurement array size.
 * \param[out] pn_ready  Destination for in-view array size.
 * \param[out] pn_total  Destination for total active trackers count.
 *
 * \return None
 */
static void collect_measurements(u64 rec_tc,
                                 channel_measurement_t meas[MAX_CHANNELS],
                                 channel_measurement_t in_view[MAX_CHANNELS],
                                 ephemeris_t ephe[MAX_CHANNELS],
                                 u8 *pn_ready,
                                 u8 *pn_inview,
                                 u8 *pn_total)
{
  u8 n_collected = 0;
  u8 n_inview = 0;
  u8 n_active = 0;

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    manage_track_flags_t flags      = 0; /* Channel flags accumulator */
    /* Load measurements from the tracking channel and ephemeris from NDB */
    flags = get_tracking_channel_meas(i, rec_tc,
                                      &meas[n_collected],
                                      &ephe[n_collected]);

    if (0 != (flags & MANAGE_TRACK_FLAG_ACTIVE) &&
        0 != (flags & MANAGE_TRACK_FLAG_CONFIRMED) &&
        0 != (flags & MANAGE_TRACK_FLAG_NO_ERROR))
    {
      /* Tracking channel is active */
      n_active++;

      if (0 == (flags & MANAGE_TRACK_FLAG_XCORR_SUSPECT)) {
        /* Tracking channel is not XCORR suspect so it's an actual SV in view */
        in_view[n_inview++] = meas[n_collected];
      }

      if (0 != (flags & MANAGE_TRACK_FLAG_HEALTHY) &&
          0 != (flags & MANAGE_TRACK_FLAG_NAV_SUITABLE) &&
          0 != (flags & MANAGE_TRACK_FLAG_ELEVATION) &&
          0 != (flags & MANAGE_TRACK_FLAG_TOW) &&
          0 != (flags & MANAGE_TRACK_FLAG_HAS_EPHE) &&
          0 != (flags & MANAGE_TRACK_FLAG_CN0_SHORT) &&
          0 != meas[n_collected].flags) {
        /* Tracking channel is suitable for solution calculation */
        n_collected++;
      }
    }
  }

  *pn_ready = n_collected;
  *pn_inview = n_inview;
  *pn_total = n_active;
}

void sbp_messages_init(sbp_messages_t *sbp_messages){
  sbp_init_gps_time(&sbp_messages->gps_time);
  sbp_init_utc_time(&sbp_messages->utc_time);
  sbp_init_pos_llh(&sbp_messages->pos_llh);
  sbp_init_pos_ecef(&sbp_messages->pos_ecef);
  sbp_init_vel_ned(&sbp_messages->vel_ned);
  sbp_init_vel_ecef(&sbp_messages->vel_ecef);
  sbp_init_sbp_dops(&sbp_messages->sbp_dops);
  sbp_init_age_corrections(&sbp_messages->age_corrections);
  sbp_init_dgnss_status(&sbp_messages->dgnss_status);
  sbp_init_baseline_ecef(&sbp_messages->baseline_ecef);
  sbp_init_baseline_ned(&sbp_messages->baseline_ned);
  sbp_init_baseline_heading(&sbp_messages->baseline_heading);
}

bool gate_covariance(gnss_solution *soln) {
  assert(soln != NULL);
  double full_covariance[9];
  extract_covariance(full_covariance, soln);

  double accuracy, h_accuracy, v_accuracy;
  covariance_to_accuracy(full_covariance, soln->pos_ecef,
                         &accuracy, &h_accuracy, &v_accuracy);
  if (accuracy > 100.0) {
    log_warn("SPP Position suppressed due to position confidence of %f exceeding 100.0m", accuracy);
    return true;
  }

  return false;
}

static THD_WORKING_AREA(wa_solution_thread, 5000000);
static void solution_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("solution");

  systime_t deadline = chVTGetSystemTime();

  sbp_messages_t sbp_messages;

  bool clock_jump = FALSE;
  last_good_fix_t lgf;

  ndb_lgf_read(&lgf);

  while (TRUE) {

    sol_thd_sleep(&deadline, CH_CFG_ST_FREQUENCY/soln_freq);
    watchdog_notify(WD_NOTIFY_SOLUTION);

    // Init the messages we want to send
    sbp_messages_init(&sbp_messages);

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation(&sbp_messages);
    }

    // Take the current nap count
    u64 current_tc = nap_timing_count();
    u64 rec_tc = current_tc;

    // Work out the expected receiver time in GPS time frame for the current nap count
    gps_time_t rec_time = napcount2rcvtime(rec_tc);
    gps_time_t expected_time;

    // If we've previously had a solution, we can work out our expected obs time
    if (time_quality == TIME_FINE) {
      // Work out the time of the current nap count
      expected_time = napcount2gpstime(rec_tc);

      // Round this time to the nearest GPS solution time
      expected_time.tow = round(expected_time.tow * soln_freq)
                                 / soln_freq;
      normalize_gps_time(&expected_time);

      // This time, taken back to nap count, is the nap count we want the observations at
      rec_tc = (u64)(round(gpstime2napcount(&expected_time)));
    }
    // The difference between the current nap count and the nap count we want the observations at
    // is the amount we want to adjust our deadline by at the end of the solution
    double delta_tc = -((double)current_tc - (double)rec_tc);

    // Get the expected nap count in receiver time (gps time frame)
    rec_time = napcount2rcvtime(rec_tc);

    if (time_quality >= TIME_COARSE
        && lgf.position_solution.valid
        && lgf.position_quality >= POSITION_GUESS) {
      /* Update the satellite elevation angles once per second */
      DO_EVERY((u32)soln_freq,
               update_sat_azel(lgf.position_solution.pos_ecef,
                               lgf.position_solution.time));
    }

    u8 n_collected = 0;
    u8 n_inview = 0;
    u8 n_total = 0;
    channel_measurement_t meas[MAX_CHANNELS];
    channel_measurement_t in_view[MAX_CHANNELS];
    static ephemeris_t e_meas[MAX_CHANNELS];

    /* TODO: package meas, nav_meas and ephemeris into e.g. an union to
     * keep them from being separated? */

    /* Collect measurements from trackers, load ephemerides and compute flags */
    collect_measurements(rec_tc,
                         meas,
                         in_view,
                         e_meas,
                         &n_collected,
                         &n_inview,
                         &n_total);

    nmea_send_gsv(n_inview, in_view);

    u8 n_ready = n_collected;
    if (n_collected > MINIMUM_SV_COUNT) {
      /* There are enough measurements for a RAIM solution. Check if there
       * are enough measurements to drop out the lowest quality ones.
       */
      n_ready = filter_out_measurements(n_collected, meas, e_meas);
    }

    log_debug("Selected %" PRIu8 " measurement(s) out of %" PRIu8
              " (total=%" PRIu8 ")", n_ready, n_collected, n_total);

    /* Update stats */
    if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {
      u8 sim_sats = simulation_current_num_sats();
      last_stats.signals_tracked = sim_sats;
      last_stats.signals_useable = sim_sats;
    } else {
      last_stats.signals_tracked = n_total;
      last_stats.signals_useable = n_collected;
    }

    if (n_ready < MINIMUM_SV_COUNT) {
      /* Not enough sats, keep on looping. */

      /* TODO if there are not enough SVs to compute PVT, shouldn't caches
       *      below be reset? I.e. nav_meas_old and nav_meas_tdcp? */
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0.0, 0, &sbp_messages);
      }
      continue;
    }

    cnav_msg_t cnav_30[MAX_CHANNELS];
    const cnav_msg_type_30_t *p_cnav_30[MAX_CHANNELS];
    for (u8 i=0; i < n_ready; i++) {
      p_cnav_30[i] = cnav_msg_get(meas[i].sid, CNAV_MSG_TYPE_30, &cnav_30[i]) ?
                     &cnav_30[i].data.type_30 : NULL;
    }

    static navigation_measurement_t nav_meas[MAX_CHANNELS];
    const channel_measurement_t *p_meas[n_ready];
    navigation_measurement_t *p_nav_meas[n_ready];
    const ephemeris_t *p_e_meas[n_ready];

    /* Create arrays of pointers for use in calc_navigation_measurement */
    for (u8 i = 0; i < n_ready; i++) {
      p_meas[i] = &meas[i];
      p_nav_meas[i] = &nav_meas[i];
      p_e_meas[i] = &e_meas[i];
    }

    /* Create navigation measurements from the channel measurements */
    /* If we have timing then we can calculate the relationship between
     * receiver time and GPS time and hence provide the pseudorange
     * calculation with the local GPS time of reception. */
    /* If a FINE quality time solution is not available then don't pass in a
     * `nav_time`. This will result in valid pseudoranges but with a large
     * and arbitrary receiver clock error. We may want to discard these
     * observations after doing a PVT solution. */
    gps_time_t *p_rec_time = (time_quality == TIME_FINE) ? &rec_time : NULL;

    s8 nm_ret = calc_navigation_measurement(n_ready, p_meas, p_nav_meas,
                                            p_rec_time);

    if (nm_ret != 0) {
      log_error("calc_navigation_measurement() returned an error");
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0.0, 0, &sbp_messages);
      }
      continue;
    }

    s8 sc_ret = calc_sat_clock_corrections(n_ready, p_nav_meas, p_e_meas);

    if (sc_ret != 0) {
       log_error("calc_sat_clock_correction() returned an error");
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0.0, 0, &sbp_messages);
      }
       continue;
     }

    calc_isc(n_ready, p_nav_meas, p_cnav_30);

    static double clock_offset_previous = 0;
    static u64 rec_tc_old = 0;
    static u8 n_ready_old = 0;
    static navigation_measurement_t nav_meas_old[MAX_CHANNELS];
    static navigation_measurement_t nav_meas_tdcp[MAX_CHANNELS];

    double rec_tc_delta = (double) (rec_tc - rec_tc_old)
                                  / NAP_FRONTEND_SAMPLE_RATE_Hz;
    u8 n_ready_tdcp;
    if (!clock_jump
        && time_quality == TIME_FINE
        && rec_tc_delta < 2 / soln_freq) {

      /* Form TDCP Dopplers only if the clock has not just been adjusted,
       * and the old measurements are at most one solution cycle old. */
      n_ready_tdcp = tdcp_doppler(n_ready, nav_meas, n_ready_old, nav_meas_old,
          nav_meas_tdcp, rec_tc_delta);
    } else {

      /* Pass the nav_meas with the measured Dopplers as is */
      memcpy(nav_meas_tdcp, nav_meas, sizeof(nav_meas));
      n_ready_tdcp = n_ready;

      /* Log the reason (if unexpected)*/
      if (!clock_jump && time_quality == TIME_FINE
          && rec_tc_delta >= 2 / soln_freq) {
        log_warn(
          "Time from last measurements %f seconds, skipping TDCP computation",
          rec_tc_delta);
      }
    }

    /* Store current observations for next time for
     * TDCP Doppler calculation. */
    memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));
    n_ready_old = n_ready;
    rec_tc_old = rec_tc;

    gnss_sid_set_t codes_tdcp;
    sid_set_init(&codes_tdcp);
    for (u8 i=0; i<n_ready_tdcp; i++) {
      sid_set_add(&codes_tdcp, nav_meas_tdcp[i].sid);
    }

    if (sid_set_get_sat_count(&codes_tdcp) < 4) {
      /* Not enough sats to compute PVT */
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0.0, 0, &sbp_messages);
      }
      continue;
    }

    /* check if we have a solution, if yes calc iono and tropo correction */
    if (lgf.position_quality >= POSITION_GUESS) {
      ionosphere_t i_params;
      ionosphere_t *p_i_params = &i_params;
      /* get iono parameters if available */
      if(ndb_iono_corr_read(p_i_params) != NDB_ERR_NONE) {
        p_i_params = NULL;
        chMtxLock(&time_matched_iono_params_lock);
        has_time_matched_iono_params = false;
        chMtxUnlock(&time_matched_iono_params_lock);
      } else {
        chMtxLock(&time_matched_iono_params_lock);
        has_time_matched_iono_params = true;
        time_matched_iono_params = *p_i_params;
        chMtxUnlock(&time_matched_iono_params_lock);
      }
      calc_iono_tropo(n_ready_tdcp, nav_meas_tdcp,
                      lgf.position_solution.pos_ecef,
                      lgf.position_solution.pos_llh,
                      p_i_params);
    }

    dops_t dops;
    gnss_solution current_fix;
    gnss_signal_t raim_removed_sid;

    /* Calculate the SPP position
     * disable_raim controlled by external setting. Defaults to false. */
    /* Don't skip velocity solving. If there is a cycle slip, tdcp_doppler will
     * just return the rough value from the tracking loop. */
     // TODO(Leith) check velocity_valid
    s8 pvt_ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp, disable_raim, false,
                          &current_fix, &dops, &raim_removed_sid);
    if (pvt_ret < 0
        || (lgf.position_quality = POSITION_FIX && gate_covariance(&current_fix))) {
      /* An error occurred with calc_PVT! */
      /* pvt_err_msg defined in libswiftnav/pvt.c */
      DO_EVERY((u32)soln_freq,
         log_warn("PVT solver: %s (code %d)", pvt_err_msg[-pvt_ret-1], pvt_ret);
      );

      /* If we can't compute a SPP position, something is wrong and no point
       * continuing to process this epoch - send out solution and observation
       * failed messages if not in time matched mode
       */
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0.0, 0, &sbp_messages);
      }

      /* If we already had a good fix, degrade its quality to STATIC */
      if (lgf.position_quality > POSITION_STATIC) {
        lgf.position_quality = POSITION_STATIC;
      }
      continue;
    }

    /* Determine the time differenced clock offset which is used to remove
       receiver clock bias from the computed doppler. */
    double computed_clock_rate =
            (current_fix.clock_offset - clock_offset_previous) / rec_tc_delta;
    clock_offset_previous = current_fix.clock_offset;

    /* If we have a success RAIM repair, mark the removed observation as
       invalid. In practice, this means setting only the CN0 flag valid. */
    if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        if (sid_is_equal(nav_meas_tdcp[i].sid, raim_removed_sid)) {
          log_warn_sid(nav_meas_tdcp[i].sid, "RAIM repair, setting observation invalid.");
          nav_meas_tdcp[i].flags = NAV_MEAS_FLAG_CN0_VALID;
        }
      }
    }

    if (time_quality < TIME_FINE) {
      /* If the time quality is not FINE then our receiver clock bias isn't
       * known. We should only use this PVT solution to update our time
       * estimate and then skip all other processing.
       *
       * Note that the lack of knowledge of the receiver clock bias does NOT
       * degrade the quality of the position solution but the rapid change in
       * bias after the time estimate is first improved may cause issues for
       * e.g. carrier smoothing. Easier just to discard this first solution.
       */
      set_time_fine(rec_tc, current_fix.time);
      clock_jump = TRUE;
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0.0, 0, &sbp_messages);
      }
      /* store this fix as a guess so the satellite elevations and iono/tropo
       * corrections can be computed for the first actual fix */
      lgf.position_solution = current_fix;
      lgf.position_quality = POSITION_GUESS;
      ndb_lgf_store(&lgf);
      continue;
    }
    // We now have the nap count we expected the measurements to be at, plus the GPS time error for that nap count
    // so we need to store this error in the GPS time (GPS time frame)
    set_gps_time_offset(rec_tc, current_fix.time);

    /* Update global position solution state. */
    lgf.position_solution = current_fix;
    lgf.position_quality = POSITION_FIX;
    ndb_lgf_store(&lgf);

    if (!simulation_enabled()) {
      /* Output solution. */

      bool disable_velocity = clock_jump ||
                              (current_fix.velocity_valid == 0);
      solution_make_sbp(&current_fix, &dops, disable_velocity, &sbp_messages);
    }

    /*
     * We need to correct our pseudorange and create a new one that is valid for
     * a different time of arrival.  In particular we'd like to propagate all the
     * observations such that they represent what we would have observed had
     * the observations all arrived at the current epoch (t').
     */

    /* Calculate the time of the nearest solution epoch, where we expected
     * to be, and calculate how far we were away from it. */
    gps_time_t new_obs_time;
    new_obs_time.tow = round(current_fix.time.tow * soln_freq)
                              / soln_freq;
    normalize_gps_time(&new_obs_time);
    gps_time_match_weeks(&new_obs_time, &current_fix.time);
    double t_err = gpsdifftime(&new_obs_time, &current_fix.time);

    // Time the base obs are propagated to the rover obs
    double propagation_time = 0.0;

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(t_err) < OBS_PROPAGATION_LIMIT) {

      /* Propagate observations to desired time. */
      /* We have to use the tdcp_doppler result to account for TCXO drift. */
      /* nav_meas_tdcp is updated in place, skipping elements if required. */
      u8 n_ready_tdcp_new = 0;
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        navigation_measurement_t *nm = &nav_meas_tdcp[n_ready_tdcp_new];

        /* Copy measurement to new index if a previous measurement
         * has been skipped. */
        if (i != n_ready_tdcp_new) {
          memcpy(nm, &nav_meas_tdcp[i], sizeof(*nm));
        }

        double doppler = 0.0;
        if (0 != (nm->flags & NAV_MEAS_FLAG_MEAS_DOPPLER_VALID)) {
          doppler = nm->raw_measured_doppler;
        }

        nm->raw_carrier_phase += t_err * doppler;
        /* Note, the pseudorange correction has opposite sign because Doppler
         * has the opposite sign compared to the pseudorange rate. */
        nm->raw_pseudorange -= t_err * doppler *
                               code_to_lambda(nm->sid.code);

        /* Correct the observations for the receiver clock error. */
        nm->raw_carrier_phase += current_fix.clock_offset *
                                      GPS_C / code_to_lambda(nm->sid.code);
        nm->raw_measured_doppler += current_fix.clock_bias *
                                    GPS_C / code_to_lambda(nm->sid.code);
        nm->raw_pseudorange -= current_fix.clock_offset * GPS_C;
        nm->raw_computed_doppler += computed_clock_rate *
                                      GPS_C / code_to_lambda(nm->sid.code);

        /* Also apply the time correction to the time of transmission so the
         * satellite positions can be calculated for the correct time. */
        nm->tot.tow += t_err;
        normalize_gps_time(&nm->tot);

        ephemeris_t *e = NULL;

        /* Find the original index of this measurement in order to point to
         * the correct ephemeris. (Do not load it again from NDB because it may
         * have changed meanwhile.) */
        for (u8 j = 0; j < n_ready; j++) {
          if (sid_is_equal(nm->sid, meas[j].sid)) {
            e = &e_meas[j];
            break;
          }
        }

        if (e == NULL || 1 != ephemeris_valid(e, &nm->tot)) {
          continue;
        }

        /* Recompute satellite position, velocity and clock errors */
        if (0 != calc_sat_state(e, &nm->tot, nm->sat_pos, nm->sat_vel,
                                &nm->sat_clock_err, &nm->sat_clock_err_rate)) {
          continue;
        }

        n_ready_tdcp_new++;
      }

      /* Update n_ready_tdcp. */
      n_ready_tdcp = n_ready_tdcp_new;

      /* If we have a recent set of observations from the base station, do a
       * differential solution. */
      chMtxLock(&base_obs_lock);
      chMtxLock(&low_latency_filter_manager_lock);
      bool sender_id_unchanged = low_latency_base_sender_id == base_obss.sender_id;
      chMtxUnlock(&low_latency_filter_manager_lock);
      if(sender_id_unchanged){
        if (base_obss.n > 0 && !simulation_enabled()) {
          if ((propagation_time = gpsdifftime(&new_obs_time, &base_obss.tor))
                < max_age_of_differential) {

            /* Propagate base station observations to the current time and
             * process a low-latency differential solution. */

            /* Hook in low-latency filter here. */
            if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY &&
                base_obss.has_pos) {

              sdiff_t sdiffs[MAX(base_obss.n, n_ready_tdcp)];
              u8 num_sdiffs = make_propagated_sdiffs(n_ready_tdcp, nav_meas_tdcp,
                                      base_obss.n, base_obss.nm,
                                      base_obss.sat_dists,
                                      base_obss.sat_dists_dot,
                                      base_obss.has_known_pos_ecef ?
                                      base_obss.known_pos_ecef :
                                      base_obss.pos_ecef,
                                      sdiffs);
              output_baseline(num_sdiffs, sdiffs, &current_fix.time,
                              &dops, propagation_time, current_fix.pos_ecef,
                              base_obss.has_known_pos_ecef, base_obss.known_pos_ecef,
                              &sbp_messages);
            }
          }
        }
      }
      chMtxUnlock(&base_obs_lock);

      /* Output observations only every obs_output_divisor times, taking
       * care to ensure that the observations are aligned. */
      /* Also only output observations once our receiver clock is
       * correctly set. */
      double t_check = new_obs_time.tow * (soln_freq / obs_output_divisor);
      if (!simulation_enabled() &&
          time_quality == TIME_FINE &&
          fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
        /* Post the observations to the mailbox. */
        post_observations(n_ready_tdcp, nav_meas_tdcp, &new_obs_time, &current_fix);

        /* Send the observations. */
        send_observations(n_ready_tdcp, nav_meas_tdcp, &new_obs_time);
      }
    }

    /* Calculate the receiver clock error and if >1ms perform a clock jump */
    double rx_err = gpsdifftime(&rec_time, &current_fix.time);
    log_debug("RX clock offset = %f", rx_err);
    clock_jump = FALSE;
    if (fabs(rx_err) >= 1e-3) {
      log_info("Receiver clock offset larger than 1 ms, applying millisecond jump");
      /* round the time adjustment to even milliseconds */
      double dt = round(rx_err * 1000.0) / 1000.0;
      /* adjust the RX to GPS time conversion */
      adjust_time_fine(dt);
      /* adjust all the carrier phase offsets */
      /* note that the adjustment is always in even cycles because millisecond
       * breaks up exactly into carrier cycles
       * TODO: verify this holds for GLONASS as well */
      tracking_channel_carrier_phase_offsets_adjust(dt);
      /* adjust the stored CP measurements so that next TDCP is correct */
      for (u8 i = 0; i < n_ready_old; i++) {
        nav_meas_old[i].raw_carrier_phase += dt *
            code_to_carr_freq(nav_meas_old[i].sid.code);
      }
      clock_offset_previous -= dt;
    }

    // Send out messages if needed
    solution_send_low_latency_output(propagation_time, base_obss.sender_id, &sbp_messages);

    /* Calculate the correction to the current deadline by converting nap count
     * difference to seconds, we convert to ms to adjust deadline later */
    double dt = delta_tc * RX_DT_NOMINAL + current_fix.clock_offset / soln_freq;

    /* Limit dt to twice the max soln rate */
    double max_deadline = ((1.0 / soln_freq) * 2.0);
    if (fabs(dt) > max_deadline) {
      dt = (dt > 0.0) ? max_deadline : -1.0 * max_deadline;
    }

    /* Reset timer period with the count that we will estimate will being
     * us up to the next solution time. */
    deadline += round(dt * CH_CFG_ST_FREQUENCY);
  }
}

void process_matched_obs(u8 n_sds, obss_t *obss, sdiff_t *sds,
                         bool has_known_base_pos_ecef, double known_base_pos[3],
                         sbp_messages_t *sbp_messages)
{
  chMtxLock(&time_matched_filter_manager_lock);
  if (!filter_manager_is_initialized(time_matched_filter_manager) && (n_sds > 4)) {
    filter_manager_init(time_matched_filter_manager);
  }

  s8 ret = -1;
  if (filter_manager_is_initialized(time_matched_filter_manager)) {
    chMtxLock(&time_matched_iono_params_lock);
    if (has_time_matched_iono_params) {
      filter_manager_update_iono_parameters(time_matched_filter_manager,
                                            &time_matched_iono_params, disable_klobuchar);
    }
    chMtxUnlock(&time_matched_iono_params_lock);
    ret = filter_manager_update(time_matched_filter_manager,
                                &obss->tor, sds, n_sds,
                                obss->has_pos ? obss->pos_ecef : NULL,
                                has_known_base_pos_ecef ? known_base_pos : NULL, 0.0);

    if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
      /* If we're in low latency mode we need to copy/update the low latency
         filter manager from the time matched filter manager. */
      chMtxLock(&low_latency_filter_manager_lock);
      copy_filter_manager(low_latency_filter_manager, time_matched_filter_manager);
      low_latency_base_sender_id = old_base_sender_id;
      chMtxUnlock(&low_latency_filter_manager_lock);
    }
  }

  /* If we are in time matched mode then calculate and output the baseline
  * for this observation. */
  if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED &&
      !simulation_enabled() && ret == 0) {
    /* Note: in time match mode we send the physically incorrect time of the
     * observation message (which can be receiver clock time, or rounded GPS
     * time) instead of the true GPS time of the solution. */
    dops_t RTK_dops;
    output_baseline(n_sds, sds, &obss->tor, &RTK_dops, 0, obss->pos_ecef,
                    has_known_base_pos_ecef, known_base_pos,
                    sbp_messages);
  }
  chMtxUnlock(&time_matched_filter_manager_lock);
}

static WORKING_AREA_CCM(wa_time_matched_obs_thread, 5000000);
static void time_matched_obs_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("time matched obs");

  // Declare all SBP messages
  sbp_messages_t sbp_messages;

  chMtxLock(&time_matched_filter_manager_lock);
  time_matched_filter_manager = create_filter_manager();
  chMtxUnlock(&time_matched_filter_manager_lock);

  chMtxLock(&low_latency_filter_manager_lock);
  low_latency_filter_manager = create_filter_manager();
  chMtxUnlock(&low_latency_filter_manager_lock);

  while (1) {
    /* Wait for a new observation to arrive from the base station. */
    chBSemWait(&base_obs_received);

    chMtxLock(&base_obs_lock);
    obss_t base_obss_copy = base_obss;
    chMtxUnlock(&base_obs_lock);

    // Init the messages we want to send
    sbp_messages_init(&sbp_messages);

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (chMBFetch(&obs_mailbox, (msg_t *)&obss, TIME_IMMEDIATE)
            == MSG_OK) {

      if (dgnss_soln_mode == SOLN_MODE_NO_DGNSS) {
        // Not doing any DGNSS.  Toss the obs away.
        chPoolFree(&obs_buff_pool, obss);
        continue;
      }

      double dt = gpsdifftime(&obss->tor, &base_obss_copy.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD) {
        /* Check if the base sender ID has changed and reset the RTK filter if
         * it has.
         */
        if ((old_base_sender_id != 0) &&
            (old_base_sender_id != base_obss_copy.sender_id)) {
          log_warn("Base station sender ID changed from %u to %u. Resetting RTK"
                   " filter.", old_base_sender_id, base_obss_copy.sender_id);
          reset_rtk_filter();
          chMtxLock(&base_pos_lock);
          base_pos_known = false;
          memset(&base_pos_ecef, 0, sizeof(base_pos_ecef));
          chMtxUnlock(&base_pos_lock);
        }
        old_base_sender_id = base_obss_copy.sender_id;

        /* Times match! Process obs and base_obss_copy */
        static sdiff_t sds[MAX_CHANNELS];
        u8 n_sds = single_diff(
            obss->n, obss->nm,
            base_obss_copy.n, base_obss_copy.nm,
            sds
        );
        bool has_known_base_pos_ecef = base_obss_copy.has_known_pos_ecef;
        double known_base_pos[3];
        if (has_known_base_pos_ecef) {
          memcpy(known_base_pos, base_obss_copy.known_pos_ecef, sizeof(base_obss_copy.known_pos_ecef));
        }
        // We need to form the SBP messages derived from the SPP at this solution time before we
        // do the differential solution so that the various messages can be overwritten as appropriate,
        // the exception is the DOP messages, as we don't have the SPP DOP and it will always be overwritten by the differential
        gnss_solution soln_copy = obss->soln;
        solution_make_sbp(&soln_copy,NULL,false, &sbp_messages);

        process_matched_obs(n_sds, obss, sds, has_known_base_pos_ecef, known_base_pos, &sbp_messages);
        chPoolFree(&obs_buff_pool, obss);
        if (spp_timeout(&last_spp, &last_dgnss, dgnss_soln_mode)) {
          solution_send_pos_messages(0.0, base_obss_copy.sender_id, &sbp_messages);
        }
        break;
      } else {
        if (dt > 0) {
          /* Time of base obs before time of local obs, we must not have a local
           * observation matching this base observation, break and wait for a
           * new base observation. */

          /* In practice this should basically never happen so lets make a note
           * if it does. */
          log_warn("Obs Matching: t_base < t_rover "
                   "(dt=%f obss.t={%d,%f} base_obss.t={%d,%f})", dt,
                   obss->tor.wn, obss->tor.tow,
                   base_obss_copy.tor.wn, base_obss_copy.tor.tow
          );
          /* Return the buffer to the mailbox so we can try it again later. */
          msg_t ret = chMBPost(&obs_mailbox, (msg_t)obss, TIME_IMMEDIATE);
          if (ret != MSG_OK) {
            /* Something went wrong with returning it to the buffer, better just
             * free it and carry on. */
            log_warn("Obs Matching: mailbox full, discarding observation!");
            chPoolFree(&obs_buff_pool, obss);
          }
          break;
        } else {
          /* Time of base obs later than time of local obs,
           * keep moving through the mailbox. */
          chPoolFree(&obs_buff_pool, obss);
        }
      }
    }
  }
}

void reset_filters_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)context;
  switch (msg[0]) {
  case 0:
    log_info("Filter reset requested");
    reset_rtk_filter();
    break;
  default:
    break;
  }
}

soln_stats_t solution_last_stats_get(void)
{
  return last_stats;
}

soln_pvt_stats_t solution_last_pvt_stats_get(void)
{
  return last_pvt_stats;
}

soln_dgnss_stats_t solution_last_dgnss_stats_get(void)
{
  return last_dgnss_stats;
}

/* Check that -180.0 <= new heading_offset setting value <= 180.0. */
static bool heading_offset_changed(struct setting *s, const char *val)
{
  double offset;
  bool ret = s->type->from_string(s->type->priv, &offset, s->len, val);
  if (!ret) {
    return ret;
  }

  if (fabs(offset) > 180.0) {
    log_error("Invalid heading offset setting of %3.1f, max is %3.1f, min is %3.1f, leaving heading offset at %3.1f",
              offset, 180.0, -180.0, heading_offset);
    ret = false;
  }
  *(double*)s->addr = offset;
  return ret;
}


void solution_setup()
{
  /* Set time of last differential solution in the past. */
  last_dgnss.wn = 0;
  last_dgnss.tow = 0;

  SETTING("solution", "soln_freq", soln_freq, TYPE_FLOAT);
  SETTING("solution", "correction_age_max", max_age_of_differential, TYPE_INT);
  SETTING("solution", "output_every_n_obs", obs_output_divisor, TYPE_INT);

  static const char const *dgnss_soln_mode_enum[] = {
    "Low Latency",
    "Time Matched",
    "No DGNSS",
    NULL
  };
  static struct setting_type dgnss_soln_mode_setting;
  int TYPE_GNSS_SOLN_MODE = settings_type_register_enum(dgnss_soln_mode_enum,
                                                        &dgnss_soln_mode_setting);
  SETTING("solution", "dgnss_solution_mode",
          dgnss_soln_mode, TYPE_GNSS_SOLN_MODE);

  static const char const *dgnss_filter_enum[] = {
    "Float",
    "Fixed",
    NULL
  };
  static struct setting_type dgnss_filter_setting;
  int TYPE_GNSS_FILTER = settings_type_register_enum(dgnss_filter_enum,
                                                     &dgnss_filter_setting);
  SETTING("solution", "dgnss_filter",
          dgnss_filter, TYPE_GNSS_FILTER);

  SETTING("sbp", "obs_msg_max_size", msg_obs_max_size, TYPE_INT);

  SETTING("solution", "disable_raim", disable_raim, TYPE_BOOL);
  SETTING("solution", "send_heading", send_heading, TYPE_BOOL);
  SETTING_NOTIFY("solution", "heading_offset", heading_offset, TYPE_FLOAT, heading_offset_changed);

  SETTING("solution", "disable_klobuchar_correction", disable_klobuchar, TYPE_BOOL);

  nmea_setup();

  static msg_t obs_mailbox_buff[OBS_N_BUFF];
  chMBObjectInit(&obs_mailbox, obs_mailbox_buff, OBS_N_BUFF);
  chPoolObjectInit(&obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[OBS_N_BUFF] _CCM;
  chPoolLoadArray(&obs_buff_pool, obs_buff, OBS_N_BUFF);

  /* Start solution thread */
  chThdCreateStatic(wa_solution_thread, sizeof(wa_solution_thread),
                    HIGHPRIO-2, solution_thread, NULL);
  chThdCreateStatic(wa_time_matched_obs_thread,
                    sizeof(wa_time_matched_obs_thread), NORMALPRIO - 3,
                    time_matched_obs_thread, NULL);

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
    SBP_MSG_RESET_FILTERS,
    &reset_filters_callback,
    &reset_filters_node
  );
}
