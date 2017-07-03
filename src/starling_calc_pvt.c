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
#include "starling_calc_pvt.h"
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
#include "common_calc_pvt.h"
#include "me_calc_pvt.h"


/* Maximum CPU time the solution thread is allowed to use. */
#define SOLN_THD_CPU_MAX (0.60f)

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

/** Minimum number of satellites to use with PVT */
#define MINIMUM_SV_COUNT 5

static MemoryPool time_matched_obs_buff_pool;
static mailbox_t  time_matched_obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;
dgnss_filter_t dgnss_filter = FILTER_FLOAT;

static FilterManager *time_matched_filter_manager;
static FilterManager *low_latency_filter_manager;

MUTEX_DECL(time_matched_filter_manager_lock);
MUTEX_DECL(low_latency_filter_manager_lock);

MUTEX_DECL(time_matched_iono_params_lock);
bool has_time_matched_iono_params = false;
static ionosphere_t time_matched_iono_params;

/* RFT_TODO *
 * starling_simulation_obs_output_divisor logic might have to change */
static u32 starling_simulation_obs_output_divisor=1;

MUTEX_DECL(last_sbp_lock);
gps_time_t last_dgnss;
gps_time_t last_spp;

/* RFT_TODO *
 * interface change should be handled last, won't work like this */
static double starling_frequency = 5.0;
u32 max_age_of_differential = 30;


bool disable_raim = false;
bool send_heading = false;

double heading_offset = 0.0;

bool disable_klobuchar = false;

static soln_dgnss_stats_t last_dgnss_stats = { .systime = -1, .mode = 0 };



static void post_observations(u8 n, const navigation_measurement_t m[],
                              const gps_time_t *t, const gnss_solution *soln )
{
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

    ret = chMBPost(&time_matched_obs_mailbox, (msg_t)obs, TIME_IMMEDIATE);
    if (ret != MSG_OK) {
      /* We could grab another item from the mailbox, discard it and then
       * post our obs again but if the size of the mailbox and the pool
       * are equal then we should have already handled the case where the
       * mailbox is full when we handled the case that the pool was full.
       * */
      log_error("Mailbox should have space!");
      chPoolFree(&time_matched_obs_buff_pool, obs);
    }
  }
}

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
    utc_params_t *p_utc_params = &utc_params;
    bool is_nv;
    /* try to read UTC parameters from NDB */
    if (NDB_ERR_NONE == ndb_utc_params_read(&utc_params, &is_nv)) {
      if (is_nv) {
        utc_flags |= (NVM_UTC << 3);
      } else {
        utc_flags |= (DECODED_UTC << 3);
      }
    } else {
      p_utc_params = NULL;
      utc_flags |= (DEFAULT_UTC << 3);
    }

    sbp_make_utc_time(&sbp_messages->utc_time, &soln->time, utc_flags, p_utc_params);

    /* Extract full covariance matrices for position and velocity solutions
     * from the upper triangular forms given in soln->err_cov and vel_cov */
    double full_covariance[9];
    double vel_covariance[9];
    extract_covariance(full_covariance, vel_covariance, soln);

    /* Compute the accuracy figures from the covariance matrix */
    double accuracy, h_accuracy, v_accuracy;
    covariance_to_accuracy(full_covariance, soln->pos_ecef,
                           &accuracy, &h_accuracy, &v_accuracy);

    double vel_accuracy, vel_h_accuracy, vel_v_accuracy;
    covariance_to_accuracy(vel_covariance, soln->pos_ecef,
                           &vel_accuracy, &vel_h_accuracy, &vel_v_accuracy);

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

  } else {
    gps_time_t time_guess = get_current_time();
    sbp_make_gps_time(&sbp_messages->gps_time, &time_guess, 0);
  }
}

/**
 *
 * @param sender_id sender id of base obs
 * @param sbp_messages struct of sbp messages
 */
static void solution_send_pos_messages(u8 sender_id, const sbp_messages_t *sbp_messages) {

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
  utc_params_t *p_utc_params = &utc_params;
  /* read UTC parameters from NDB if they exist*/
  if (NDB_ERR_NONE != ndb_utc_params_read(&utc_params, NULL)) {
    p_utc_params = NULL;
  }

  /* Send NMEA alongside the sbp */
  double propagation_time = sbp_messages->age_corrections.age * 0.1;
  nmea_send_msgs(&sbp_messages->pos_llh, &sbp_messages->vel_ned,
                 &sbp_messages->sbp_dops, &sbp_messages->gps_time,
                 propagation_time, sender_id, p_utc_params,
                 &sbp_messages->baseline_heading);

}

static void solution_send_low_latency_output(u8 sender_id, const sbp_messages_t *sbp_messages) {
  // Work out if we need to wait for a certain period of no time matched positions before we output a SBP position
  bool wait_for_timeout = false;
  if (!(dgnss_timeout(last_dgnss_stats.systime, dgnss_soln_mode)) && dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    wait_for_timeout = true;
  }

  if (!wait_for_timeout) {
    solution_send_pos_messages(sender_id, sbp_messages);
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
void solution_make_baseline_sbp(const rtk_baseline_result_t *result,
                                const double rover_ecef[3], const dops_t *dops,
                                sbp_messages_t *sbp_messages) {
  double b_ned[3];
  wgsecef2ned(result->baseline, rover_ecef, b_ned);

  double accuracy, h_accuracy, v_accuracy;
  covariance_to_accuracy(result->covariance, rover_ecef, &accuracy, &h_accuracy,
                         &v_accuracy);

  sbp_make_baseline_ecef(&sbp_messages->baseline_ecef, &result->result_time,
                         result->num_sats_used, result->baseline, accuracy,
                         result->flags);

  sbp_make_baseline_ned(&sbp_messages->baseline_ned, &result->result_time,
                        result->num_sats_used, b_ned, h_accuracy, v_accuracy,
                        result->flags);

  sbp_make_age_corrections(&sbp_messages->age_corrections, &result->result_time,
                           result->propagation_time);

  sbp_make_dgnss_status(&sbp_messages->dgnss_status, result->num_sats_used,
                        result->propagation_time, result->flags);

  if (result->flags == FIXED_POSITION &&
      dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    double heading = calc_heading(b_ned);
    sbp_make_heading(&sbp_messages->baseline_heading, &result->result_time,
                     heading + heading_offset, result->num_sats_used,
                     result->flags);
  }

  if (result->has_known_reference_pos ||
      (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
       simulation_enabled_for(SIMULATION_MODE_RTK))) {
    double pseudo_absolute_ecef[3];
    double pseudo_absolute_llh[3];

    vector_add(3, result->known_reference_pos, result->baseline,
               pseudo_absolute_ecef);
    wgsecef2llh(pseudo_absolute_ecef, pseudo_absolute_llh);

    /* now send pseudo absolute sbp message */
    sbp_make_pos_llh_vect(&sbp_messages->pos_llh, pseudo_absolute_llh,
                          h_accuracy, v_accuracy, &result->result_time,
                          result->num_sats_used, result->flags);
    sbp_make_pos_ecef_vect(&sbp_messages->pos_ecef, pseudo_absolute_ecef,
                           accuracy, &result->result_time,
                           result->num_sats_used, result->flags);
  }
  sbp_make_dops(&sbp_messages->sbp_dops, dops, sbp_messages->pos_llh.tow,
                result->flags);

  chMtxLock(&last_sbp_lock);
  last_dgnss.wn = result->result_time.wn;
  last_dgnss.tow = result->result_time.tow;
  chMtxUnlock(&last_sbp_lock);

  /* Update stats */
  last_dgnss_stats.systime = chVTGetSystemTime();
  last_dgnss_stats.mode =
      (result->flags == FIXED_POSITION) ? FILTER_FIXED : FILTER_FLOAT;
}

static PVT_ENGINE_INTERFACE_RC update_filter(FilterManager *filter_manager) {
  PVT_ENGINE_INTERFACE_RC ret = PVT_ENGINE_FAILURE;
  if (filter_manager_is_initialized(filter_manager)) {
    ret = filter_manager_update(filter_manager);
    if (ret != PVT_ENGINE_SUCCESS) {
      detailed_log_info("Skipping filter update with ret = %d", ret);
    }
  } else {
    detailed_log_info("Filter not initialized.");
  }
  return ret;
}

static PVT_ENGINE_INTERFACE_RC get_baseline(
    const FilterManager *filter_manager, const bool use_time_matched_baseline,
    dops_t *dops, rtk_baseline_result_t *result) {
  PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;

  get_baseline_ret = filter_manager_get_result(
      filter_manager, use_time_matched_baseline, result);

  if (get_baseline_ret == PVT_ENGINE_SUCCESS) {
    *dops = filter_manager_get_dop_values(filter_manager);
  } else {
    detailed_log_info("Baseline calculation failed");
  }
  return get_baseline_ret;
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

    rtk_baseline_result_t result = {
        .result_time = soln->time,
        .num_sats_used = simulation_current_num_sats(),
        .num_sigs_used = 0,
        .flags = flags,
        .has_known_reference_pos = true,
        .propagation_time = 0.0,
    };
    memcpy(result.baseline, simulation_current_baseline_ecef(),
           sizeof(result.baseline));
    memcpy(result.covariance, simulation_current_covariance_ecef(),
           sizeof(result.covariance));
    memcpy(result.known_reference_pos, simulation_ref_ecef(),
           sizeof(result.known_reference_pos));

    solution_make_baseline_sbp(&result, simulation_ref_ecef(),
                               simulation_current_dops_solution(),
                               sbp_messages);

    double t_check = soln->time.tow * (starling_frequency / starling_simulation_obs_output_divisor);
    if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
      /* RFT_TODO *
       * SBP_FRAMING_MAX_PAYLOAD_SIZE replaces the setting for now, but
       * this function will completely go away */
      send_observations(simulation_current_num_sats(), SBP_FRAMING_MAX_PAYLOAD_SIZE,
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
  almanac_t almanac;
  double az, el;
  u64 nap_count = gpstime2napcount(&t);

  /* compute elevation for any valid ephemeris/almanac we can pull from NDB */
  for (u16 sv_index = 0; sv_index < NUM_SATS; sv_index++) {

    /* form a SID with the first code for the constellation */
    gnss_signal_t sid = sv_index_to_sid(sv_index);
    if (!sid_valid(sid)) {
      continue;
    }
    ndb_op_code_t res = ndb_ephemeris_read(sid, &ephemeris);

    /* try to compute elevation from any valid ephemeris */
    if ((NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res)
        && ephemeris_valid(&ephemeris, &t)
        && calc_sat_az_el(&ephemeris, &t, rcv_pos, &az, &el, false) >= 0) {
      sv_azel_degrees_set(sid, round(az * R2D), round(el * R2D), nap_count);
      log_debug_sid(sid, "Updated elevation from ephemeris %.1f", el * R2D);

    /* else try to fetch almanac and use it if it is valid */
    } else if (NDB_ERR_NONE == ndb_almanac_read(sid, &almanac)
               && calc_sat_az_el_almanac(&almanac, &t, rcv_pos, &az, &el) >= 0) {
      sv_azel_degrees_set(sid, round(az * R2D), round(el * R2D), nap_count);
      log_debug_sid(sid, "Updated elevation from almanac %.1f", el * R2D);
    }
  }
}


/* RFT_TODO *
 * removing sleep, as it shouldn't be needed when starling is called
 * by ME, however we might want to keep the CPU usage check */

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

static THD_WORKING_AREA(wa_starling_thread, 5000000);
static void starling_thread(void *arg)
{
  (void)arg;
  msg_t ret;
  me_msg_obs_t *rover_channel_epoch;

  chRegSetThreadName("starling");

  sbp_messages_t sbp_messages;

  bool clock_jump = FALSE;
  static last_good_fix_t lgf;
  static navigation_measurement_t nav_meas[MAX_CHANNELS];
  static ephemeris_t e_meas[MAX_CHANNELS];

  /* RFT_TODO *
   * removed access to NDB */
  /* ndb_lgf_read(&lgf); */
  memset(&lgf, 0, sizeof(last_good_fix_t));

  while (TRUE) {

    watchdog_notify(WD_NOTIFY_STARLING);

    rover_channel_epoch = NULL;
    ret = chMBFetch(&obs_mailbox, (msg_t *)&rover_channel_epoch, DGNSS_TIMEOUT_MS);
    if (ret != MSG_OK) {
      if (NULL != rover_channel_epoch) {
        log_error("STARLING: mailbox fetch failed with %d", ret);
        chPoolFree(&obs_buff_pool, rover_channel_epoch);
      }
      continue;
    }

    u8 n_ready = (rover_channel_epoch->size);
    memset(nav_meas, 0, sizeof(nav_meas));
    memcpy(nav_meas, rover_channel_epoch->obs,   n_ready*sizeof(navigation_measurement_t));
    memset(e_meas, 0, sizeof(e_meas));
    memcpy(e_meas,   rover_channel_epoch->ephem, n_ready*sizeof(ephemeris_t));

    chPoolFree(&obs_buff_pool, rover_channel_epoch);

    starling_frequency = soln_freq;

    // Init the messages we want to send
    sbp_messages_init(&sbp_messages);

    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation(&sbp_messages);
    }

    /* RFT_TODO: we know update_sat_azel() should be removed but we think
     * calc_iono_tropo() migth still need it */
    if (time_quality >= TIME_COARSE
        && lgf.position_solution.valid
        && lgf.position_quality >= POSITION_GUESS) {
      /* Update the satellite elevation angles so that they stay current
       * (currently once every 30 seconds) */
      DO_EVERY((u32)starling_frequency * MAX_AZ_EL_AGE_SEC/2,
               update_sat_azel(lgf.position_solution.pos_ecef,
                               lgf.position_solution.time));
    }

    /* RFT_TODO *
     * these ^^^ should be filled by parsing rover_channel_epoch, otherwise it
     * will be always zero and nothing will be ever done */

    if (n_ready < MINIMUM_SV_COUNT) {
      /* Not enough sats, keep on looping. */
      continue;
    }

    static navigation_measurement_t nav_meas_old[MAX_CHANNELS];
    static navigation_measurement_t nav_meas_tdcp[MAX_CHANNELS];

    /* RFT_TODO *
     * tdcp computation removed as heavily dependent on time from NAP */
    u8 n_ready_tdcp;

    /* Pass the nav_meas with the measured Dopplers as is */
    memcpy(nav_meas_tdcp, nav_meas, sizeof(nav_meas));
    n_ready_tdcp = n_ready;

    /* Store current observations for next time for
     * TDCP Doppler calculation. */
    memcpy(nav_meas_old, nav_meas, sizeof(nav_meas));

    gnss_sid_set_t codes_tdcp;
    sid_set_init(&codes_tdcp);
    for (u8 i=0; i<n_ready_tdcp; i++) {
      sid_set_add(&codes_tdcp, nav_meas_tdcp[i].sid);
    }

    if (sid_set_get_sat_count(&codes_tdcp) < 4) {
      /* Not enough sats to compute PVT */
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0, &sbp_messages);
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
    gnss_sid_set_t raim_removed_sids;

    /* Calculate the SPP position
     * disable_raim controlled by external setting. Defaults to false. */
    /* Don't skip velocity solving. If there is a cycle slip, tdcp_doppler will
     * just return the rough value from the tracking loop. */
     // TODO(Leith) check velocity_valid
    s8 pvt_ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp, disable_raim, false,
                          &current_fix, &dops, &raim_removed_sids);
    if (pvt_ret < 0
        || (lgf.position_quality == POSITION_FIX && gate_covariance(&current_fix))) {

      if (pvt_ret < 0) {
        /* An error occurred with calc_PVT! */
        /* pvt_err_msg defined in libswiftnav/pvt.c */
        DO_EVERY((u32)starling_frequency,
                 log_warn("PVT solver: %s (code %d)", pvt_err_msg[-pvt_ret-1], pvt_ret);
        );
      }

      /* If we can't report a SPP position, something is wrong and no point
       * continuing to process this epoch - send out solution and observation
       * failed messages if not in time matched mode
       */
      if(dgnss_soln_mode != SOLN_MODE_TIME_MATCHED) {
        solution_send_low_latency_output(0, &sbp_messages);
      }

      /* If we already had a good fix, degrade its quality to STATIC */
      if (lgf.position_quality > POSITION_STATIC) {
        lgf.position_quality = POSITION_STATIC;
      }
      continue;
    }

    /* If we have a success RAIM repair, mark the removed observation as
       invalid. In practice, this means setting only the CN0 flag valid. */
    if (pvt_ret == PVT_CONVERGED_RAIM_REPAIR) {
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        if (sid_set_contains(&raim_removed_sids, nav_meas_tdcp[i].sid)) {
          log_warn_sid(nav_meas_tdcp[i].sid, "RAIM repair, setting observation invalid.");
          nav_meas_tdcp[i].flags |= NAV_MEAS_FLAG_RAIM_EXCLUSION;
        }
      }
    }

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
    gps_time_t new_obs_time = GPS_TIME_UNKNOWN;
    new_obs_time.tow = round(current_fix.time.tow * starling_frequency)
                              / starling_frequency;
    normalize_gps_time(&new_obs_time);
    gps_time_match_weeks(&new_obs_time, &current_fix.time);
    double t_err = gpsdifftime(&new_obs_time, &current_fix.time);

    /* Only send observations that are closely aligned with the desired
     * solution epochs to ensure they haven't been propagated too far. */
    if (fabs(t_err) < OBS_PROPAGATION_LIMIT) {

      // This will duplicate pointers to satellites with mutliple frequencies,
      // but this scenario is expected and handled
      const ephemeris_t *stored_ephs[MAX_CHANNELS];
      for( s16 i = 0; i < MAX_CHANNELS; ++i ) {
        stored_ephs[i] = NULL;
      }
      for (u8 i = 0; i < n_ready_tdcp; i++) {
        navigation_measurement_t *nm = &nav_meas_tdcp[i];
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

      if (!simulation_enabled() && dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
        PVT_ENGINE_INTERFACE_RC update_rov_obs = PVT_ENGINE_FAILURE;
        PVT_ENGINE_INTERFACE_RC update_filter_ret = PVT_ENGINE_FAILURE;
        PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;
        rtk_baseline_result_t result;

        chMtxLock(&low_latency_filter_manager_lock);

        bool is_initialized =
            filter_manager_is_initialized(low_latency_filter_manager);

        if (is_initialized) {
          set_pvt_engine_elevation_mask(low_latency_filter_manager,get_solution_elevation_mask());

          filter_manager_overwrite_ephemerides( low_latency_filter_manager, stored_ephs);

          update_rov_obs = filter_manager_update_rov_obs(low_latency_filter_manager,
                                        &current_fix.time, n_ready_tdcp,
                                        nav_meas_tdcp);


        }

        if (update_rov_obs == PVT_ENGINE_SUCCESS) {
          update_filter_ret = update_filter(low_latency_filter_manager);
        }

        if (update_filter_ret == PVT_ENGINE_SUCCESS) {
          result.result_time = current_fix.time;
          get_baseline_ret =
              get_baseline(low_latency_filter_manager, false, &dops, &result);
        }

        chMtxUnlock(&low_latency_filter_manager_lock);

        if (get_baseline_ret == PVT_ENGINE_SUCCESS) {
          solution_make_baseline_sbp(&result, current_fix.pos_ecef, &dops,
                                     &sbp_messages);
        }
      }

      /* Output observations only every obs_output_divisor times, taking
       * care to ensure that the observations are aligned. */
      /* Also only output observations once our receiver clock is
       * correctly set. */
      const u32 nav_output_divisor = 1;
      /* RFT_TODO *
       * starling should determine the obs rate depending on the
       * mailbox communication with the rover, right now it's fixed...
       * the divisor should be a user setting so that every N obs there
       * is a solution produced */
      double t_check = new_obs_time.tow * (starling_frequency / nav_output_divisor);
      if (!simulation_enabled() &&
          time_quality == TIME_FINE &&
          fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
        /* Post the observations to the mailbox. */
        post_observations(n_ready_tdcp, nav_meas_tdcp, &new_obs_time, &current_fix);

      }
    }

    // Send out messages if needed
    solution_send_low_latency_output(base_obss.sender_id, &sbp_messages);
  }
}

void process_matched_obs(const obss_t *rover_channel_meass, const obss_t *reference_obss,
                         sbp_messages_t *sbp_messages) {
  PVT_ENGINE_INTERFACE_RC update_rov_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_ref_obs = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC update_filter_ret = PVT_ENGINE_FAILURE;
  PVT_ENGINE_INTERFACE_RC get_baseline_ret = PVT_ENGINE_FAILURE;
  dops_t RTK_dops;
  rtk_baseline_result_t result;

  ephemeris_t ephs[MAX_CHANNELS];
  const ephemeris_t *stored_ephs[MAX_CHANNELS];
  for( s16 i = 0; i < MAX_CHANNELS; ++i ) {
    stored_ephs[i] = NULL;
  }

  for (u8 i = 0; i < rover_channel_meass->n; i++) {
    const navigation_measurement_t *nm = &rover_channel_meass->nm[i];
    if( NDB_ERR_NONE == ndb_ephemeris_read(nm->sid, &ephs[i]) ) {
      if (1 == ephemeris_valid(&ephs[i], &nm->tot)) {
        stored_ephs[i] = &ephs[i];
      }
    }
  }

  chMtxLock(&time_matched_filter_manager_lock);

  if (!filter_manager_is_initialized(time_matched_filter_manager)) {
    filter_manager_init(time_matched_filter_manager);
  }

  if (filter_manager_is_initialized(time_matched_filter_manager)) {
    filter_manager_overwrite_ephemerides( time_matched_filter_manager, stored_ephs);

    chMtxLock(&time_matched_iono_params_lock);
    if (has_time_matched_iono_params) {
      filter_manager_update_iono_parameters(time_matched_filter_manager,
                                            &time_matched_iono_params,
                                            disable_klobuchar);
    }
    chMtxUnlock(&time_matched_iono_params_lock);

    update_rov_obs = filter_manager_update_rov_obs(time_matched_filter_manager, &rover_channel_meass->tor,
                                  rover_channel_meass->n, rover_channel_meass->nm);
    update_ref_obs = filter_manager_update_ref_obs(
        time_matched_filter_manager, &reference_obss->tor, reference_obss->n,
        reference_obss->nm, reference_obss->pos_ecef,
        reference_obss->has_known_pos_ecef ? reference_obss->known_pos_ecef
                                           : NULL);

    if ( update_rov_obs == PVT_ENGINE_SUCCESS &&
        update_ref_obs == PVT_ENGINE_SUCCESS) {
      update_filter_ret = update_filter(time_matched_filter_manager);
    }

    if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY) {
      /* If we're in low latency mode we need to copy/update the low latency
         filter manager from the time matched filter manager. */
      chMtxLock(&low_latency_filter_manager_lock);
      copy_filter_manager(low_latency_filter_manager,
                          time_matched_filter_manager);
      chMtxUnlock(&low_latency_filter_manager_lock);
    }
  }

  /* If we are in time matched mode then calculate and output the baseline
  * for this observation. */
  if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED && !simulation_enabled() &&
      update_filter_ret == PVT_ENGINE_SUCCESS) {
    /* Note: in time match mode we send the physically incorrect time of the
     * observation message (which can be receiver clock time, or rounded GPS
     * time) instead of the true GPS time of the solution. */
    result.result_time = rover_channel_meass->tor;
    result.propagation_time = 0;
    get_baseline_ret =
        get_baseline(time_matched_filter_manager, true, &RTK_dops, &result);
  }

  chMtxUnlock(&time_matched_filter_manager_lock);

  if (get_baseline_ret == PVT_ENGINE_SUCCESS) {
    solution_make_baseline_sbp(&result, rover_channel_meass->pos_ecef, &RTK_dops,
                               sbp_messages);
  }
}

bool update_time_matched(gps_time_t *last_update_time, gps_time_t *current_time, u8 num_obs){
  double update_dt = gpsdifftime(current_time, last_update_time);
  double update_rate_limit = 0.99;
  if(num_obs > 16) {
    update_rate_limit = 1.99;
  }
  if(update_dt < update_rate_limit) {
    return false;
  }
  return true;
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

    // Init the messages we want to send
    sbp_messages_init(&sbp_messages);

    chMtxLock(&base_obs_lock);
    obss_t base_obss_copy = base_obss;
    chMtxUnlock(&base_obs_lock);

    // Check if the el mask has changed and update
    chMtxLock(&time_matched_filter_manager_lock);
    set_pvt_engine_elevation_mask(time_matched_filter_manager,get_solution_elevation_mask());
    chMtxUnlock(&time_matched_filter_manager_lock);

    obss_t *obss;
    /* Look through the mailbox (FIFO queue) of locally generated observations
     * looking for one that matches in time. */
    while (chMBFetch(&time_matched_obs_mailbox, (msg_t *)&obss, TIME_IMMEDIATE)
            == MSG_OK) {

      if (dgnss_soln_mode == SOLN_MODE_NO_DGNSS) {
        // Not doing any DGNSS.  Toss the obs away.
        chPoolFree(&time_matched_obs_buff_pool, obss);
        continue;
      }

      double dt = gpsdifftime(&obss->tor, &base_obss_copy.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD &&
          (base_obss_copy.has_pos == 1 || base_obss_copy.has_known_pos_ecef)) {
        // We need to form the SBP messages derived from the SPP at this solution time before we
        // do the differential solution so that the various messages can be overwritten as appropriate,
        // the exception is the DOP messages, as we don't have the SPP DOP and it will always be overwritten by the differential
        gnss_solution soln_copy = obss->soln;
        solution_make_sbp(&soln_copy, NULL, false, &sbp_messages);

        static gps_time_t last_update_time = {.wn = 0, .tow = 0.0};
        if(update_time_matched(&last_update_time, &obss->tor, obss->n) || dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
          process_matched_obs(obss, &base_obss_copy, &sbp_messages);
          last_update_time = obss->tor;
        }

        chPoolFree(&time_matched_obs_buff_pool, obss);
        if (spp_timeout(&last_spp, &last_dgnss, dgnss_soln_mode)) {
          solution_send_pos_messages(base_obss_copy.sender_id, &sbp_messages);
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
          msg_t ret = chMBPost(&time_matched_obs_mailbox, (msg_t)obss, TIME_IMMEDIATE);
          if (ret != MSG_OK) {
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

static bool enable_fix_mode(struct setting *s, const char *val)
{
  int value;
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
  *(dgnss_filter_t*)s->addr = value;
  return ret;
}

static bool set_max_age(struct setting *s, const char *val)
{
  int value;
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
  *(int*)s->addr = value;
  return ret;
}

void starling_calc_pvt_setup()
{
  /* Set time of last differential solution in the past. */
  last_dgnss = GPS_TIME_UNKNOWN;
  last_spp = GPS_TIME_UNKNOWN;

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


  SETTING("solution", "disable_raim", disable_raim, TYPE_BOOL);
  SETTING("solution", "send_heading", send_heading, TYPE_BOOL);
  SETTING_NOTIFY("solution", "heading_offset", heading_offset, TYPE_FLOAT, heading_offset_changed);

  SETTING("solution", "disable_klobuchar_correction", disable_klobuchar, TYPE_BOOL);

  static msg_t time_matched_obs_mailbox_buff[STARLING_OBS_N_BUFF];
  chMBObjectInit(&time_matched_obs_mailbox, time_matched_obs_mailbox_buff, STARLING_OBS_N_BUFF);
  chPoolObjectInit(&time_matched_obs_buff_pool, sizeof(obss_t), NULL);
  static obss_t obs_buff[STARLING_OBS_N_BUFF] _CCM;
  chPoolLoadArray(&time_matched_obs_buff_pool, obs_buff, STARLING_OBS_N_BUFF);

  /* Start solution thread */
  chThdCreateStatic(wa_starling_thread, sizeof(wa_starling_thread),
                    HIGHPRIO-3, starling_thread, NULL);
  chThdCreateStatic(wa_time_matched_obs_thread,
                    sizeof(wa_time_matched_obs_thread), NORMALPRIO-3,
                    time_matched_obs_thread, NULL);

  SETTING_NOTIFY("solution", "dgnss_filter", dgnss_filter, TYPE_GNSS_FILTER, enable_fix_mode);
  SETTING_NOTIFY("solution", "correction_age_max", max_age_of_differential, TYPE_INT, set_max_age);

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
    SBP_MSG_RESET_FILTERS,
    &reset_filters_callback,
    &reset_filters_node
  );
}
