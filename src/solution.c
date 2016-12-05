/*
 * Copyright (C) 2014-2016 Swift Navigation Inc.
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
#include <libswiftnav/dgnss_management.h>
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

/** number of solution periods before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_PERIODS 2

/** number of OS ticks before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT(soln_freq_hz) MS2ST((DGNSS_TIMEOUT_PERIODS * \
  1/((float) (soln_freq_hz)) * 1000))

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
/** Minimum number of measurements to use with PVT */
#define MINIMUM_MEAS_COUNT 4

MemoryPool obs_buff_pool;
mailbox_t obs_mailbox;

dgnss_solution_mode_t dgnss_soln_mode = SOLN_MODE_LOW_LATENCY;
dgnss_filter_t dgnss_filter = FILTER_FLOAT;

/** Mutex to control access to the eigen filter. This is a very big mutex
 * that locks the entire eigen filter on access, a better method would be
 * to use mutexs as appropriate within the eigen filter code itself */
MUTEX_DECL(eigen_state_lock);

systime_t last_dgnss;

double soln_freq = 5.0;
u32 max_age_of_differential = 30;
u32 obs_output_divisor = 1;

double known_baseline[3] = {0, 0, 0};
s16 msg_obs_max_size = 102;

static last_good_fix_t lgf;

bool disable_raim = false;
bool send_heading = false;

/** Mutex to control access to the RTK filter init flag. */
MUTEX_DECL(rtk_init_done_lock);
static bool rtk_init_done = false;

static u8 old_base_sender_id = 0;

static soln_stats_t last_stats = { .signals_tracked = 0, .signals_useable = 0 };
static soln_pvt_stats_t last_pvt_stats = { .systime = -1, .signals_used = 0 };
static soln_dgnss_stats_t last_dgnss_stats = { .systime = -1, .mode = 0 };

void reset_rtk_filter(void) {
  chMtxLock(&rtk_init_done_lock);
  rtk_init_done = false;
  chMtxUnlock(&rtk_init_done_lock);
}

// Declare all SBP messages
msg_gps_time_t gps_time;
msg_pos_llh_t pos_llh;
msg_pos_ecef_t pos_ecef;
msg_vel_ned_t vel_ned;
msg_vel_ecef_t vel_ecef;
dops_t epoch_dops;
msg_baseline_ecef_t sbp_ecef;
msg_baseline_ned_t sbp_ned;
msg_baseline_heading_t sbp_heading;
msg_nmea_gga nmea_gga;

/** Determine if we have had a DGNSS timeout.
 *
 * \param _last_dgnss. Last time of DGNSS solution
 * \param _soln_freq. The current solution rate (in hz) commanded for soln thread
 * \param _dgnss_soln_mode.  Enumeration of the DGNSS solution mode
 *
 */
bool dgnss_timeout(systime_t _last_dgnss, double _soln_freq_hz,
                            dgnss_solution_mode_t _dgnss_soln_mode) {
  double dgnss_timeout_sec = DGNSS_TIMEOUT(_soln_freq_hz);

  if (_dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
    dgnss_timeout_sec *= 2;
  }
  return (chVTTimeElapsedSinceX(_last_dgnss) > dgnss_timeout_sec);
}

void solution_make_sbp(gnss_solution *soln, dops_t *dops, bool clock_jump) {
  if (soln) {
    /* Send GPS_TIME message first. */
    sbp_make_gps_time(&gps_time, &soln->time, 0);

    sbp_make_pos_llh(&pos_llh, soln, 0);

    /* Position in ECEF. */
    sbp_make_pos_ecef(&pos_ecef, soln, 0);

    /* Velocity in NED. */
    /* Do not send if there has been a clock jump. Velocity may be unreliable.*/
    if (!clock_jump) {
      sbp_make_vel_ned(&vel_ned, soln, 0);

      /* Velocity in ECEF. */
      sbp_make_vel_ecef(&vel_ecef, soln, 0);
    }

    // DOP message can be sent even if solution fails to compute
    if (dops) {
      epoch_dops = *dops;
    }

    /* Update stats */
    last_pvt_stats.systime = chVTGetSystemTime();
    last_pvt_stats.signals_used = soln->n_sigs_used;
  }
}

void solution_send_sbp(void) {
  sbp_send_msg(SBP_MSG_GPS_TIME, sizeof(gps_time), (u8 *) &gps_time);
  sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);
  sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *) &pos_ecef);
  sbp_send_msg(SBP_MSG_VEL_NED, sizeof(vel_ned), (u8 *) &vel_ned);
  sbp_send_msg(SBP_MSG_VEL_ECEF, sizeof(vel_ecef), (u8 *) &vel_ecef);


  sbp_send_msg(SBP_MSG_BASELINE_ECEF, sizeof(sbp_ecef), (u8 *)&sbp_ecef);
  sbp_send_msg(SBP_MSG_BASELINE_NED, sizeof(sbp_ned), (u8 *)&sbp_ned);
  sbp_send_msg(SBP_MSG_BASELINE_HEADING, sizeof(sbp_heading), (u8 *)&sbp_heading);
  sbp_send_msg(SBP_MSG_POS_LLH, sizeof(pos_llh), (u8 *) &pos_llh);
  sbp_send_msg(SBP_MSG_POS_ECEF, sizeof(pos_ecef), (u8 *) &pos_ecef);

  msg_dops_t sbp_dops;
  sbp_make_dops(&sbp_dops,&epoch_dops,pos_ecef.tow, pos_llh.flags);
  sbp_send_msg(SBP_MSG_DOPS, sizeof(sbp_dops), (u8 *) &sbp_dops);

  // To send nmea, we want to reconstruct the gnss_solution_t from the best
  // positions calculated
  

  u8 fix_mode = (pos_llh.flags & 1) ? NMEA_GGA_FIX_RTK : NMEA_GGA_FIX_FLOAT;
  /* TODO: Don't fake DOP!! */
  solution_send_nmea(&nmea_gga, soln, epoch_dops, num_sats, nav_meas, true);

}

void solution_send_nmea(msg_nmea_gga *nmea_gga, gnss_solution *soln, dops_t *dops,
                        u8 n, navigation_measurement_t *nm, bool velocity_valid)
{
  nmea_send_msgs(nmea_gga, soln, n, nm, dops, velocity_valid);
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
 * \note this function relies upon the global base_pos_ecef and base_pos_known
 * for logic and base station position when sending psuedo absolutes.
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
void solution_make_baseline(const gps_time_t *t, u8 n_sats, double b_ecef[3],
                            double covariance_ecef[9],
                            double ref_ecef[3], u8 flags, double hdop,
                            double corrections_age, u16 sender_id)
{
  double b_ned[3];
  wgsecef2ned(b_ecef, ref_ecef, b_ned);

  double accuracy, h_accuracy, v_accuracy;
  covariance_to_accuracy(covariance_ecef, ref_ecef, &accuracy, &h_accuracy, &v_accuracy);

  double* base_station_pos;
  sbp_make_baseline_ecef(&sbp_ecef, t, n_sats, b_ecef, accuracy, flags);

  sbp_make_baseline_ned(&sbp_ned, t, n_sats, b_ned, h_accuracy, v_accuracy, flags);

  if (send_heading) {
    double heading = calc_heading(b_ned);
    sbp_make_heading(&sbp_heading, t, heading, n_sats, flags);
  }

  chMtxLock(&base_pos_lock);
  if (base_pos_known || (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK))) {
    last_dgnss = chVTGetSystemTime();
    double pseudo_absolute_ecef[3];
    double pseudo_absolute_llh[3];
    /* if simulation use the simulator's base station position */
    if ((simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
        simulation_enabled_for(SIMULATION_MODE_RTK))) {
      base_station_pos = simulation_ref_ecef();
    }
    else { /* else use the global variable */
      base_station_pos = base_pos_ecef;
    }

    vector_add(3, base_station_pos, b_ecef, pseudo_absolute_ecef);
    wgsecef2llh(pseudo_absolute_ecef, pseudo_absolute_llh);
    u8 fix_mode = (flags == FIXED_POSITION) ? NMEA_GGA_FIX_RTK : NMEA_GGA_FIX_FLOAT;
    nmea_make_gpgga(&nmea_gga, pseudo_absolute_llh, t, n_sats, fix_mode, hdop, corrections_age, sender_id);

    /* now send pseudo absolute sbp message */
    /* Flag in message is defined as follows :float->2, fixed->1 */
    /* We defined the flags for the SBP protocol to be spp->0, fixed->1, float->2 */
    /* TODO: Define these flags from the yaml and remove hardcoding */
    u8 sbp_flags = (flags == 1) ? 1 : 2;
    sbp_make_pos_llh_vect(&pos_llh, pseudo_absolute_llh, h_accuracy, v_accuracy, t, n_sats, sbp_flags);
    sbp_make_pos_ecef_vect(&pos_ecef, pseudo_absolute_ecef, accuracy, t, n_sats, sbp_flags);
  }
  chMtxUnlock(&base_pos_lock);

  /* Update stats */
  last_dgnss_stats.systime = chVTGetSystemTime();
  last_dgnss_stats.mode = (flags == FIXED_POSITION) ? FILTER_FIXED : FILTER_FLOAT;
}

static void output_baseline(u8 num_sdiffs, const sdiff_t *sdiffs,
                            const gps_time_t *t, double hdop, double diff_time, u16 base_id) {
  double baseline[3];
  double covariance[9];
  u8 num_sats_used = 0;
  u8 num_sigs_used = 0;
  u8 flags = 0;
  s8 ret;
  bool send_baseline = false;

  /* If not initialised we can't output a baseline */
  chMtxLock(&rtk_init_done_lock);
  if (rtk_init_done) {
    if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED) {
      log_debug("solution time matched");
      /* Filter is already updated so no need to update filter again just get the baseline*/
      chMtxLock(&eigen_state_lock);
      ret = get_baseline(baseline, covariance, &num_sats_used, &num_sigs_used, &flags);
      chMtxUnlock(&eigen_state_lock);
      if (ret != 0) {
        log_warn("output_baseline: Time matched baseline calculation failed");
      } else {
        send_baseline = true;
      }
    } else {
      log_debug("solution low latency");
      /* Need to update filter with propogated obs before we can get the baseline */
      chMtxLock(&eigen_state_lock);
      ret = dgnss_update_v3(t, num_sdiffs, sdiffs, lgf.position_solution.pos_ecef,
                      base_pos_known ? base_pos_ecef : NULL, diff_time);
      chMtxUnlock(&eigen_state_lock);
      if (ret == 0) {
        chMtxLock(&eigen_state_lock);
        ret = get_baseline(baseline, covariance, &num_sats_used, &num_sigs_used, &flags);
        chMtxUnlock(&eigen_state_lock);
        if (ret != 0) {
          log_warn("output_baseline: Low latency baseline calculation failed");
        } else {
          send_baseline = true;
        }
      }
    }

    if (send_baseline)
    {
      solution_make_baseline(t, num_sats_used, baseline, covariance,
                             lgf.position_solution.pos_ecef,
                             flags, hdop, diff_time, base_id);
    }
  } else {
    log_debug("DGNSS Filter not Initialized");
  }
  chMtxUnlock(&rtk_init_done_lock);
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
                              const gps_time_t *t)
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

static void solution_simulation(void)
{
  simulation_step();

  /* TODO: The simulator's handling of time is a bit crazy. This is a hack
   * for now but the simulator should be refactored so that it can give the
   * exact correct solution time output without this nonsense. */
  gnss_solution *soln = simulation_current_gnss_solution();
  double expected_tow = \
    round(soln->time.tow * soln_freq) / soln_freq;
  soln->time.tow = expected_tow;
  normalize_gps_time(&soln->time);

  if (simulation_enabled_for(SIMULATION_MODE_PVT)) {
    /* Then we send fake messages. */
    solution_make_sbp(soln, simulation_current_dops_solution(), FALSE);
    solution_send_nmea(soln, simulation_current_dops_solution(),
                       simulation_current_num_sats(),
                       simulation_current_navigation_measurements(),
                       NMEA_GGA_FIX_GPS, FALSE);

  }

  if (simulation_enabled_for(SIMULATION_MODE_FLOAT) ||
      simulation_enabled_for(SIMULATION_MODE_RTK)) {

    u8 flags = simulation_enabled_for(SIMULATION_MODE_RTK) ? FIXED_POSITION : FLOAT_POSITION;

    solution_send_baseline(&(soln->time),
      simulation_current_num_sats(),
      simulation_current_baseline_ecef(),
      simulation_current_covariance_ecef(),
      simulation_ref_ecef(), flags, 1.5, 0.25, 1023);

    double t_check = expected_tow * (soln_freq / obs_output_divisor);
    if (fabs(t_check - (u32)t_check) < TIME_MATCH_THRESHOLD) {
      send_observations(simulation_current_num_sats(),
          simulation_current_navigation_measurements(), &(soln->time));
    }
  }
}

/** Update the tracking channel states with satellite elevation angles
 * \param nav_meas Navigation measurements with .sat_pos populated
 * \param n_meas Number of navigation measurements
 * \param rcv_pos_ecef Receiver position
 */
static void update_sat_elevations(const navigation_measurement_t nav_meas[],
                                  u8 n_meas, const double rcv_pos_ecef[3])
{
  double _, el;
  for (int i = 0; i < n_meas; i++) {
    wgsecef2azel(nav_meas[i].sat_pos, rcv_pos_ecef, &_, &el);
    tracking_channel_elevation_degrees_set(nav_meas[i].sid, (float)el * R2D);
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
 * Counts SV with required flags.
 *
 * The method counts SV whose measurements contain required flags.
 *
 * \param[in] n_ready Total count of input entries.
 * \param[in] meas    Channel measurements.
 * \param[in] flags   Required flags.
 *
 * \return Number of SV whose measurements have flags.
 */
static u32 count_meas_with_accuracy(u8 n_ready,
                                    const channel_measurement_t meas[],
                                    chan_meas_flags_t flags)
{
  gnss_sid_set_t codes_in_track;

  sid_set_init(&codes_in_track);
  for (u8 i = 0; i < n_ready; i++) {
    if (flags == (meas[i].flags & flags)) {
      sid_set_add(&codes_in_track, meas[i].sid);
    }
  }

  return sid_set_get_sat_count(&codes_in_track);
}

/**
 * The method excludes measurements that might decrease accuracy.
 *
 * The method selects the lowest measurement accuracy requirement, for which
 * at least 4 SVs are available.
 *
 * \param[in]     n_ready Number of available measurements.
 * \param[in,out] meas    Measurements data vector.
 *
 * \return Number of available measurements.
 */
static u8 filter_out_measurements(u8 n_ready, channel_measurement_t meas[])
{
  static const chan_meas_flags_t flags[] = {
    /* High phase accuracy only (high code accuracy implied) */
    CHAN_MEAS_FLAG_PHASE_VALID | CHAN_MEAS_FLAG_HALF_CYCLE_KNOWN |
    CHAN_MEAS_FLAG_CODE_VALID | CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID,
    /* Some phase accuracy  (high code accuracy implied) */
    CHAN_MEAS_FLAG_PHASE_VALID | CHAN_MEAS_FLAG_CODE_VALID |
    CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID,
    /* Any phase accuracy, high code accuracy */
    CHAN_MEAS_FLAG_CODE_VALID | CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID,
    /* Any phase accuracy, high or low code accuracy */
    CHAN_MEAS_FLAG_CODE_VALID,
  };

  /* Go though criteria vector from the most strict till the least strict, and
   * count the individual SVs that match the criteria.
   *
   * Matching for the upper criteria also means that lower criteria is met.
   *
   * As long as there is no measurement weighting in PVT, try to drop the least
   * accurate measurements, as long as total number of SVs is above a threshold.
   *
   * TODO Instead of filtering out measurements, implement weight computation
   *      and support in PVT.
   */
  u8 idx  = 0;
  for (idx = 0;
       idx < sizeof(flags) / sizeof(flags[0]) &&
       count_meas_with_accuracy(n_ready,
                                meas,
                                flags[idx]) < MINIMUM_MEAS_COUNT;
       ++idx) {
    /* Noop */
  }

  if (idx == sizeof(flags) / sizeof(flags[0])) {
    /* Not found */
    n_ready = 0;
  } else {
    chan_meas_flags_t requred_flags = flags[idx];

    /* Ignore measurements with insufficient accuracy for now
     *
     * TODO change the accuracy filtering into algorithm of processing
     *      observation weights */
    for (u8 i = 0; i < n_ready; ) {
      if (requred_flags != (meas[i].flags & requred_flags)) {
        /* This measurement can't be used */
        meas[i] = meas[n_ready - 1];
        --n_ready;
      } else {
        ++i;
      }
    }
  }

  return n_ready;
}

/**
 * Collects channel measurements and auxilary data.
 *
 * \param[in]  rec_tc    Timestamp [samples]
 * \param[out] meas      Destination measurement array.
 * \param[out] pn_ready  Destination for measurement array size.
 * \param[out] pn_total  Destination for total active trackers count.
 *
 * \return None
 */
static void collect_measurements(u64 rec_tc,
                                 channel_measurement_t meas[MAX_CHANNELS],
                                 u8 *pn_ready,
                                 u8 *pn_total)
{
  u8 n_collected = 0;
  u8 n_active = 0;

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    manage_track_flags_t flags      = 0; /* Channel flags accumulator */
    flags = get_tracking_channel_meas(i, rec_tc, &meas[n_collected]);

    if (0 != (flags & MANAGE_TRACK_FLAG_ACTIVE) &&
        0 != (flags & MANAGE_TRACK_FLAG_CONFIRMED) &&
        0 != (flags & MANAGE_TRACK_FLAG_NO_ERROR))
    {
      n_active++;
      if (0 != (flags & MANAGE_TRACK_FLAG_HEALTHY) &&
          0 != (flags & MANAGE_TRACK_FLAG_NAV_SUITABLE) &&
          0 != (flags & MANAGE_TRACK_FLAG_ELEVATION) &&
          0 != (flags & MANAGE_TRACK_FLAG_TOW) &&
          0 != (flags & MANAGE_TRACK_FLAG_HAS_EPHE) &&
          0 != (flags & MANAGE_TRACK_FLAG_CN0_SHORT) &&
          0 != meas[n_collected].flags) {
        n_collected++;
      }
    }
  }

  *pn_ready = n_collected;
  *pn_total = n_active;
}

static THD_WORKING_AREA(wa_solution_thread, 2000000);
static void solution_thread(void *arg)
{
  /* The flag is true when we have a solution */
  bool soln_flag = false;

  (void)arg;
  chRegSetThreadName("solution");

  systime_t deadline = chVTGetSystemTimeX();

  bool clock_jump = FALSE;

  ndb_lgf_read(&lgf);

  while (TRUE) {

    sol_thd_sleep(&deadline, CH_CFG_ST_FREQUENCY/soln_freq);
    watchdog_notify(WD_NOTIFY_SOLUTION);

    // Init the messages we want to send
    memset(&gps_time, 0, sizeof(msg_gps_time_t));
    memset(&pos_llh, 0, sizeof(msg_pos_llh_t));
    memset(&pos_ecef, 0, sizeof(msg_pos_ecef_t));
    memset(&vel_ned, 0, sizeof(msg_vel_ned_t));
    memset(&vel_ecef, 0, sizeof(msg_vel_ecef_t));
    memset(&epoch_dops, 0, sizeof(dops_t));
    memset(&sbp_ecef, 0, sizeof(msg_baseline_ecef_t));
    memset(&sbp_ned, 0, sizeof(msg_baseline_ned_t));
    memset(&sbp_heading, 0, sizeof(msg_baseline_heading_t));
    memset(&pos_llh, 0, sizeof(msg_pos_llh_t));
    memset(&pos_ecef, 0, sizeof(msg_pos_ecef_t));
    memset(&nmea_gga, 0, sizeof(msg_nmea_gga));q


    /* Here we do all the nice simulation-related stuff. */
    if (simulation_enabled()) {
      solution_simulation();
    }

    u64 rec_tc = nap_timing_count();
    gps_time_t rec_time = rx2gpstime(rec_tc);
    u8 n_collected = 0;
    u8 n_total = 0;
    channel_measurement_t meas[MAX_CHANNELS];

    /* Collect measurements from trackers */
    collect_measurements(rec_tc, meas, &n_collected, &n_total);

    u8 n_ready = n_collected;
    if (n_collected >= MINIMUM_MEAS_COUNT) {
      /* Select best measurements. */
      n_ready = filter_out_measurements(n_collected, meas);
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

    if (n_ready < MINIMUM_MEAS_COUNT) {
      /* Not enough sats, keep on looping. */

      /* TODO if there are not enough SVs to compute PVT, shouldn't caches
       *      below be reset? I.e. nav_meas_old and nav_meas_tdcp? */

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
    static ephemeris_t e_meas[MAX_CHANNELS];
    const ephemeris_t *p_e_meas[n_ready];

    /* Create arrays of pointers for use in calc_navigation_measurement
     *
     * TODO Ephemeris are already loaded once when extended flags are computed.
     *      NDB access can be optimized out.
     */
    for (u8 i = 0; i < n_ready; i++) {
      p_meas[i] = &meas[i];
      p_nav_meas[i] = &nav_meas[i];
      ndb_ephemeris_read(meas[i].sid, &e_meas[i]);
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
      continue;
    }

    s8 sc_ret = calc_sat_clock_corrections(n_ready, p_nav_meas, p_e_meas);

    if (sc_ret != 0) {
       log_error("calc_sat_clock_correction() returned an error");
       continue;
     }

    calc_isc(n_ready, p_nav_meas, p_cnav_30);

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
      continue;
    }

    /* check if we have a solution, if yes calc iono and tropo correction */
    if (soln_flag) {
      ionosphere_t i_params;
      ionosphere_t *p_i_params = &i_params;
      /* get iono parameters if available */
      chMtxLock(&rtk_init_done_lock);
      if(ndb_iono_corr_read(p_i_params) != NDB_ERR_NONE) {
        p_i_params = NULL;
      } else if (rtk_init_done) {
        chMtxLock(&eigen_state_lock);
        dgnss_update_iono_parameters(p_i_params);
        chMtxUnlock(&eigen_state_lock);
      }
      chMtxUnlock(&rtk_init_done_lock);
      calc_iono_tropo(n_ready_tdcp, nav_meas_tdcp,
                      lgf.position_solution.pos_ecef,
                      lgf.position_solution.pos_llh,
                      p_i_params);
    }

    dops_t dops;
    /* Calculate the SPP position
     * disable_raim controlled by external setting. Defaults to false. */
    /* Don't skip velocity solving. If there is a cycle slip, tdcp_doppler will
     * just return the rough value from the tracking loop. */
     // TODO(Leith) check velocity_valid
    s8 pvt_ret = calc_PVT(n_ready_tdcp, nav_meas_tdcp, disable_raim, false,
                          (double) get_elevation_mask(),
                          &lgf.position_solution, &dops);
    if (pvt_ret < 0) {
      /* An error occurred with calc_PVT! */
      /* pvt_err_msg defined in libswiftnav/pvt.c */
      log_warn("PVT solver: %s (code %d)", pvt_err_msg[-pvt_ret-1], pvt_ret);

      soln_flag = false;

      // If we can't compute a SPP position, something is wrong and no point
      // continuing to process this epoch
      continue;
    }

    soln_flag = true;

    if (pvt_ret == 1) {
      log_warn("calc_PVT: RAIM repair");
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
      set_time_fine(rec_tc, lgf.position_solution.time);
      clock_jump = TRUE;
      continue;
    }
    set_gps_time_offset(rec_tc, lgf.position_solution.time);

    /* Update global position solution state. */
    ndb_lgf_store(&lgf);

    /* Save elevation angles every so often */
    update_sat_elevations(nav_meas_tdcp, n_ready_tdcp,
                          lgf.position_solution.pos_ecef);

    if (!simulation_enabled()) {
      /* Output solution. */

      bool disable_velocity = clock_jump ||
                              (lgf.position_solution.velocity_valid == 0);
      solution_make_sbp(&lgf.position_solution, &dops, disable_velocity);
      solution_send_nmea(&lgf.position_solution, &dops,
                         n_ready_tdcp, nav_meas_tdcp,
                         NMEA_GGA_FIX_GPS, disable_velocity);
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
    new_obs_time.tow = round(lgf.position_solution.time.tow * soln_freq)
                              / soln_freq;
    normalize_gps_time(&new_obs_time);
    gps_time_match_weeks(&new_obs_time, &lgf.position_solution.time);

    double t_err = gpsdifftime(&new_obs_time, &lgf.position_solution.time);

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
        nm->raw_carrier_phase += lgf.position_solution.clock_offset *
                                      GPS_C / code_to_lambda(nm->sid.code);
        nm->raw_pseudorange -= lgf.position_solution.clock_offset * GPS_C;

        /* Also apply the time correction to the time of transmission so the
         * satellite positions can be calculated for the correct time. */
        nm->tot.tow += t_err;
        normalize_gps_time(&nm->tot);

        ephemeris_t ephe;
        ndb_ephemeris_read(nm->sid, &ephe);
        u8 eph_valid;
        s8 ss_ret;
        double clock_rate_err;

        eph_valid = ephemeris_valid(&ephe, &nm->tot);
        if (eph_valid) {
          ss_ret = calc_sat_state(&ephe, &nm->tot, nm->sat_pos, nm->sat_vel,
                                  &nm->sat_clock_err, &clock_rate_err);
        }

        if (!eph_valid || (ss_ret != 0)) {
          continue;
        }

        n_ready_tdcp_new++;
      }

      /* Update n_ready_tdcp. */
      n_ready_tdcp = n_ready_tdcp_new;

      /* If we have a recent set of observations from the base station, do a
       * differential solution. */
      double pdt;
      chMtxLock(&base_obs_lock);
      if (base_obss.n > 0 && !simulation_enabled()) {
        if ((pdt = gpsdifftime(&new_obs_time, &base_obss.tor))
              < max_age_of_differential) {

          /* Propagate base station observations to the current time and
           * process a low-latency differential solution. */

          /* Hook in low-latency filter here. */
          if (dgnss_soln_mode == SOLN_MODE_LOW_LATENCY &&
              base_obss.has_pos) {

            sdiff_t sdiffs[MAX(base_obss.n, n_ready_tdcp)];
            u8 num_sdiffs = make_propagated_sdiffs(n_ready_tdcp, nav_meas_tdcp,
                                    base_obss.n, base_obss.nm,
                                    base_obss.sat_dists, base_obss.pos_ecef,
                                    sdiffs);
            if (num_sdiffs >= 4) {
              output_baseline(num_sdiffs, sdiffs, &lgf.position_solution.time,
                              dops.hdop, pdt, base_obss.sender_id);
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
        post_observations(n_ready_tdcp, nav_meas_tdcp, &new_obs_time);
        /* Send the observations. */
        send_observations(n_ready_tdcp, nav_meas_tdcp, &new_obs_time);
      }
    }

    /* Calculate the receiver clock error and if >1ms perform a clock jump */
    double rx_err = gpsdifftime(&rec_time, &lgf.position_solution.time);
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
    }

    /* Calculate time till the next desired solution epoch. */
    double dt = gpsdifftime(&new_obs_time, &lgf.position_solution.time);

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

void process_matched_obs(u8 n_sds, gps_time_t *t, sdiff_t *sds, u16 base_id)
{
  chMtxLock(&rtk_init_done_lock);
  if (!rtk_init_done) {
    if (n_sds > 4) {
      /* Initialize filters. */
      log_info("Initializing DGNSS filters");
      chMtxLock(&eigen_state_lock);
      dgnss_init_v3();
      chMtxUnlock(&eigen_state_lock);
      rtk_init_done = true;
    }
  }
  chMtxUnlock(&rtk_init_done_lock);

  chMtxLock(&rtk_init_done_lock);
  if (rtk_init_done) {
    /* Update filters. */
    s8 ret;
    chMtxLock(&eigen_state_lock);
    ret = dgnss_update_v3(t, n_sds, sds, lgf.position_solution.pos_ecef,
                    base_pos_known ? base_pos_ecef : NULL, 0.0);
    chMtxUnlock(&eigen_state_lock);

    /* If we are in time matched mode then calculate and output the baseline
     * for this observation. */
    if (dgnss_soln_mode == SOLN_MODE_TIME_MATCHED &&
        !simulation_enabled() && n_sds >= 4 && ret == 0) {
      /* Note: in time match mode we send the physically incorrect time of the
       * observation message (which can be receiver clock time, or rounded GPS
       * time) instead of the true GPS time of the solution. */
      output_baseline(n_sds, sds, t, 0, 0, base_id);
    }
  }
  chMtxUnlock(&rtk_init_done_lock);
}

static WORKING_AREA_CCM(wa_time_matched_obs_thread, 2000000);
static void time_matched_obs_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("time matched obs");
  while (1) {
    /* Wait for a new observation to arrive from the base station. */
    chBSemWait(&base_obs_received);

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

      chMtxLock(&base_obs_lock);
      double dt = gpsdifftime(&obss->tor, &base_obss.tor);

      if (fabs(dt) < TIME_MATCH_THRESHOLD) {
        /* Check if the base sender ID has changed and reset the RTK filter if
         * it has.
         */
        if ((old_base_sender_id != 0) &&
            (old_base_sender_id != base_obss.sender_id)) {
          log_warn("Base station sender ID changed from %u to %u. Resetting RTK"
                   " filter.", old_base_sender_id, base_obss.sender_id);
          reset_rtk_filter();
          chMtxLock(&base_pos_lock);
          base_pos_known = false;
          memset(&base_pos_ecef, 0, sizeof(base_pos_ecef));
          chMtxUnlock(&base_pos_lock);
        }
        old_base_sender_id = base_obss.sender_id;

        /* Times match! Process obs and base_obss */
        static sdiff_t sds[MAX_CHANNELS];
        u8 n_sds = single_diff(
            obss->n, obss->nm,
            base_obss.n, base_obss.nm,
            sds
        );
        chMtxUnlock(&base_obs_lock);

        process_matched_obs(n_sds, &obss->tor, sds, base_obss.sender_id);
        chPoolFree(&obs_buff_pool, obss);
        break;
      } else {
        chMtxUnlock(&base_obs_lock);
        if (dt > 0) {
          /* Time of base obs before time of local obs, we must not have a local
           * observation matching this base observation, break and wait for a
           * new base observation. */

          /* In practice this should basically never happen so lets make a note
           * if it does. */
          log_warn("Obs Matching: t_base < t_rover "
                   "(dt=%f obss.t={%d,%f} base_obss.t={%d,%f})", dt,
                   obss->tor.wn, obss->tor.tow,
                   base_obss.tor.wn, base_obss.tor.tow
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

void solution_setup()
{
  /* Set time of last differential solution in the past. */
  last_dgnss = chVTGetSystemTime() - DGNSS_TIMEOUT(soln_freq);
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
                    sizeof(wa_time_matched_obs_thread), LOWPRIO,
                    time_matched_obs_thread, NULL);

  static sbp_msg_callbacks_node_t reset_filters_node;
  sbp_register_cbk(
    SBP_MSG_RESET_FILTERS,
    &reset_filters_callback,
    &reset_filters_node
  );
}
