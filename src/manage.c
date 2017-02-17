/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include <ch.h>

#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/constants.h>

#include "main.h"
#include "board/nap/track_channel.h"
#include "board/acq.h"
#include "ephemeris.h"
#include "track.h"
#include "decode.h"
#include "timing.h"
#include "position.h"
#include "manage.h"
#include "nmea.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "./system_monitor.h"
#include "settings.h"
#include "signal.h"
#include "ndb.h"
#include "shm.h"
#include "dum.h"
#include "reacq/reacq_api.h"

/** \defgroup manage Manage
 * Manage acquisition and tracking.
 * Manage how acquisition searches are performed, with data from almanac if
 * available. Transition from acquisition search to initialization of an
 * available tracking channel when a satellite is successfully found. Disable
 * tracking channels that have lost lock on their satellites.
 * \{ */

/** Supported channel drop reasons */
typedef enum {
  CH_DROP_REASON_ERROR,         /**< Tracking channel error */
  CH_DROP_REASON_MASKED,        /**< Tracking channel is disabled by mask */
  CH_DROP_REASON_NO_BIT_SYNC,   /**< Bit sync timeout */
  CH_DROP_REASON_NO_PLOCK,      /**< Pessimistic lock timeout */
  CH_DROP_REASON_LOW_CN0,       /**< Low C/N0 for too long */
  CH_DROP_REASON_XCORR,         /**< Confirmed cross-correlation */
  CH_DROP_REASON_NO_UPDATES,    /**< No tracker updates for too long */
  CH_DROP_REASON_L2CL_SYNC      /**< Drop L2CL after half-cycle ambiguity
                                     has been resolved */
} ch_drop_reason_t;

/** Different hints on satellite info to aid the acqusition */
enum acq_hint {
  ACQ_HINT_WARMSTART,  /**< Information from almanac or ephemeris */
  ACQ_HINT_PREV_ACQ,   /**< Previous successful acqusition. */
  ACQ_HINT_PREV_TRACK, /**< Previously tracked satellite. */
  ACQ_HINT_REMOTE_OBS, /**< Observation from reference station. */

  ACQ_HINT_NUM
};

/** Status of acquisition for a particular SID. */
typedef struct {
  enum {
    ACQ_PRN_SKIP = 0,
    ACQ_PRN_ACQUIRING,
    ACQ_PRN_TRACKING,
    ACQ_PRN_UNHEALTHY
  } state;                 /**< Management status of signal. */
  bool masked;             /**< Prevent acquisition. */
  u16 score[ACQ_HINT_NUM]; /**< Acquisition preference of signal. */
  float dopp_hint_low;     /**< Low bound of doppler search hint. */
  float dopp_hint_high;    /**< High bound of doppler search hint. */
  gnss_signal_t sid;       /**< Signal identifier. */
} acq_status_t;

static acq_status_t acq_status[PLATFORM_SIGNAL_COUNT];
static bool track_mask[ARRAY_SIZE(acq_status)];

#define SCORE_COLDSTART     100
#define SCORE_WARMSTART     200
#define SCORE_BELOWMASK     0
#define SCORE_ACQ           100
#define SCORE_TRACK         200
#define SCORE_OBS           200

#define DOPP_UNCERT_ALMANAC 4000
#define DOPP_UNCERT_EPHEM   500

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

#define TRACKING_STARTUP_FIFO_SIZE 8    /* Must be a power of 2 */

#define TRACKING_STARTUP_FIFO_INDEX_MASK ((TRACKING_STARTUP_FIFO_SIZE) - 1)
#define TRACKING_STARTUP_FIFO_INDEX_DIFF(write_index, read_index) \
          ((tracking_startup_fifo_index_t)((write_index) - (read_index)))
#define TRACKING_STARTUP_FIFO_LENGTH(p_fifo) \
          (TRACKING_STARTUP_FIFO_INDEX_DIFF((p_fifo)->write_index, \
                                            (p_fifo)->read_index))

typedef u8 tracking_startup_fifo_index_t;

typedef struct {
  tracking_startup_fifo_index_t read_index;
  tracking_startup_fifo_index_t write_index;
  tracking_startup_params_t elements[TRACKING_STARTUP_FIFO_SIZE];
} tracking_startup_fifo_t;

static tracking_startup_fifo_t tracking_startup_fifo;

static MUTEX_DECL(tracking_startup_mutex);

/* Elevation mask for tracking, degrees */
static float tracking_elevation_mask = 0.0;
/* Elevation mask for solution, degrees */
static float solution_elevation_mask = 10.0;

static bool sbas_enabled = false;
/** Flag if almanacs can be used in acq */
static bool almanacs_enabled = false;

static u8 manage_track_new_acq(gnss_signal_t sid);
static void manage_acq(void);
static void manage_track(void);

static void manage_tracking_startup(void);
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo);
static bool tracking_startup_fifo_write(tracking_startup_fifo_t *fifo,
                                        const tracking_startup_params_t *
                                        element);
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element);

static sbp_msg_callbacks_node_t almanac_callback_node;
static void almanac_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void)context; (void)msg;
}

static bool tracking_startup_fifo_sid_present(
                                            const tracking_startup_fifo_t *fifo,
                                            gnss_signal_t sid);


static sbp_msg_callbacks_node_t mask_sat_callback_node;
static void mask_sat_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;
  enum {
    MASK_ACQUISITION = 1,
    MASK_TRACKING = 2,
  };

  msg_mask_satellite_t *m = (msg_mask_satellite_t *)msg;
  gnss_signal_t sid = sid_from_sbp(m->sid);

  if (sid_supported(sid)) {
    u16 global_index = sid_to_global_index(sid);
    acq_status_t *acq = &acq_status[global_index];
    acq->masked = (m->mask & MASK_ACQUISITION) ? true : false;
    track_mask[global_index] = (m->mask & MASK_TRACKING) ? true : false;
    log_info_sid(sid, "Mask = 0x%02x", m->mask);
  } else {
    log_warn("Mask not set for invalid SID");
  }
}

static THD_WORKING_AREA(wa_manage_acq_thread, MANAGE_ACQ_THREAD_STACK);
static void manage_acq_thread(void *arg)
{
  /* TODO: This should be trigged by a semaphore from the acq ISR code, not
   * just ran periodically. */

  bool had_fix = false;

  (void)arg;
  chRegSetThreadName("manage acq");

  init_reacq();

  while (TRUE) {
    last_good_fix_t lgf;
    bool have_fix;

    have_fix = (ndb_lgf_read(&lgf) == NDB_ERR_NONE) &&
               lgf.position_solution.valid &&
               (POSITION_FIX == lgf.position_quality) &&
               ((TIME_COARSE <= time_quality));
    if (have_fix && !had_fix) {
      had_fix = true;
      log_info("Switching to re-acq mode");
    }

    if (had_fix) {
      manage_reacq();
    } else {
      manage_acq();
    }

    manage_tracking_startup();
    watchdog_notify(WD_NOTIFY_ACQ_MGMT);
  }
}

void manage_acq_setup()
{
  SETTING("acquisition", "sbas_enabled", sbas_enabled, TYPE_BOOL);
  SETTING("acquisition", "almanacs_enabled", almanacs_enabled, TYPE_BOOL);

  tracking_startup_fifo_init(&tracking_startup_fifo);

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    gnss_signal_t sid = sid_from_global_index(i);
    acq_status[i].state = ACQ_PRN_ACQUIRING;
    acq_status[i].masked = false;
    memset(&acq_status[i].score, 0, sizeof(acq_status[i].score));

    if (code_requires_direct_acq(sid.code)) {
      acq_status[i].dopp_hint_low = code_to_sv_doppler_min(sid.code) +
                                    code_to_tcxo_doppler_min(sid.code);
      acq_status[i].dopp_hint_high = code_to_sv_doppler_max(sid.code) +
                                     code_to_tcxo_doppler_max(sid.code);
    }
    acq_status[i].sid = sid;

    track_mask[i] = false;

    if (!sbas_enabled && (sid_to_constellation(sid) == CONSTELLATION_SBAS)) {
      acq_status[i].masked = true;
      track_mask[i] = true;
    }
  }

  sbp_register_cbk(
    SBP_MSG_ALMANAC,
    &almanac_callback,
    &almanac_callback_node
  );

  sbp_register_cbk(
    SBP_MSG_MASK_SATELLITE,
    &mask_sat_callback,
    &mask_sat_callback_node
  );

  chThdCreateStatic(
      wa_manage_acq_thread,
      sizeof(wa_manage_acq_thread),
      MANAGE_ACQ_THREAD_PRIORITY,
      manage_acq_thread, NULL
  );
}


/** Using available almanac and ephemeris information, determine
 * whether a satellite is in view and the range of doppler frequencies
 * in which we expect to find it.
 *
 * \param prn 0-indexed PRN
 * \param t Time at which to evaluate ephemeris and almanac (typically system's
 *  estimate of current time)
 * \param dopp_hint_low, dopp_hint_high Pointers to store doppler search range
 *  from ephemeris or almanac, if available and elevation > mask
 * \return Score (higher is better)
 */
static u16 manage_warm_start(gnss_signal_t sid, const gps_time_t* t,
                             float *dopp_hint_low, float *dopp_hint_high)
{
    /* Do we have any idea where/when we are?  If not, no score. */
    /* TODO: Stricter requirement on time and position uncertainty?
       We ought to keep track of a quantitative uncertainty estimate. */
  last_good_fix_t lgf;
  if(ndb_lgf_read(&lgf) != NDB_ERR_NONE ||
      lgf.position_quality < POSITION_GUESS ||
      time_quality < TIME_GUESS) {
    return SCORE_COLDSTART;
  }

  float el = sv_elevation_degrees_get(sid);
  if (el < tracking_elevation_mask) {
    return SCORE_BELOWMASK;
  }

  double _, dopp_hint = 0, dopp_uncertainty = DOPP_UNCERT_ALMANAC;
  bool ready = false;
  /* Do we have a suitable ephemeris for this sat?  If so, use
     that in preference to the almanac. */
  union { ephemeris_t e; almanac_t a; } orbit;
  ndb_ephemeris_read(sid, &orbit.e);
  u8 eph_valid;
  s8 ss_ret;
  double sat_pos[3], sat_vel[3], el_d;

  eph_valid = ephemeris_valid(&orbit.e, t);
  if (eph_valid) {
    ss_ret = calc_sat_state(&orbit.e, t, sat_pos, sat_vel, &_, &_);
  }

  if (eph_valid && (ss_ret == 0)) {
    double dopp_hint_sat_vel; /* Doppler hint induced by sat velocity */
    double dopp_hint_clock;   /* Doppler hint induced by clock drift */
    vector_subtract(3, sat_pos, lgf.position_solution.pos_ecef, sat_pos);
    vector_normalize(3, sat_pos);
    /* sat_pos now holds unit vector from us to satellite */
    vector_subtract(3, sat_vel, lgf.position_solution.vel_ecef, sat_vel);
    /* sat_vel now holds velocity of sat relative to us */
    dopp_hint_sat_vel = -GPS_L1_HZ * vector_dot(3, sat_pos, sat_vel) / GPS_C;
    /* TODO: Check sign of receiver frequency offset correction.
             There seems to be a sign flip somewhere in 'clock_bias'
             computation that gets compensated here */
    dopp_hint_clock = -GPS_L1_HZ * lgf.position_solution.clock_bias;
    dopp_hint = dopp_hint_sat_vel + dopp_hint_clock;
    if (time_quality >= TIME_FINE) {
      dopp_uncertainty = DOPP_UNCERT_EPHEM;
    }
    ready = true;

    if ((dopp_hint_sat_vel < code_to_sv_doppler_min(sid.code)) ||
        (dopp_hint_sat_vel > code_to_sv_doppler_max(sid.code)) ||
        (dopp_hint_clock < code_to_tcxo_doppler_min(sid.code)) ||
        (dopp_hint_clock > code_to_tcxo_doppler_max(sid.code))) {
      log_error_sid(sid,
                    "Acq: bogus ephe/clock dopp hints "
                    "(unc,sat_hint,clk_hint,lgf_pos[0..2],drift,ele) "
                    "(%.1lf,%.1lf,%.1lf,[%.1lf,%.1lf,%.1lf],%g,%.1f)",
                    dopp_uncertainty,
                    dopp_hint_sat_vel,
                    dopp_hint_clock,
                    lgf.position_solution.pos_ecef[0],
                    lgf.position_solution.pos_ecef[1],
                    lgf.position_solution.pos_ecef[2],
                    lgf.position_solution.clock_bias,
                    el);
      return SCORE_COLDSTART;
    }
  }

  float doppler_min = code_to_sv_doppler_min(sid.code) +
                      code_to_tcxo_doppler_min(sid.code);
  float doppler_max = code_to_sv_doppler_max(sid.code) +
                      code_to_tcxo_doppler_max(sid.code);

  if(!ready) {
    if (almanacs_enabled &&
        NDB_ERR_NONE == ndb_almanac_read(sid, &orbit.a) &&
        almanac_valid(&orbit.a, t) &&
        calc_sat_az_el_almanac(&orbit.a, t, lgf.position_solution.pos_ecef,
                               &_, &el_d) == 0) {
      el = (float)(el_d * R2D);
      if (el < tracking_elevation_mask) {
        return SCORE_BELOWMASK;
      }
      if (calc_sat_doppler_almanac(&orbit.a, t, lgf.position_solution.pos_ecef,
                                   &dopp_hint) != 0) {
        return SCORE_COLDSTART;
      }
      dopp_hint = -dopp_hint;

      if ((dopp_hint < doppler_min) || (dopp_hint > doppler_max)) {
        log_error_sid(sid,
                      "Acq: bogus alm dopp_hint "
                      "(unc,sat_hint,lgf_pos[0..2],ele) "
                      "(%.1lf,%.1lf,[%.1lf,%.1lf,%.1lf],%.1f)",
                      dopp_uncertainty,
                      dopp_hint,
                      lgf.position_solution.pos_ecef[0],
                      lgf.position_solution.pos_ecef[1],
                      lgf.position_solution.pos_ecef[2],
                      el);
        return SCORE_COLDSTART;
      }
    } else {
      return SCORE_COLDSTART; /* Couldn't determine satellite state. */
    }
  }

  /* Return the doppler hints and a score proportional to elevation */
  *dopp_hint_low = MAX(dopp_hint - dopp_uncertainty, doppler_min);
  *dopp_hint_high = MIN(dopp_hint + dopp_uncertainty, doppler_max);
  return SCORE_COLDSTART + SCORE_WARMSTART * el / 90.f;
}

static acq_status_t * choose_acq_sat(void)
{
  u32 total_score = 0;
  gps_time_t t = get_current_time();

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (!code_requires_direct_acq(acq_status[i].sid.code)) {
      continue;
    }

    if ((acq_status[i].state != ACQ_PRN_ACQUIRING) ||
        acq_status[i].masked)
      continue;

    acq_status[i].score[ACQ_HINT_WARMSTART] =
      manage_warm_start(acq_status[i].sid, &t,
                        &acq_status[i].dopp_hint_low,
                        &acq_status[i].dopp_hint_high);

    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++) {
      total_score += acq_status[i].score[hint];
    }
  }

  if (total_score == 0) {
    log_error("Failed to pick a sat for acquisition!");
    return NULL;
  }

  u32 pick = rand() % total_score;

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (!code_requires_direct_acq(acq_status[i].sid.code)) {
      continue;
    }

    if ((acq_status[i].state != ACQ_PRN_ACQUIRING) ||
        acq_status[i].masked)
      continue;

    u32 sat_score = 0;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++)
      sat_score += acq_status[i].score[hint];
    if (pick < sat_score) {
      return &acq_status[i];
    } else {
      pick -= sat_score;
    }
  }

  assert(!"Error picking a sat for acquisition");
  return NULL;
}

/** Hint acqusition at satellites observed by peer.

RTK relies on a common set of measurements, have the receivers focus search
efforts on satellites both are likely to be able to see. Receiver will need
to be sufficiently close for RTK to function, and so should have a similar
view of the constellation, even if obstructed this may change if the receivers
move or the satellite arcs across the sky.

cturvey 10-Feb-2015
*/
void manage_set_obs_hint(gnss_signal_t sid)
{
  bool valid = sid_supported(sid);
  assert(valid);
  if (valid)
    acq_status[sid_to_global_index(sid)].score[ACQ_HINT_REMOTE_OBS] = SCORE_OBS;
}

/** Manages acquisition searches and starts tracking channels after successful acquisitions. */
static void manage_acq()
{
  /* Decide which SID to try and then start it acquiring. */
  acq_status_t *acq = choose_acq_sat();
  if (acq == NULL) {
    return;
  }

  /* Only GPS L1CA acquistion is supported. */
  assert(CODE_GPS_L1CA == acq->sid.code);

  float doppler_min = code_to_sv_doppler_min(acq->sid.code) +
                      code_to_tcxo_doppler_min(acq->sid.code);
  float doppler_max = code_to_sv_doppler_max(acq->sid.code) +
                      code_to_tcxo_doppler_max(acq->sid.code);

  /* Check for NaNs in dopp hints, or low > high */
  if ((acq->dopp_hint_low > acq->dopp_hint_high) ||
      (acq->dopp_hint_low < doppler_min) ||
      (acq->dopp_hint_high > doppler_max)) {
    log_error_sid(acq->sid, "Acq: caught bogus dopp_hints (%lf, %lf)",
                  acq->dopp_hint_low,
                  acq->dopp_hint_high);
    acq->dopp_hint_high = doppler_max;
    acq->dopp_hint_low = doppler_min;
  }

  acq_result_t acq_result;
  if (acq_search(acq->sid, acq->dopp_hint_low, acq->dopp_hint_high,
                 ACQ_FULL_CF_STEP, &acq_result)) {

    /* Send result of an acquisition to the host. */
    acq_result_send(acq->sid, acq_result.cn0, acq_result.cp, acq_result.cf);

    if (acq_result.cn0 < ACQ_THRESHOLD) {
      /* Didn't find the satellite :( */
      /* Double the size of the doppler search space for next time. */
      float dilute = (acq->dopp_hint_high - acq->dopp_hint_low) / 2;
      acq->dopp_hint_high = MIN(acq->dopp_hint_high + dilute, doppler_max);
      acq->dopp_hint_low = MAX(acq->dopp_hint_low - dilute, doppler_min);
      /* Decay hint scores */
      for (u8 i = 0; i < ACQ_HINT_NUM; i++)
        acq->score[i] = (acq->score[i] * 3) / 4;
      /* Reset hint score for acquisition. */
      acq->score[ACQ_HINT_PREV_ACQ] = 0;
      return;
    }

    tracking_startup_params_t tracking_startup_params = {
      .sid = acq->sid,
      .sample_count = acq_result.sample_count,
      .carrier_freq = acq_result.cf,
      .code_phase = acq_result.cp,
      .chips_to_correlate = code_to_chip_count(acq->sid.code),
      .cn0_init = acq_result.cn0,
      .elevation = TRACKING_ELEVATION_UNKNOWN
    };

    tracking_startup_request(&tracking_startup_params);
  }
}

/** Send results of an acquisition to the host.
 *
 * \param sid SID of the acquisition
 * \param cn0 Carrier to noise ratio of best point from acquisition.
 * \param cp  Code phase of best point.
 * \param cf  Carrier frequency of best point.
 */
void acq_result_send(gnss_signal_t sid, float cn0, float cp, float cf)
{
  msg_acq_result_t acq_result_msg;

  acq_result_msg.sid = sid_to_sbp(sid);
  acq_result_msg.cn0 = cn0;
  acq_result_msg.cp = cp;
  acq_result_msg.cf = cf;

  sbp_send_msg(SBP_MSG_ACQ_RESULT,
               sizeof(msg_acq_result_t),
               (u8 *)&acq_result_msg);
}

static void drop_channel(u8 channel_id,
                         ch_drop_reason_t reason,
                         const tracking_channel_info_t      *info,
                         const tracking_channel_time_info_t *time_info,
                         const tracking_channel_freq_info_t *freq_info);

/** Find an available tracking channel to start tracking an acquired PRN with.
 *
 * \return Index of first unused tracking channel.
 */
static u8 manage_track_new_acq(gnss_signal_t sid)
{
  /* Decide which (if any) tracking channel to put
   * a newly acquired satellite into.
   */
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    if (code_requires_decoder(sid.code) &&
        tracker_channel_available(i, sid) &&
        decoder_channel_available(i, sid)) {
      return i;
    } else if (!code_requires_decoder(sid.code)  &&
               tracker_channel_available(i, sid)) {
      return i;
    }
  }

  return MANAGE_NO_CHANNELS_FREE;
}

/** Clear unhealthy flags after some time, so we eventually retry
    those sats in case they recover from their sickness.  Call this
    function regularly, and once per day it will reset the flags. */
static void check_clear_unhealthy(void)
{
  static systime_t ticks;
  if (chVTTimeElapsedSinceX(ticks) < S2ST(24*60*60))
    return;

  ticks = chVTGetSystemTime();

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (ACQ_PRN_UNHEALTHY == acq_status[i].state) {
      acq_status[i].state = ACQ_PRN_ACQUIRING;
    }
  }
}

static WORKING_AREA_BCKP(wa_manage_track_thread, MANAGE_TRACK_THREAD_STACK);
static void manage_track_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage track");
  while (TRUE) {
    chThdSleepMilliseconds(500);
    DO_EVERY(2,
      check_clear_unhealthy();
      manage_track();
      watchdog_notify(WD_NOTIFY_TRACKING_MGMT);
    );
    tracking_send_state();
    tracking_send_detailed_state();
  }
}

void manage_track_setup()
{
  SETTING("track", "elevation_mask", tracking_elevation_mask, TYPE_FLOAT);
  SETTING("solution", "elevation_mask", solution_elevation_mask, TYPE_FLOAT);

  chThdCreateStatic(
      wa_manage_track_thread,
      sizeof(wa_manage_track_thread),
      MANAGE_TRACK_THREAD_PRIORITY,
      manage_track_thread, NULL
  );
}

/**
 * Expose the elevation mask setting
 */
float get_solution_elevation_mask()
{
  return solution_elevation_mask;
}

/**
 * Helper to provide channel drop reason literal.
 * \param[in] reason Channel drop reason.
 *
 * \return Literal for the given \a reason.
 */
static const char* get_ch_drop_reason_str(ch_drop_reason_t reason)
{
  const char *str = "";
  switch (reason) {
  case CH_DROP_REASON_ERROR: str = "error occurred, dropping"; break;
  case CH_DROP_REASON_MASKED: str = "channel is masked, dropping"; break;
  case CH_DROP_REASON_NO_BIT_SYNC: str = "no bit sync, dropping"; break;
  case CH_DROP_REASON_NO_PLOCK: str = "No pessimistic lock for too long, dropping"; break;
  case CH_DROP_REASON_LOW_CN0: str = "low CN0 too long, dropping"; break;
  case CH_DROP_REASON_XCORR: str = "cross-correlation confirmed, dropping"; break;
  case CH_DROP_REASON_NO_UPDATES: str = "no updates, dropping"; break;
  case CH_DROP_REASON_L2CL_SYNC: str = "L2CM half-cycle ambiguity resolved, dropping L2CL"; break;
  default: assert(!"Unknown channel drop reason");
  }
  return str;
}

/**
 * Processes channel drop operation.
 *
 * The method logs channel drop reason message, actually disables tracking
 * channel components and updates ACQ hints for re-acqusition.
 *
 * \param[in] channel_id Channel number
 * \param[in] reason     Channel drop reason
 * \param[in] info       Generic data block for dropped channel
 * \param[in] time_info  Time data block for dropped channel
 * \param[in] freq_info  Frequency/phase data block for dropped channel
 *
 * \return None
 */
static void drop_channel(u8 channel_id,
                         ch_drop_reason_t reason,
                         const tracking_channel_info_t      *info,
                         const tracking_channel_time_info_t *time_info,
                         const tracking_channel_freq_info_t *freq_info)
{
  /* Read the required parameters from the tracking channel first to ensure
   * that the tracking channel is not restarted in the mean time.
   */
  gnss_signal_t sid = info->sid;
  tracking_channel_flags_t flags = info->flags;
  u64 now = timing_getms();
  u32 time_in_track = (u32)(now - info->init_timestamp_ms);

  /* Log message with appropriate priority. */
  if (CH_DROP_REASON_ERROR == reason) {
    /* Errors are always logged as errors */
    log_error_sid(sid, "[+%" PRIu32 "ms] %s", time_in_track,
                  get_ch_drop_reason_str(reason));
  } else if (0 == (flags & TRACKING_CHANNEL_FLAG_CONFIRMED)) {
    /* Unconfirmed tracker messages are always logged at debug level */
    log_debug_sid(sid, "[+%" PRIu32 "ms] %s", time_in_track,
                  get_ch_drop_reason_str(reason));
  } else {
    /* Confirmed tracker messages are always logged at info level */
    log_info_sid(sid, "[+%" PRIu32 "ms] %s", time_in_track,
                 get_ch_drop_reason_str(reason));
  }
  /*
   * TODO add generation of a tracker state change message
   */

  acq_status_t *acq = &acq_status[sid_to_global_index(sid)];
  if (code_requires_direct_acq(sid.code)) {
    bool had_locks = 0 != (info->flags & TRACKING_CHANNEL_FLAG_HAD_LOCKS);
    bool long_in_track = time_in_track > TRACK_REACQ_T;
    u32 unlocked_time = time_info->ld_pess_unlocked_ms;
    bool long_unlocked = unlocked_time > TRACK_REACQ_T;
    bool was_xcorr = (info->flags & TRACKING_CHANNEL_FLAG_XCORR_CONFIRMED);

    if (long_in_track && had_locks && !long_unlocked && !was_xcorr) {
      double carrier_freq = freq_info->carrier_freq_at_lock;
      float doppler_min = code_to_sv_doppler_min(sid.code) +
                          code_to_tcxo_doppler_min(sid.code);
      float doppler_max = code_to_sv_doppler_max(sid.code) +
                          code_to_tcxo_doppler_max(sid.code);
      if ((carrier_freq < doppler_min) || (carrier_freq > doppler_max)) {
        log_error_sid(sid, "Acq: bogus carr freq: %lf. Rejected.", carrier_freq);
      } else {
        /* FIXME other constellations/bands */
        acq->score[ACQ_HINT_PREV_TRACK] = SCORE_TRACK;
        acq->dopp_hint_low = MAX(carrier_freq - ACQ_FULL_CF_STEP, doppler_min);
        acq->dopp_hint_high = MIN(carrier_freq + ACQ_FULL_CF_STEP, doppler_max);
      }
    }
  }
  acq->state = ACQ_PRN_ACQUIRING;

  /* Finally disable the decoder and tracking channels */
  decoder_channel_disable(channel_id);
  tracker_channel_disable(channel_id);
}

/** Disable any tracking channel that has errored, too weak, lost phase lock
 * or bit sync, or is flagged as cross-correlation, etc.
 * Keep tracking unhealthy and low-elevation satellites for cross-correlation
 * purposes. */
static void manage_track()
{
  tracking_channel_info_t info;
  tracking_channel_time_info_t time_info;
  tracking_channel_freq_info_t freq_info;
  tracking_channel_misc_info_t misc_info;
  u64 now;

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracking_channel_get_values(i,
                                &info,      /* Generic info */
                                &time_info, /* Timers */
                                &freq_info, /* Frequencies */
                                NULL,       /* Loop controller values */
                                &misc_info, /* Misc info */
                                false);     /* Reset stats */

    now = timing_getms();

    /* Skip channels that aren't in use */
    if (0 == (info.flags & TRACKING_CHANNEL_FLAG_ACTIVE)) {
      continue;
    }

    gnss_signal_t sid = info.sid;
    u16 global_index = sid_to_global_index(sid);

    /* Has an error occurred? */
    if (0 == (info.flags & TRACKING_CHANNEL_FLAG_NO_ERROR)) {
      drop_channel(i, CH_DROP_REASON_ERROR, &info, &time_info, &freq_info);
      continue;
    }

    /* Is tracking masked? */
    if (track_mask[global_index]) {
      drop_channel(i, CH_DROP_REASON_MASKED, &info, &time_info, &freq_info);
      continue;
    }

    /* Give newly-initialized channels a chance to converge.
     * Signals other than GPS L2CL are given longer time. */
    if ((now - info.init_timestamp_ms) < TRACK_INIT_T &&
        sid.code != CODE_GPS_L2CL) {
      continue;
    }

    /* Give newly-initialized L2CL channels a chance to converge.
     * GPS L2CL signals are expected to stabilize fast. */
    if ((now - info.init_timestamp_ms) < TRACK_INIT_T_L2CL &&
        sid.code == CODE_GPS_L2CL) {
      continue;
    }

    if ((now - info.update_timestamp_ms) > NAP_CORR_LENGTH_MAX_MS) {
      drop_channel(i, CH_DROP_REASON_NO_UPDATES, &info, &time_info, &freq_info);
      continue;
    }

    /* Do we not have nav bit sync yet? */
    if (0 == (info.flags & TRACKING_CHANNEL_FLAG_BIT_SYNC)) {
      drop_channel(i, CH_DROP_REASON_NO_BIT_SYNC, &info, &time_info, &freq_info);
      continue;
    }

    /* PLL/FLL pessimistic lock detector "unlocked" for a while? */
    if (time_info.ld_pess_unlocked_ms > TRACK_DROP_UNLOCKED_T) {
      drop_channel(i, CH_DROP_REASON_NO_PLOCK, &info, &time_info, &freq_info);
      continue;
    }

    /* CN0 below threshold for a while? */
    if (time_info.cn0_drop_ms > TRACK_DROP_CN0_T) {
      drop_channel(i, CH_DROP_REASON_LOW_CN0, &info, &time_info, &freq_info);
      continue;
    }

    /* Do we have confirmed cross-correlation? */
    if (0 != (info.flags & TRACKING_CHANNEL_FLAG_XCORR_CONFIRMED)) {
      drop_channel(i, CH_DROP_REASON_XCORR, &info, &time_info, &freq_info);
      continue;
    }

    /* Drop L2CL if the half-cycle ambiguity has been resolved. */
    if (0 != (info.flags & TRACKING_CHANNEL_FLAG_L2CL_AMBIGUITY_SOLVED)) {
      drop_channel(i, CH_DROP_REASON_L2CL_SYNC, &info, &time_info, &freq_info);
      continue;
    }
  }
}

/**
 * Provides a set of base channel flags
 *
 * The method queries tracking channel status and combines it as a set of flags.
 * Because flag computation involves loading of data from external sources, the
 * caller may optionally provide data destination pointers to avoid data
 * reloading.
 *
 * \param[in]  i         Channel index.
 * \param[out] info      Optional destination for tracker generic information.
 * \param[out] time_info Optional destination for tracker time information.
 * \param[out] freq_info Optional destination for tracker carrier and phase
 *                       information.
 * \param[out] ctrl_info Optional destination for tracker controller information.
 *
 * \return Tracker status flags combined in a single set.
 *
 * \sa get_tracking_channel_sid_flags
 *
 * \sa use_tracking_channel
 * \sa tracking_channels_ready
 * \sa tracking_channel_lock
 * \sa tracking_channel_unlock
 */
static manage_track_flags_t get_tracking_channel_flags_info(u8 i,
                                        tracking_channel_info_t *info,
                                        tracking_channel_time_info_t *time_info,
                                        tracking_channel_freq_info_t *freq_info,
                                        tracking_channel_ctrl_info_t *ctrl_info,
                                        tracking_channel_misc_info_t *misc_info)
{
  tracking_channel_info_t tmp_info;
  tracking_channel_time_info_t tmp_time_info;

  if (NULL == info) {
    info = &tmp_info;
  }
  if (NULL == time_info) {
    time_info = &tmp_time_info;
  }

  manage_track_flags_t result = 0;
  tracking_channel_flags_t tc_flags = 0;

  tracking_channel_get_values(i,
                              info,       /* Generic info */
                              time_info,  /* Timers */
                              freq_info,  /* Frequencies */
                              ctrl_info,  /* Loop controller values */
                              misc_info,  /* Misc info */
                              false);     /* Reset stats */

  /* Convert 'tracking_channel_flags_t' flags into 'manage_track_flags_t' */
  tc_flags = info->flags;
  if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_ACTIVE)) {
    result |= MANAGE_TRACK_FLAG_ACTIVE;

    /* Make sure no errors have occurred. */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_NO_ERROR)) {
      result |= MANAGE_TRACK_FLAG_NO_ERROR;
    }
    /* Check if the tracking is in confirmed state. */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_CONFIRMED)) {
      result |= MANAGE_TRACK_FLAG_CONFIRMED;
    }
    /* Channel time of week has been decoded. */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_TOW)) {
      result |= MANAGE_TRACK_FLAG_TOW;
    }
    /* Nav bit polarity is known, i.e. half-cycles have been resolved. */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_BIT_POLARITY)) {
      result |= MANAGE_TRACK_FLAG_BIT_POLARITY;
    }
    /* PLL tracking with or without FLL assist */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_PLL_USE)) {
      result |= MANAGE_TRACK_FLAG_PLL_USE;
    }
    /* FLL tracking or PLL with FLL assist */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_FLL_USE)) {
      result |= MANAGE_TRACK_FLAG_FLL_USE;
    }
    /* Tracking status: pessimistic PLL lock */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_PLL_PLOCK)) {
      result |= MANAGE_TRACK_FLAG_PLL_PLOCK;
    }
    /* Tracking status: optimistic PLL lock */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_PLL_OLOCK)) {
      result |= MANAGE_TRACK_FLAG_PLL_OLOCK;
    }
    /* Tracking status: FLL lock */
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_FLL_LOCK)) {
      result |= MANAGE_TRACK_FLAG_FLL_LOCK;
    }

    /* Check C/N0 has been above threshold for a long time (RTK). */
    if (time_info->cn0_usable_ms > TRACK_CN0_THRES_COUNT_LONG) {
      result |= MANAGE_TRACK_FLAG_CN0_LONG;
    }
    /* Check C/N0 has been above threshold for the minimum time (SPP). */
    if (time_info->cn0_usable_ms  > TRACK_CN0_THRES_COUNT_SHORT) {
      result |= MANAGE_TRACK_FLAG_CN0_SHORT;
    }
    /* Pessimistic phase lock detector = "locked". */
    if (time_info->ld_pess_locked_ms > TRACK_USE_LOCKED_T) {
      result |= MANAGE_TRACK_FLAG_CONFIRMED_LOCK;
    }
    /* Some time has elapsed since the last tracking channel mode
     * change, to allow any transients to stabilize.
     * TODO: is this still necessary? */
    if (time_info->last_mode_change_ms > TRACK_STABILIZATION_T) {
      result |= MANAGE_TRACK_FLAG_STABLE;
    }
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_XCORR_CONFIRMED)) {
      result |= MANAGE_TRACK_FLAG_XCORR_CONFIRMED;
    }
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_XCORR_SUSPECT)) {
      result |= MANAGE_TRACK_FLAG_XCORR_SUSPECT;
    }
    if (0 != (tc_flags & TRACKING_CHANNEL_FLAG_L2CL_AMBIGUITY_SOLVED)) {
      result |= MANAGE_TRACK_FLAG_L2CL_AMBIGUITY;
    }
  }

  return result;
}
manage_track_flags_t get_tracking_channel_flags(u8 i)
{
  return get_tracking_channel_flags_info(i,    /* Tracking channel index */
                                         NULL, /* Generic info */
                                         NULL, /* Time info */
                                         NULL, /* Frequencies/phases */
                                         NULL, /* Controller info */
                                         NULL);/* Misc info */
}

/**
 * Computes carrier phase offset.
 *
 * \param[in]  ref_tc Reference time
 * \param[in]  info   Generic tracker data for update
 * \param[in]  meas   Pre-populated channel measurement
 * \param[out] carrier_phase_offset Result
 *
 * \retval true Carrier phase offset is computed and \a carrier_phase_offset
 *              updated
 * \retval false Error in computation.
 */
static bool compute_cpo(u64 ref_tc,
                        const tracking_channel_info_t *info,
                        const channel_measurement_t *meas,
                        double *carrier_phase_offset)
{
  /* compute the pseudorange for this signal */
  double raw_pseudorange;
  bool ret = tracking_channel_calc_pseudorange(ref_tc, meas,
                                               &raw_pseudorange);
  if (ret) {
    /* We don't want to adjust for the recevier clock drift,
     * so we need to calculate an estimate of that before we
     * calculate the carrier phase offset */
    gps_time_t receiver_time = napcount2rcvtime(ref_tc);
    gps_time_t gps_time = napcount2gpstime(ref_tc);

    double rcv_clk_error =  gpsdifftime(&gps_time,&receiver_time);

    double phase = (code_to_carr_freq(meas->sid.code) *
      ( raw_pseudorange / GPS_C - rcv_clk_error ));

    /* initialize the carrier phase offset with the pseudorange measurement */
    /* NOTE: CP sign flip - change the plus sign below */
    *carrier_phase_offset = round(meas->carrier_phase + phase);

    if (0 != (info->flags & TRACKING_CHANNEL_FLAG_PLL_PLOCK) &&
        0 != (info->flags & TRACKING_CHANNEL_FLAG_CN0_SHORT)) {
      /* Remember offset for the future use */
      tracking_channel_set_carrier_phase_offset(info, *carrier_phase_offset);
    }
  }
  return ret;
}

/**
 * Computes channel measurement flags from input.
 *
 * \param[in] flags Tracker manager flags
 * \param[in] phase_offset_ok Phase offset flag
 * \param[in] sid SID
 *
 * \return Channel measurement flags
 */
static chan_meas_flags_t compute_meas_flags(manage_track_flags_t flags,
                                            bool phase_offset_ok,
                                            gnss_signal_t sid)
{
  chan_meas_flags_t meas_flags = 0;

  if (0 != (flags & MANAGE_TRACK_FLAG_PLL_USE)) {
    /* PLL is in use. */
    if (phase_offset_ok) {
      if (0 != (flags & MANAGE_TRACK_FLAG_PLL_PLOCK) &&
          0 != (flags & MANAGE_TRACK_FLAG_STABLE) &&
          0 != (flags & MANAGE_TRACK_FLAG_CARRIER_PHASE_OFFSET)) {
        meas_flags |= CHAN_MEAS_FLAG_PHASE_VALID;

        /* Make sense to set half cycle known flag when carrier phase is valid */
        if (0 != (flags & MANAGE_TRACK_FLAG_BIT_POLARITY)) {
          /* Bit polarity is known */
          meas_flags |= CHAN_MEAS_FLAG_HALF_CYCLE_KNOWN;
        }
      }

      /* sanity check */
      if ((flags & MANAGE_TRACK_FLAG_BIT_POLARITY)
           && !(flags & MANAGE_TRACK_FLAG_PLL_PLOCK)) {
        /* Somehow we managed to decode TOW when phase lock lost. this should not happen,
         * so print out warning */
        log_warn_sid(sid, "Half cycle known, but no phase lock!");
      }

      if (0 != (flags & MANAGE_TRACK_FLAG_PLL_OLOCK)) {
        /* Optimistic PLL lock: very high noise may prevent phase usage */
        /* meas_flags |= CHAN_MEAS_FLAG_PHASE_VALID; */
      }
    }
    /* In PLL mode code and doppler accuracy are assumed to be high */
    meas_flags |= CHAN_MEAS_FLAG_CODE_VALID;
    meas_flags |= CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID;
  } else if (0 != (flags & MANAGE_TRACK_FLAG_FLL_USE)) {
    /* FLL is in use: no phase measurements; code is valid */
    meas_flags |= CHAN_MEAS_FLAG_CODE_VALID;
    if (0 != (flags & MANAGE_TRACK_FLAG_FLL_LOCK)) {
      /* Doppler is valid only if there is FLL lock */
      meas_flags |= CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID;
    }
  } else {
    assert(!"Unknown tracker mode");
  }

  return meas_flags;
}

/**
 * Loads measurement data.
 *
 * The method loads data from a tracker thread and populates result in \a meas
 * container.
 *
 * Additionally, the method computes initial carrier phase offset if it has
 * not been yet available and feeds it back to tracker.
 *
 * \param[in]  i      Tracking channel number.
 * \param[in]  ref_tc Reference time [ticks]
 * \param[out] meas   Container for measurement data.
 *
 * \return Flags
 */
manage_track_flags_t get_tracking_channel_meas(u8 i,
                                               u64 ref_tc,
                                               channel_measurement_t *meas)
{
  manage_track_flags_t         flags = 0; /* Result */
  tracking_channel_info_t      info;      /* Container for generic info */
  tracking_channel_freq_info_t freq_info; /* Container for measurements */
  tracking_channel_time_info_t time_info; /* Container for time info */
  tracking_channel_misc_info_t misc_info; /* Container for measurements */

  memset(meas, 0, sizeof(*meas));

  /* Load information from tracker: info locks */
  flags =  get_tracking_channel_flags_info(i,           /* Channel index */
                                           &info,       /* General */
                                           &time_info,  /* Time info */
                                           &freq_info,  /* Freq info */
                                           NULL,        /* Ctrl info */
                                           &misc_info); /* Misc info */

  if (0 != (flags & MANAGE_TRACK_FLAG_ACTIVE) &&
      0 != (flags & MANAGE_TRACK_FLAG_CONFIRMED) &&
      0 != (flags & MANAGE_TRACK_FLAG_NO_ERROR) &&
      0 == (flags & MANAGE_TRACK_FLAG_XCORR_SUSPECT)) {
    /* Load information from SID cache and NDB */
    flags |= get_tracking_channel_sid_flags(info.sid, info.tow_ms, NULL);

    tracking_channel_measurement_get(ref_tc, &info,
                                     &freq_info, &time_info, &misc_info, meas);

    /* Adjust for half phase ambiguity */
    if (0 != (info.flags & TRACKING_CHANNEL_FLAG_BIT_INVERTED)) {
      meas->carrier_phase += 0.5;
    }

    /* Adjust carrier phase initial integer offset to be approximately equal to
     * pseudorange.
     *
     * The initial integer offset shall be adjusted only when conditions that
     * have caused initial offset reset are not longer present. See callers of
     * tracker_ambiguity_unknown() for more details.
     *
     * For now, compute pseudorange for all measurements even when phase is
     * not available.
     */
    double carrier_phase_offset = misc_info.carrier_phase_offset.value;
    bool cpo_ok = true;
    if (TIME_FINE <= time_quality &&
        0.0 == carrier_phase_offset &&
        /* 0 != (flags & MANAGE_TRACK_FLAG_CONFIRMED) && */
        0 != (flags & MANAGE_TRACK_FLAG_PLL_USE) &&
        0 != (flags & MANAGE_TRACK_FLAG_PLL_PLOCK) &&
        0 != (flags & MANAGE_TRACK_FLAG_TOW)
        /* 0 != (flags & MANAGE_TRACK_FLAG_CN0_SHORT) */) {
      cpo_ok = compute_cpo(ref_tc, &info, meas, &carrier_phase_offset);
    }
    if (0.0 != carrier_phase_offset) {
      flags |= MANAGE_TRACK_FLAG_CARRIER_PHASE_OFFSET;
      meas->carrier_phase -= carrier_phase_offset;
    }
    meas->flags = compute_meas_flags(flags, cpo_ok, info.sid);
  } else {
    memset(meas, 0, sizeof(*meas));
  }

  return flags;
}

/**
 * Retrieve tracking loop controller parameters for weights computation.
 *
 * \param[in]  i       Channel index.
 * \param[out] pparams Loop controller parameters.
 *
 * \return None
 */
void get_tracking_channel_ctrl_params(u8 i, tracking_ctrl_params_t *pparams)
{
  tracking_channel_ctrl_info_t tmp;

  tracking_channel_get_values(i,
                              NULL,   /* Generic info */
                              NULL,   /* Timers */
                              NULL,   /* Frequencies */
                              &tmp,   /* Loop controller values */
                              NULL,   /* Misc info */
                              false); /* Reset stats */
  pparams->pll_bw = tmp.pll_bw;
  pparams->fll_bw = tmp.fll_bw;
  pparams->dll_bw = tmp.dll_bw;
  pparams->int_ms = tmp.int_ms;
}
/**
 * Compute extended tracking flags for GNSS signal.
 *
 * The method computes additional channel flags by using non-tracking data
 * sources. This is done to prevent potential dead-locking and reduce the
 * size of tracking lock congestion.
 *
 * \param[in]  sid    GNSS signal identifier.
 * \param[in]  tow_ms ToW in milliseconds. Can be #TOW_UNKNOWN
 * \param[out] pephe  Optional destination for ephemeris when available.
 *
 * \return Flags, computed from ephemeris and other sources.
 */
manage_track_flags_t get_tracking_channel_sid_flags(gnss_signal_t sid,
                                                    s32 tow_ms,
                                                    ephemeris_t *pephe)
{
  manage_track_flags_t result = 0;

  /* Satellite elevation is above the solution mask. */
  if (sv_elevation_degrees_get(sid) >= solution_elevation_mask) {
    result |= MANAGE_TRACK_FLAG_ELEVATION;
  }

  if (TOW_UNKNOWN != tow_ms) {
    /* Ephemeris must be valid, not stale. Satellite must be healthy.
       This also acts as a sanity check on the channel TOW.*/
    gps_time_t t = {
      .wn = WN_UNKNOWN,
      .tow = 1e-3 * tow_ms
    };
    ephemeris_t ephe;
    if (NULL == pephe) {
      /* If no external storage for ephemeris is provided, use local one */
      pephe = &ephe;
    }
    ndb_op_code_t res = ndb_ephemeris_read(sid, pephe);
    /* TTFF shortcut: accept also unconfirmed ephemeris candidate when there
     * is no confirmed candidate */
    if (NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res) {
      if (ephemeris_valid(pephe, &t)) {
        result |= MANAGE_TRACK_FLAG_HAS_EPHE;

        if (signal_healthy(pephe->valid,
                           pephe->health_bits,
                           pephe->ura,
                           sid.code)) {
          result |= MANAGE_TRACK_FLAG_HEALTHY;
        }
      }
    }
  }
  /* Navigation suitable flag */
  if (shm_navigation_suitable(sid)) {
    result |= MANAGE_TRACK_FLAG_NAV_SUITABLE;
  }
  return result;
}

/**
 * Helper method check if the channel state is usable.
 *
 * The method checks if the tracking channel state has \a required_flags
 *
 * \param[in] i              Tracking channel index.
 * \param[in] required_flags Flags that are required to be present in channel
 *                           state to be usable.
 *
 * \retval true  Tracking channel state has all \a required_flags.
 * \retval false One or more flags are missing from the tracking channel state.
 *
 * \sa get_tracking_channel_flags
 * \sa tracking_channels_ready
 */
bool tracking_channel_is_usable(u8 i, manage_track_flags_t required_flags)
{
  manage_track_flags_t flags = 0; /* Channel flags accumulator */

  /* While locked, load base flags and ToW */

  tracking_channel_info_t info;
  tracking_channel_get_values(i,
                              &info,  /* Generic info */
                              NULL,   /* Timers */
                              NULL,   /* Frequencies */
                              NULL,   /* Loop controller values */
                              NULL,   /* Misc values */
                              false); /* Reset stats */

  flags = info.flags;
  if (0 != (flags & MANAGE_TRACK_FLAG_ACTIVE)) {
    /* While unlocked, load ext flags and ephe. */
    flags |= get_tracking_channel_sid_flags(info.sid, info.tow_ms, NULL);
  }

  return (flags & required_flags) == required_flags;
}

/**
 * Counts a number of usable tracking channels.
 *
 * \param[in] required_flags Flags that are required to be present in channel
 *                           state to be usable.
 *
 * \return Count of tracking channels that satisfy \a required_flags
 *
 * \sa tracking_channel_is_usable
 */
u8 tracking_channels_ready(manage_track_flags_t required_flags)
{
  u8 n_ready = 0;
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    if (tracking_channel_is_usable(i, required_flags)) {
      n_ready++;
    }
  }
  return n_ready;
}

/** Checks if tracking can be started for a given sid.
 *
 * \param sid Signal ID to check.
 * \retval true sid tracking can be started.
 * \retval false sid tracking cannot be started.
 */
bool tracking_startup_ready(gnss_signal_t sid)
{
  u16 global_index = sid_to_global_index(sid);
  acq_status_t *acq = &acq_status[global_index];
  return (acq->state == ACQ_PRN_ACQUIRING) && (!acq->masked);
}

/**
 * Checks if the tracker is running.
 *
 * \param[in] sid GNSS signal ID to check.
 *
 * \retval true  Tracker for \a sid is in running state
 * \retval false Tracker for \a sid is not in running state
 */
bool tracking_is_running(gnss_signal_t sid)
{
  u16 global_index = sid_to_global_index(sid);
  acq_status_t *acq = &acq_status[global_index];
  return (acq->state == ACQ_PRN_TRACKING);
}

/** Check if a startup request for an sid is present in a
 *  tracking startup FIFO.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 * \param sid         gnss_signal_t to use.
 *
 * \return true if the sid is present, false otherwise.
 */
static bool tracking_startup_fifo_sid_present(
                                            const tracking_startup_fifo_t *fifo,
                                            gnss_signal_t sid)
{
  tracking_startup_fifo_index_t read_index = fifo->read_index;
  tracking_startup_fifo_index_t write_index = fifo->write_index;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  while(read_index != write_index) {
    const tracking_startup_params_t *p =
        &fifo->elements[read_index++ & TRACKING_STARTUP_FIFO_INDEX_MASK];
    if (sid_is_equal(p->sid, sid)) {
      return true;
    }
  }
  return false;
}

/** Queue a request to start up tracking and decoding for the specified sid.
 *
 * \note This function is thread-safe and non-blocking.
 *
 * \param startup_params    Struct containing startup parameters.
 *
 * \retval 0 if the request was successfully submitted
 * \retval 1 if requested sid is already in the fifo
 * \retval 2 otherwise
 *
 */
u8 tracking_startup_request(const tracking_startup_params_t *startup_params)
{
  u8 result = 2;
  if(chMtxTryLock(&tracking_startup_mutex))
  {
    if(!tracking_startup_fifo_sid_present(&tracking_startup_fifo,
                                          startup_params->sid)) {
      if(tracking_startup_fifo_write(&tracking_startup_fifo,
                                           startup_params)) {
        result = 0;
      }
    } else {
      result = 1;
    }

    chMtxUnlock(&tracking_startup_mutex);
  }

  return result;
}

/** Read tracking startup requests from the FIFO and attempt to start
 * tracking and decoding.
 */
static void manage_tracking_startup(void)
{
  tracking_startup_params_t startup_params;
  while(tracking_startup_fifo_read(&tracking_startup_fifo, &startup_params)) {

    acq_status_t *acq = &acq_status[sid_to_global_index(startup_params.sid)];

    /* Make sure the SID is not already tracked. */
    if (acq->state == ACQ_PRN_TRACKING) {
      continue;
    }

    /* Make sure a tracking channel and a decoder channel are available */
    u8 chan = manage_track_new_acq(startup_params.sid);
    if (chan == MANAGE_NO_CHANNELS_FREE) {

      if (code_requires_direct_acq(acq->sid.code)) {
        float doppler_min = code_to_sv_doppler_min(acq->sid.code) +
                            code_to_tcxo_doppler_min(acq->sid.code);
        float doppler_max = code_to_sv_doppler_max(acq->sid.code) +
                            code_to_tcxo_doppler_max(acq->sid.code);

        /* No channels are free to accept our new satellite :( */
        /* TODO: Perhaps we can try to warm start this one
         * later using another fine acq.
         */
        if (startup_params.cn0_init > ACQ_RETRY_THRESHOLD) {
          acq->score[ACQ_HINT_PREV_ACQ] =
              SCORE_ACQ + (startup_params.cn0_init - ACQ_THRESHOLD);
          acq->dopp_hint_low = MAX(startup_params.carrier_freq - ACQ_FULL_CF_STEP,
                                   doppler_min);
          acq->dopp_hint_high = MIN(startup_params.carrier_freq + ACQ_FULL_CF_STEP,
                                    doppler_max);
        }
      }
      log_debug("No free tracking channel available.");
      continue;
    }

    /* Start the tracking channel */
    if(!tracker_channel_init(chan, startup_params.sid,
                             startup_params.sample_count,
                             startup_params.code_phase,
                             startup_params.carrier_freq,
                             startup_params.chips_to_correlate,
                             startup_params.cn0_init)) {
      log_error("tracker channel init failed");
    }

    /* TODO: Initialize elevation from ephemeris if we know it precisely */

    /* Start the decoder channel if needed */
    if (code_requires_decoder(startup_params.sid.code) &&
        !decoder_channel_init(chan, startup_params.sid)) {
      log_error("decoder channel init failed");
    }

    /* Change state to TRACKING */
    acq->state = ACQ_PRN_TRACKING;
  }
}

/** Initialize a tracking_startup_fifo_t struct.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 */
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo)
{
  fifo->read_index = 0;
  fifo->write_index = 0;
}

/** Write data to a tracking startup FIFO.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 * \param element     Element to write to the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool tracking_startup_fifo_write(tracking_startup_fifo_t *fifo,
                                        const tracking_startup_params_t *
                                        element)
{
  if (TRACKING_STARTUP_FIFO_LENGTH(fifo) < TRACKING_STARTUP_FIFO_SIZE) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(&fifo->elements[fifo->write_index &
                           TRACKING_STARTUP_FIFO_INDEX_MASK],
           element, sizeof(tracking_startup_params_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->write_index++;
    return true;
  }

  return false;
}

/** Read pending data from a tracking startup FIFO.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 * \param element     Output element read from the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element)
{
  if (TRACKING_STARTUP_FIFO_LENGTH(fifo) > 0) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    memcpy(element,
           &fifo->elements[fifo->read_index &
                           TRACKING_STARTUP_FIFO_INDEX_MASK],
           sizeof(tracking_startup_params_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->read_index++;
    return true;
  }

  return false;
}

/** Checks if SV is tracked
 *
 * \param sid Signal ID to check.
 * \retval true sid is tracked
 * \retval false sid is not tracked
 */
bool sid_is_tracked(gnss_signal_t sid)
{
  /* This function is used in reacquisition which runs in
     the same thread as acquisition.
     Revisit this if there will be any better way to check
     if SV is in track. */
  u16 global_index = sid_to_global_index(sid);
  acq_status_t *acq = &acq_status[global_index];
  return acq->state == ACQ_PRN_TRACKING;
}


/** \} */
