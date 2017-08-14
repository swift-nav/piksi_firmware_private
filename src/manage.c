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

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libsbp/piksi.h>
#include <libsbp/sbp.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/glo_map.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/signal.h>

#include "./system_monitor.h"
#include "board/nap/track_channel.h"
#include "decode.h"
#include "dum.h"
#include "ephemeris.h"
#include "main.h"
#include "manage.h"
#include "ndb.h"
#include "nmea.h"
#include "piksi_systime.h"
#include "position.h"
#include "reacq/reacq_api.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings.h"
#include "shm.h"
#include "signal.h"
#include "soft_macq/soft_macq_main.h"
#include "timing.h"
#include "track.h"
#include "track/track_sid_db.h"

/** \defgroup manage Manage
 * Manage acquisition and tracking.
 * Manage how acquisition searches are performed, with data from almanac if
 * available. Transition from acquisition search to initialization of an
 * available tracking channel when a satellite is successfully found. Disable
 * tracking channels that have lost lock on their satellites.
 * \{ */

/** Supported channel drop reasons */
typedef enum {
  CH_DROP_REASON_ERROR,        /**< Tracking channel error */
  CH_DROP_REASON_MASKED,       /**< Tracking channel is disabled by mask */
  CH_DROP_REASON_NO_BIT_SYNC,  /**< Bit sync timeout */
  CH_DROP_REASON_NO_PLOCK,     /**< Pessimistic lock timeout */
  CH_DROP_REASON_LOW_CN0,      /**< Low C/N0 for too long */
  CH_DROP_REASON_XCORR,        /**< Confirmed cross-correlation */
  CH_DROP_REASON_NO_UPDATES,   /**< No tracker updates for too long */
  CH_DROP_REASON_L2CL_SYNC,    /**< Drop L2CL after half-cycle ambiguity
                                    has been resolved */
  CH_DROP_REASON_SV_UNHEALTHY, /**< The SV is Unhealthy */
  CH_DROP_REASON_LEAP_SECOND,  /**< Leap second event is imminent,
                                    drop GLO satellites */
  CH_DROP_REASON_OUTLIER,      /**< Doppler outlier */
  CH_DROP_REASON_RAIM          /**< Signal removed by RAIM */
} ch_drop_reason_t;

/** Different hints on satellite info to aid the acqusition */
enum acq_hint {
  ACQ_HINT_WARMSTART,  /**< Information from almanac or ephemeris */
  ACQ_HINT_PREV_ACQ,   /**< Previous successful acqusition. */
  ACQ_HINT_PREV_TRACK, /**< Previously tracked satellite. */
  ACQ_HINT_REMOTE_OBS, /**< Observation from reference station. */

  ACQ_HINT_NUM
};

/** Status of acquisition for a particular ME SID. */
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
  me_gnss_signal_t mesid;  /**< ME signal identifier. */
} acq_status_t;

static acq_status_t acq_status[PLATFORM_ACQ_TRACK_COUNT];
static bool track_mask[ARRAY_SIZE(acq_status)];

#define SCORE_COLDSTART 100
#define SCORE_WARMSTART 200
#define SCORE_BELOWMASK 0
#define SCORE_ACQ 100
#define SCORE_TRACK 200
#define SCORE_OBS 200

#define DOPP_UNCERT_ALMANAC 4000
#define DOPP_UNCERT_EPHEM 500

#define COMPILER_BARRIER() asm volatile("" : : : "memory")

#define TRACKING_STARTUP_FIFO_SIZE 16 /* Must be a power of 2 */

#define TRACKING_STARTUP_FIFO_INDEX_MASK ((TRACKING_STARTUP_FIFO_SIZE)-1)
#define TRACKING_STARTUP_FIFO_INDEX_DIFF(write_index, read_index) \
  ((tracking_startup_fifo_index_t)((write_index) - (read_index)))
#define TRACKING_STARTUP_FIFO_LENGTH(p_fifo)               \
  (TRACKING_STARTUP_FIFO_INDEX_DIFF((p_fifo)->write_index, \
                                    (p_fifo)->read_index))
/* Refer also internal NDB definition NDB_NV_GLO_EPHEMERIS_AGE_SECS */
#define ACQ_GLO_EPH_VALID_TIME_SEC (30 * MINUTE_SECS)

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

/** Flag if almanacs can be used in acq */
static bool almanacs_enabled = false;
/** Flag if GLONASS enabled */
static bool glo_enabled = CODE_GLO_L1CA_SUPPORT || CODE_GLO_L2CA_SUPPORT;

typedef struct {
  piksi_systime_t tick; /**< Time when GLO SV was detected as unhealthy */
  acq_status_t *status; /**< Pointer to acq status for the GLO SV */
} glo_acq_state_t;

/* The array keeps time when GLO SV was detected as unhealthy
 * Number of elemnts is n+1 to avoid index adjusting */
static glo_acq_state_t glo_acq_timer[NUM_SATS_GLO + 1] = {0};

static u8 manage_track_new_acq(const me_gnss_signal_t mesid);
static void manage_acq(void);

static void manage_tracking_startup(void);
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo);
static bool tracking_startup_fifo_write(
    tracking_startup_fifo_t *fifo, const tracking_startup_params_t *element);
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element);

static sbp_msg_callbacks_node_t almanac_callback_node;
static void almanac_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)sender_id;
  (void)len;
  (void)context;
  (void)msg;
}

static sbp_msg_callbacks_node_t mask_sat_callback_node;
static void mask_sat_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)sender_id;
  (void)len;
  (void)context;
  enum {
    MASK_ACQUISITION = 1,
    MASK_TRACKING = 2,
  };

  msg_mask_satellite_t *m = (msg_mask_satellite_t *)msg;
  gnss_signal_t sid = sid_from_sbp(m->sid);

  if (sid_supported(sid)) {
    /* TODO GLO: Handle GLO signals properly. */
    me_gnss_signal_t mesid;
    constellation_t constellation = sid_to_constellation(sid);
    if (CONSTELLATION_GLO == constellation) {
      assert(glo_map_valid(sid));
      u16 fcn = glo_map_get_fcn(sid);
      mesid = construct_mesid(sid.code, fcn);
    } else {
      mesid = construct_mesid(sid.code, sid.sat);
    }
    u16 me_global_index = mesid_to_global_index(mesid);
    acq_status_t *acq = &acq_status[me_global_index];
    acq->masked = (m->mask & MASK_ACQUISITION) ? true : false;
    track_mask[me_global_index] = (m->mask & MASK_TRACKING) ? true : false;
    log_info_sid(sid, "Mask = 0x%02x", m->mask);
  } else {
    log_warn("Mask not set for invalid SID");
  }
}

static THD_WORKING_AREA(wa_manage_acq_thread, MANAGE_ACQ_THREAD_STACK);
static void manage_acq_thread(void *arg) {
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
               ((TIME_COARSE <= get_time_quality()));
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

/* The function masks/unmasks all GLO satellite,
 * NOTE: this function does not check if GLO SV is already masked or not */
static bool glo_enable_notify(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    log_debug("GLONASS status (1 - on, 0 - off): %u", glo_enabled);
    if (glo_enabled && !(CODE_GLO_L1CA_SUPPORT || CODE_GLO_L2CA_SUPPORT)) {
      /* user tries enable GLONASS on the platform that does not support it */
      log_error("The platform does not support GLONASS");
      glo_enabled = false;
      return false;
    }
    for (int i = 0; i < PLATFORM_ACQ_TRACK_COUNT; i++) {
      if (is_glo_sid(acq_status[i].mesid)) {
        acq_status[i].masked = !glo_enabled;
      }
    }
    return true;
  }
  return false;
}

void manage_acq_setup() {
  SETTING("acquisition", "almanacs_enabled", almanacs_enabled, TYPE_BOOL);
  SETTING_NOTIFY("acquisition",
                 "GLONASS_enabled",
                 glo_enabled,
                 TYPE_BOOL,
                 glo_enable_notify);

  tracking_startup_fifo_init(&tracking_startup_fifo);

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    me_gnss_signal_t mesid = mesid_from_global_index(i);
    acq_status[i].state = ACQ_PRN_ACQUIRING;
    if (is_glo_sid(mesid) && !glo_enabled) {
      acq_status[i].masked = true;
    } else {
      acq_status[i].masked = false;
    }
    memset(&acq_status[i].score, 0, sizeof(acq_status[i].score));

    if (code_requires_direct_acq(mesid.code)) {
      acq_status[i].dopp_hint_low = code_to_sv_doppler_min(mesid.code) +
                                    code_to_tcxo_doppler_min(mesid.code);
      acq_status[i].dopp_hint_high = code_to_sv_doppler_max(mesid.code) +
                                     code_to_tcxo_doppler_max(mesid.code);
    }
    acq_status[i].mesid = mesid;

    track_mask[i] = false;
  }

  sbp_register_cbk(SBP_MSG_ALMANAC, &almanac_callback, &almanac_callback_node);

  sbp_register_cbk(
      SBP_MSG_MASK_SATELLITE, &mask_sat_callback, &mask_sat_callback_node);

  chThdCreateStatic(wa_manage_acq_thread,
                    sizeof(wa_manage_acq_thread),
                    MANAGE_ACQ_THREAD_PRIORITY,
                    manage_acq_thread,
                    NULL);
}

/** Using available almanac and ephemeris information, determine
 * whether a satellite is in view and the range of doppler frequencies
 * in which we expect to find it.
 *
 * \param mesid ME signal id
 * \param t     Time at which to evaluate ephemeris and almanac (typically
 * system's
 *              estimate of current time)
 * \param dopp_hint_low, dopp_hint_high Pointers to store doppler search range
 *  from ephemeris or almanac, if available and elevation > mask
 * \return Score (higher is better)
 */
static u16 manage_warm_start(const me_gnss_signal_t mesid,
                             const gps_time_t *t,
                             float *dopp_hint_low,
                             float *dopp_hint_high) {
  /* Do we have any idea where/when we are?  If not, no score. */
  /* TODO: Stricter requirement on time and position uncertainty?
     We ought to keep track of a quantitative uncertainty estimate. */
  last_good_fix_t lgf;
  if (ndb_lgf_read(&lgf) != NDB_ERR_NONE ||
      lgf.position_quality < POSITION_GUESS ||
      get_time_quality() < TIME_GUESS) {
    return SCORE_COLDSTART;
  }

  /* TODO GLO: Handle GLO orbit slot properly. */
  assert(!is_glo_sid(mesid));
  gnss_signal_t sid = mesid2sid(mesid, GLO_ORBIT_SLOT_UNKNOWN);
  float el = TRACKING_ELEVATION_UNKNOWN;
  el = sv_elevation_degrees_get(sid);
  if (el < tracking_elevation_mask) {
    return SCORE_BELOWMASK;
  }

  double dopp_hint = 0, dopp_uncertainty = DOPP_UNCERT_ALMANAC;
  bool ready = false;
  /* Do we have a suitable ephemeris for this sat?  If so, use
     that in preference to the almanac. */
  union {
    ephemeris_t e;
    almanac_t a;
  } orbit;

  u8 eph_valid = 0;
  ndb_op_code_t ndb_ret = NDB_ERR_NO_DATA;
  ndb_ret = ndb_ephemeris_read(sid, &orbit.e);

  s8 ss_ret;
  double sat_pos[3], sat_vel[3], el_d;

  eph_valid = NDB_ERR_NONE == ndb_ret && ephemeris_valid(&orbit.e, t);
  if (eph_valid) {
    double unused;
    u8 iode;
    u16 iodc;
    ss_ret = calc_sat_state(&orbit.e,
                            t,
                            sat_pos,
                            sat_vel,
                            /* double *clock_err = */ &unused,
                            /* double *clock_rate_err = */ &unused,
                            &iodc,
                            &iode);
  }

  if (eph_valid && (ss_ret == 0)) {
    double dopp_hint_sat_vel; /* Doppler hint induced by sat velocity */
    double dopp_hint_clock;   /* Doppler hint induced by clock drift */
    vector_subtract(3, sat_pos, lgf.position_solution.pos_ecef, sat_pos);
    vector_normalize(3, sat_pos);
    /* sat_pos now holds unit vector from us to satellite */
    vector_subtract(3, sat_vel, lgf.position_solution.vel_ecef, sat_vel);
    /* sat_vel now holds velocity of sat relative to us */
    dopp_hint_sat_vel = -sid_to_carr_freq(orbit.e.sid) *
                        vector_dot(3, sat_pos, sat_vel) / GPS_C;
    /* TODO: Check sign of receiver frequency offset correction.
             There seems to be a sign flip somewhere in 'clock_bias'
             computation that gets compensated here */
    dopp_hint_clock =
        -sid_to_carr_freq(orbit.e.sid) * lgf.position_solution.clock_bias;
    dopp_hint = dopp_hint_sat_vel + dopp_hint_clock;
    if (get_time_quality() >= TIME_FINE) {
      dopp_uncertainty = DOPP_UNCERT_EPHEM;
    }
    ready = true;

    if ((dopp_hint_sat_vel < code_to_sv_doppler_min(mesid.code)) ||
        (dopp_hint_sat_vel > code_to_sv_doppler_max(mesid.code)) ||
        (dopp_hint_clock < code_to_tcxo_doppler_min(mesid.code)) ||
        (dopp_hint_clock > code_to_tcxo_doppler_max(mesid.code))) {
      log_error_mesid(mesid,
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

  float doppler_min =
      code_to_sv_doppler_min(mesid.code) + code_to_tcxo_doppler_min(mesid.code);
  float doppler_max =
      code_to_sv_doppler_max(mesid.code) + code_to_tcxo_doppler_max(mesid.code);

  if (!ready) {
    double unused;

    if (almanacs_enabled && (NDB_ERR_NONE == ndb_almanac_read(sid, &orbit.a)) &&
        almanac_valid(&orbit.a, t) &&
        calc_sat_az_el_almanac(&orbit.a,
                               t,
                               lgf.position_solution.pos_ecef,
                               /* double *az = */ &unused,
                               &el_d) == 0) {
      el = (float)(el_d * R2D);
      if (el < tracking_elevation_mask) {
        return SCORE_BELOWMASK;
      }
      if (calc_sat_doppler_almanac(
              &orbit.a, t, lgf.position_solution.pos_ecef, &dopp_hint) != 0) {
        return SCORE_COLDSTART;
      }
      dopp_hint = -dopp_hint;

      if ((dopp_hint < doppler_min) || (dopp_hint > doppler_max)) {
        log_error_mesid(mesid,
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

static acq_status_t *choose_acq_sat(void) {
  u32 total_score = 0;
  gps_time_t t = get_current_time();

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if ((!code_requires_direct_acq(acq_status[i].mesid.code)) ||
        (acq_status[i].state != ACQ_PRN_ACQUIRING) || (acq_status[i].masked)) {
      continue;
    }

    acq_status[i].score[ACQ_HINT_WARMSTART] =
        manage_warm_start(acq_status[i].mesid,
                          &t,
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
    if ((!code_requires_direct_acq(acq_status[i].mesid.code)) ||
        (acq_status[i].state != ACQ_PRN_ACQUIRING) || (acq_status[i].masked)) {
      continue;
    }

    u32 sat_score = 0;
    for (enum acq_hint hint = 0; hint < ACQ_HINT_NUM; hint++) {
      sat_score += acq_status[i].score[hint];
    }
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
void manage_set_obs_hint(gnss_signal_t sid) {
  bool valid = sid_supported(sid);
  assert(valid);
  if (valid) {
    /* TODO GLO: Handle GLO signals properly. */
    me_gnss_signal_t mesid;
    constellation_t constellation = sid_to_constellation(sid);
    if (CONSTELLATION_GLO == constellation) {
      if (!glo_map_valid(sid)) {
        /* no guarantee that we have FCN mapping for an observation
         * received from peer */
        return;
      }
      u16 fcn = glo_map_get_fcn(sid);
      mesid = construct_mesid(sid.code, fcn);
    } else {
      mesid = construct_mesid(sid.code, sid.sat);
    }
    acq_status[mesid_to_global_index(mesid)].score[ACQ_HINT_REMOTE_OBS] =
        SCORE_OBS;
  }
}

/** Manages acquisition searches and starts tracking channels after successful
 * acquisitions. */
static void manage_acq(void) {
  /* Decide which SID to try and then start it acquiring. */
  acq_status_t *acq = choose_acq_sat();
  if (acq == NULL) {
    return;
  }

  /* Only GPS L1CA and GLO L1 direct acquisition is supported. */
  assert((CODE_GPS_L1CA == acq->mesid.code) ||
         (CODE_GLO_L1CA == acq->mesid.code));

  float doppler_min = code_to_sv_doppler_min(acq->mesid.code) +
                      code_to_tcxo_doppler_min(acq->mesid.code);
  float doppler_max = code_to_sv_doppler_max(acq->mesid.code) +
                      code_to_tcxo_doppler_max(acq->mesid.code);

  /* Check for NaNs in dopp hints, or low > high */
  if ((acq->dopp_hint_low > acq->dopp_hint_high) ||
      (acq->dopp_hint_low < doppler_min) ||
      (acq->dopp_hint_high > doppler_max)) {
    log_error_mesid(acq->mesid,
                    "Acq: caught bogus dopp_hints (%lf, %lf)",
                    acq->dopp_hint_low,
                    acq->dopp_hint_high);
    acq->dopp_hint_high = doppler_max;
    acq->dopp_hint_low = doppler_min;
  }

  acq_result_t acq_result;
  if (soft_multi_acq_search(
          acq->mesid, acq->dopp_hint_low, acq->dopp_hint_high, &acq_result)) {
    /* Send result of an acquisition to the host. */
    acq_result_send(acq->mesid, acq_result.cn0, acq_result.cp, acq_result.cf);

    if (acq_result.cn0 < ACQ_THRESHOLD) {
      /* Didn't find the satellite :( */
      /* Double the size of the doppler search space for next time. */
      float dilute = (acq->dopp_hint_high - acq->dopp_hint_low) / 2;
      acq->dopp_hint_high = MIN(acq->dopp_hint_high + dilute, doppler_max);
      acq->dopp_hint_low = MAX(acq->dopp_hint_low - dilute, doppler_min);
      /* Decay hint scores */
      for (u8 i = 0; i < ACQ_HINT_NUM; i++) {
        acq->score[i] = (acq->score[i] * 3) / 4;
      }
      /* Reset hint score for acquisition. */
      acq->score[ACQ_HINT_PREV_ACQ] = 0;
      return;
    }

    tracking_startup_params_t tracking_startup_params = {
        .mesid = acq->mesid,
        .glo_slot_id = GLO_ORBIT_SLOT_UNKNOWN,
        .sample_count = acq_result.sample_count,
        .carrier_freq = acq_result.cf,
        .code_phase = acq_result.cp,
        .chips_to_correlate = code_to_chip_count(acq->mesid.code),
        .cn0_init = acq_result.cn0,
        .elevation = TRACKING_ELEVATION_UNKNOWN};

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
void acq_result_send(const me_gnss_signal_t mesid,
                     float cn0,
                     float cp,
                     float cf) {
  msg_acq_result_t acq_result_msg;
  /* TODO GLO: Handle GLO orbit slot properly. */
  if (is_glo_sid(mesid)) {
    return;
  }
  acq_result_msg.sid = sid_to_sbp(mesid2sid(mesid, GLO_ORBIT_SLOT_UNKNOWN));
  acq_result_msg.cn0 = cn0;
  acq_result_msg.cp = cp;
  acq_result_msg.cf = cf;

  sbp_send_msg(
      SBP_MSG_ACQ_RESULT, sizeof(msg_acq_result_t), (u8 *)&acq_result_msg);
}

/** Find an available tracking channel to start tracking an acquired PRN with.
 *
 * \return Index of first unused tracking channel.
 */
static u8 manage_track_new_acq(const me_gnss_signal_t mesid) {
  /* Decide which (if any) tracking channel to put
   * a newly acquired satellite into.
   */
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    if (code_requires_decoder(mesid.code) &&
        tracker_channel_available(i, mesid) &&
        decoder_channel_available(i, mesid)) {
      return i;
    } else if (!code_requires_decoder(mesid.code) &&
               tracker_channel_available(i, mesid)) {
      return i;
    }
  }

  return MANAGE_NO_CHANNELS_FREE;
}

/** Clear unhealthy flags after some time, so we eventually retry
    those sats in case they recover from their sickness.  Call this
    function regularly, and once per day it will reset the flags. */
void check_clear_unhealthy(void) {
  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (ACQ_PRN_UNHEALTHY == acq_status[i].state) {
      acq_status[i].state = ACQ_PRN_ACQUIRING;
    }
  }
}

/** Check GLO unhealthy flags and clear after GLO ephemeris valid time
 * This function blocks acquiring GLO SV for some time if the SV is unhealthy */
void check_clear_glo_unhealthy(void) {
  if (!is_glo_enabled()) {
    return;
  }

  for (u32 i = 1; i <= NUM_SATS_GLO; i++) {
    if (glo_acq_timer[i].status &&
        (ACQ_PRN_UNHEALTHY == glo_acq_timer[i].status->state)) {
      /* check if time since channel dropped due to SV unhealthy greater
       * than GLO ephemeris valid time (30 min) */
      if (piksi_systime_elapsed_since_s_x(&glo_acq_timer[i].tick) >
          ACQ_GLO_EPH_VALID_TIME_SEC) {
        /* enable GLO aqcuisition again */
        glo_acq_timer[i].status->state = ACQ_PRN_ACQUIRING;
        log_info_mesid(glo_acq_timer[i].status->mesid, "is back to aquisition");
      }
    }
  }
}

void me_settings_setup(void) {
  SETTING("track", "elevation_mask", tracking_elevation_mask, TYPE_FLOAT);
  SETTING("solution", "elevation_mask", solution_elevation_mask, TYPE_FLOAT);
}

/**
 * Expose the elevation mask setting
 */
float get_solution_elevation_mask() { return solution_elevation_mask; }

/**
 * Helper to provide channel drop reason literal.
 * \param[in] reason Channel drop reason.
 *
 * \return Literal for the given \a reason.
 */
static const char *get_ch_drop_reason_str(ch_drop_reason_t reason) {
  const char *str = "";
  switch (reason) {
    case CH_DROP_REASON_ERROR:
      str = "error occurred, dropping";
      break;
    case CH_DROP_REASON_MASKED:
      str = "channel is masked, dropping";
      break;
    case CH_DROP_REASON_NO_BIT_SYNC:
      str = "no bit sync, dropping";
      break;
    case CH_DROP_REASON_NO_PLOCK:
      str = "No pessimistic lock for too long, dropping";
      break;
    case CH_DROP_REASON_LOW_CN0:
      str = "low CN0 too long, dropping";
      break;
    case CH_DROP_REASON_XCORR:
      str = "cross-correlation confirmed, dropping";
      break;
    case CH_DROP_REASON_NO_UPDATES:
      str = "no updates, dropping";
      break;
    case CH_DROP_REASON_L2CL_SYNC:
      str = "L2CM half-cycle ambiguity resolved, dropping L2CL";
      break;
    case CH_DROP_REASON_SV_UNHEALTHY:
      str = "SV is unhealthy, dropping";
      break;
    case CH_DROP_REASON_LEAP_SECOND:
      str = "Leap second event, dropping GLO signal";
      break;
    case CH_DROP_REASON_OUTLIER:
      str = "SV measurement outlier, dropping";
      break;
    case CH_DROP_REASON_RAIM:
      str = "Measurement flagged by RAIM, dropping";
      break;
    default:
      assert(!"Unknown channel drop reason");
  }
  return str;
}

/**
 * Processes channel drop operation.
 *
 * The method logs channel drop reason message, actually disables tracking
 * channel components and updates ACQ hints for re-acqusition.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in] reason     Channel drop reason
 */
static void drop_channel(tracker_channel_t *tracker_channel,
                         ch_drop_reason_t reason) {
  /* Read the required parameters from the tracking channel first to ensure
   * that the tracking channel is not restarted in the mean time.
   */
  const u32 flags = tracker_channel->flags;
  me_gnss_signal_t mesid = tracker_channel->mesid;
  u64 now_ms = timing_getms();
  u32 time_in_track_ms = (u32)(now_ms - tracker_channel->init_timestamp_ms);

  /* Log message with appropriate priority. */
  if ((CH_DROP_REASON_ERROR == reason) ||
      (CH_DROP_REASON_NO_UPDATES == reason)) {
    log_error_mesid(mesid,
                    "[+%" PRIu32 "ms] nap_channel = %" PRIu8 " %s",
                    time_in_track_ms,
                    tracker_channel->nap_channel,
                    get_ch_drop_reason_str(reason));
  } else if (0 == (flags & TRACKER_FLAG_CONFIRMED)) {
    /* Unconfirmed tracker messages are always logged at debug level */
    log_debug_mesid(mesid,
                    "[+%" PRIu32 "ms] %s",
                    time_in_track_ms,
                    get_ch_drop_reason_str(reason));
  } else {
    /* Confirmed tracker messages are always logged at info level */
    log_info_mesid(mesid,
                   "[+%" PRIu32 "ms] %s",
                   time_in_track_ms,
                   get_ch_drop_reason_str(reason));
  }
  /*
   * TODO add generation of a tracker state change message
   */

  acq_status_t *acq = &acq_status[mesid_to_global_index(mesid)];
  if (code_requires_direct_acq(mesid.code)) {
    bool had_locks =
        (0 != (flags & (TRACKER_FLAG_HAD_PLOCK | TRACKER_FLAG_HAD_FLOCK)));
    bool long_in_track = time_in_track_ms > TRACK_REACQ_MS;
    u32 unlocked_time_ms = update_count_diff(
        tracker_channel, &tracker_channel->ld_pess_change_count);
    bool long_unlocked = unlocked_time_ms > TRACK_REACQ_MS;
    bool was_xcorr = (flags & TRACKER_FLAG_XCORR_CONFIRMED);

    if (long_in_track && had_locks && !long_unlocked && !was_xcorr) {
      double carrier_freq = tracker_channel->carrier_freq_at_lock;
      float doppler_min = code_to_sv_doppler_min(mesid.code) +
                          code_to_tcxo_doppler_min(mesid.code);
      float doppler_max = code_to_sv_doppler_max(mesid.code) +
                          code_to_tcxo_doppler_max(mesid.code);
      if ((carrier_freq < doppler_min) || (carrier_freq > doppler_max)) {
        log_error_mesid(
            mesid, "Acq: bogus carr freq: %lf. Rejected.", carrier_freq);
      } else {
        /* FIXME other constellations/bands */
        acq->score[ACQ_HINT_PREV_TRACK] = SCORE_TRACK;
        acq->dopp_hint_low = MAX(carrier_freq - ACQ_FULL_CF_STEP, doppler_min);
        acq->dopp_hint_high = MIN(carrier_freq + ACQ_FULL_CF_STEP, doppler_max);
      }
    }
  }

  if (code_to_constellation(mesid.code) == CONSTELLATION_GLO) {
    bool glo_health_decoded = (0 != (flags & TRACKER_FLAG_GLO_HEALTH_DECODED));
    if (glo_health_decoded && (GLO_SV_UNHEALTHY == tracker_channel->health)) {
      acq->state = ACQ_PRN_UNHEALTHY;
      glo_acq_timer[tracker_channel->glo_orbit_slot].status = acq;
      /* store system time when GLO channel dropped */
      piksi_systime_get(&glo_acq_timer[tracker_channel->glo_orbit_slot].tick);
    } else {
      acq->state = ACQ_PRN_ACQUIRING;
    }
  } else {
    acq->state = ACQ_PRN_ACQUIRING;
  }

  /* Finally disable the decoder and tracking channels */
  decoder_channel_disable(tracker_channel->nap_channel);
  tracker_channel_disable(tracker_channel->nap_channel);
}

/**
 * Check if leap second event is taking place within the next two seconds.
 *
 * In case of a leap second event
 * all GLONASS satellites will be dropped from tracking.
 * After the leap second event the satellites will be re-acquired,
 * and new time will be decoded.
 *
 * \return true if leap second event is imminent, false otherwise.
 */
static bool leap_second_is_imminent(void) {
  /* Check if GPS time is known.
   * If GPS time is not known,
   * leap second event cannot be detected. */
  time_quality_t tq = get_time_quality();
  if (TIME_UNKNOWN == tq) {
    return false;
  }

  bool leap_second_event = false;
  utc_params_t utc_params;
  gps_time_t gps_time = get_current_gps_time();

  /* is_leap_second_event() returns true within 1 second of the event.
   * Thus, add 1 second to current GPS time.
   * This way the leap second event will be flagged 2 seconds beforehand,
   * and GLO satellites will be dropped in time.
   */
  add_secs(&gps_time, 1.0);
  /* Check if utc parameters are available */
  if (NDB_ERR_NONE == ndb_utc_params_read(&utc_params, NULL)) {
    leap_second_event = is_leap_second_event(&gps_time, &utc_params);
  } else {
    leap_second_event = is_leap_second_event(&gps_time, NULL);
  }
  return leap_second_event;
}

/** Disable any tracking channel that has errored, too weak, lost phase lock
 * or bit sync, or is flagged as cross-correlation, etc.
 * Keep tracking unhealthy (except GLO) and low-elevation satellites for
 * cross-correlation purposes. */
void sanitize_trackers(void) {
  const u64 now_ms = timing_getms();
  bool leap_second_event = leap_second_is_imminent();

  /* Clear GLO satellites TOW cache if it is leap second event */
  if (leap_second_event) {
    track_sid_db_clear_glo_tow();
  }

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    /*! Addressing the problem where we try to disable a channel that is
     * not in `STATE_ENABLED` in the first place. It remains to check
     * why `TRACKING_CHANNEL_FLAG_ACTIVE` might not be effective here?
     * */
    tracker_channel_t *tracker_channel = tracker_channel_get(i);

    state_t state = tracker_channel->state;
    COMPILER_BARRIER();
    if (STATE_ENABLED != state) {
      continue;
    }

    u32 flags = tracker_channel->flags;
    me_gnss_signal_t mesid = tracker_channel->mesid;

    /* Skip channels that aren't in use */
    if (0 == (flags & TRACKER_FLAG_ACTIVE)) {
      continue;
    }

    /* Drop GLO satellites if it is leap second event */
    constellation_t constellation = mesid_to_constellation(mesid);
    if (leap_second_event && (CONSTELLATION_GLO == constellation)) {
      drop_channel(tracker_channel, CH_DROP_REASON_LEAP_SECOND);
      continue;
    }

    /* Has an error occurred? */
    if (0 != (flags & TRACKER_FLAG_ERROR)) {
      drop_channel(tracker_channel, CH_DROP_REASON_ERROR);
      continue;
    }

    /* Is tracking masked? */
    u16 global_index = mesid_to_global_index(mesid);
    if (track_mask[global_index]) {
      drop_channel(tracker_channel, CH_DROP_REASON_MASKED);
      continue;
    }

    /* Do we have a large measurement outlier? */
    if (flags & TRACKER_FLAG_OUTLIER) {
      drop_channel(tracker_channel, CH_DROP_REASON_OUTLIER);
      continue;
    }

    /* Give newly-initialized channels a chance to converge. */
    u32 age_ms = now_ms - tracker_channel->init_timestamp_ms;
    u32 wait_ms = code_requires_direct_acq(mesid.code)
                      ? TRACK_INIT_FROM_ACQ_MS
                      : TRACK_INIT_FROM_HANDOVER_MS;
    if (age_ms < wait_ms) {
      continue;
    }

    u32 update_delay_ms = now_ms - tracker_channel->update_timestamp_ms;
    if (update_delay_ms > NAP_CORR_LENGTH_MAX_MS) {
      drop_channel(tracker_channel, CH_DROP_REASON_NO_UPDATES);
      continue;
    }

    /* Do we not have nav bit sync yet? */
    if (0 == (flags & TRACKER_FLAG_BIT_SYNC)) {
      drop_channel(tracker_channel, CH_DROP_REASON_NO_BIT_SYNC);
      continue;
    }

    /* PLL/FLL pessimistic lock detector "unlocked" for a while? */
    u32 unlocked_ms = 0;
    if ((0 == (flags & TRACKER_FLAG_HAS_PLOCK)) &&
        (0 == (flags & TRACKER_FLAG_HAS_FLOCK))) {
      unlocked_ms = update_count_diff(tracker_channel,
                                      &tracker_channel->ld_pess_change_count);
    }
    if (unlocked_ms > TRACK_DROP_UNLOCKED_MS) {
      drop_channel(tracker_channel, CH_DROP_REASON_NO_PLOCK);
      continue;
    }

    /* CN0 below threshold for a while? */
    u32 cn0_drop_ms = update_count_diff(
        tracker_channel, &tracker_channel->cn0_above_drop_thres_count);
    if (cn0_drop_ms > TRACK_DROP_CN0_MS) {
      drop_channel(tracker_channel, CH_DROP_REASON_LOW_CN0);
      continue;
    }

    /* Do we have confirmed cross-correlation? */
    if (0 != (flags & TRACKER_FLAG_XCORR_CONFIRMED)) {
      drop_channel(tracker_channel, CH_DROP_REASON_XCORR);
      continue;
    }

    /* Drop L2CL if the half-cycle ambiguity has been resolved. */
    if (0 != (flags & TRACKER_FLAG_L2CL_AMBIGUITY_RESOLVED)) {
      drop_channel(tracker_channel, CH_DROP_REASON_L2CL_SYNC);
      continue;
    }

    /* Drop GLO if the SV is unhealthy */
    if (is_glo_sid(mesid)) {
      bool glo_health_decoded =
          (0 != (flags & TRACKER_FLAG_GLO_HEALTH_DECODED));
      if (glo_health_decoded && (GLO_SV_UNHEALTHY == tracker_channel->health)) {
        drop_channel(tracker_channel, CH_DROP_REASON_SV_UNHEALTHY);
        continue;
      }
    }

    /* Drop channel if signal was excluded by RAIM */
    if (0 != (flags & TRACKER_FLAG_RAIM_EXCLUSION)) {
      drop_channel(tracker_channel, CH_DROP_REASON_RAIM);
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
 * \param[out] ctrl_info Optional destination for tracker controller
 * information.
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
static u32 get_tracking_channel_flags_info(
    u8 i,
    tracking_channel_info_t *info,
    tracking_channel_time_info_t *time_info,
    tracking_channel_freq_info_t *freq_info,
    tracking_channel_ctrl_info_t *ctrl_info,
    tracking_channel_misc_info_t *misc_info) {
  tracking_channel_info_t tmp_info;
  tracking_channel_time_info_t tmp_time_info;

  if (NULL == info) {
    info = &tmp_info;
  }
  if (NULL == time_info) {
    time_info = &tmp_time_info;
  }

  tracking_channel_get_values(i,
                              info,      /* Generic info */
                              time_info, /* Timers */
                              freq_info, /* Frequencies */
                              ctrl_info, /* Loop controller values */
                              misc_info);

  return info->flags;
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
                        double *carrier_phase_offset) {
  /* compute the pseudorange for this signal */
  double raw_pseudorange;
  bool ret = tracking_channel_calc_pseudorange(ref_tc, meas, &raw_pseudorange);
  if (ret) {
    /* We don't want to adjust for the recevier clock drift,
     * so we need to calculate an estimate of that before we
     * calculate the carrier phase offset */
    gps_time_t receiver_time = napcount2rcvtime(ref_tc);
    gps_time_t gps_time = napcount2gpstime(ref_tc);

    double rcv_clk_error = gpsdifftime(&gps_time, &receiver_time);

    double phase = (sid_to_carr_freq(meas->sid) *
                    (raw_pseudorange / GPS_C - rcv_clk_error));

    /* initialize the carrier phase offset with the pseudorange measurement */
    /* NOTE: CP sign flip - change the plus sign below */
    *carrier_phase_offset = round(meas->carrier_phase + phase);

    if ((0 != (info->flags & TRACKER_FLAG_HAS_PLOCK)) &&
        (0 != (info->flags & TRACKER_FLAG_CN0_SHORT))) {
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
 * \param[in] mesid ME SID
 *
 * \return Channel measurement flags
 */
static chan_meas_flags_t compute_meas_flags(u32 flags,
                                            bool phase_offset_ok,
                                            const me_gnss_signal_t mesid) {
  chan_meas_flags_t meas_flags = 0;

  if (0 != (flags & TRACKER_FLAG_PLL_USE)) {
    /* PLL is in use. */
    if (phase_offset_ok) {
      if ((0 != (flags & TRACKER_FLAG_HAS_PLOCK)) &&
          (0 != (flags & TRACKER_FLAG_CARRIER_PHASE_OFFSET))) {
        meas_flags |= CHAN_MEAS_FLAG_PHASE_VALID;

        /* Make sense to set half cycle known flag when carrier phase is valid
         */
        if (0 != (flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) {
          /* Bit polarity is known */
          meas_flags |= CHAN_MEAS_FLAG_HALF_CYCLE_KNOWN;
        }
      }

      /* sanity check */
      if ((flags & TRACKER_FLAG_BIT_POLARITY_KNOWN) &&
          !(flags & TRACKER_FLAG_HAS_PLOCK)) {
        /* Somehow we managed to decode TOW when phase lock lost.
         * This should not happen, so print out warning. */
        log_warn_mesid(mesid, "Half cycle known, but no phase lock!");
      }
    }
    /* If PLL is not in pessimistic lock and the tracker
       is not going to re-lock on a signal, then both code and
       doppler measurements can go really wrong before tracker switches to
       FLL mode. So we do not want to report them in such case. */
    bool pll_in_lock = (0 != (flags & TRACKER_FLAG_HAS_PLOCK));
    if (pll_in_lock) {
      meas_flags |= CHAN_MEAS_FLAG_CODE_VALID;
      meas_flags |= CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID;
    }
  } else if (0 != (flags & TRACKER_FLAG_FLL_USE)) {
    /* If FLL is not in pessimistic lock and the tracker
       is not going to re-lock on a signal, then both code and
       doppler measurements can go really wrong before tracker is terminated
       by low CN0 or no lock for too long condition.
       So we do not want to report them in such case. */
    bool fll_in_lock = (0 != (flags & TRACKER_FLAG_HAS_FLOCK));
    if (fll_in_lock) {
      meas_flags |= CHAN_MEAS_FLAG_CODE_VALID;
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
 * container, and the ephemeris from NDB if it is available.
 *
 * Additionally, the method computes initial carrier phase offset if it has
 * not been yet available and feeds it back to tracker.
 *
 * \param[in]  i      Tracking channel number.
 * \param[in]  ref_tc Reference time [ticks]
 * \param[out] meas   Container for measurement data.
 * \param[out] ephe   Container for ephemeris
 *
 * \return Flags
 */
u32 get_tracking_channel_meas(u8 i,
                              u64 ref_tc,
                              channel_measurement_t *meas,
                              ephemeris_t *ephe) {
  u32 flags = 0;                          /* Result */
  tracking_channel_info_t info;           /* Container for generic info */
  tracking_channel_freq_info_t freq_info; /* Container for measurements */
  tracking_channel_time_info_t time_info; /* Container for time info */
  tracking_channel_misc_info_t misc_info; /* Container for measurements */

  memset(meas, 0, sizeof(*meas));

  /* Load information from tracker: info locks */
  flags = get_tracking_channel_flags_info(i,           /* Channel index */
                                          &info,       /* General */
                                          &time_info,  /* Time info */
                                          &freq_info,  /* Freq info */
                                          NULL,        /* Ctrl info */
                                          &misc_info); /* Misc info */

  constellation_t constellation = code_to_constellation(info.mesid.code);
  if ((CONSTELLATION_GLO == constellation) &&
      !glo_slot_id_is_valid(info.glo_orbit_slot)) {
    memset(meas, 0, sizeof(*meas));
    return flags | TRACKER_FLAG_MASKED;
  }

  if ((0 != (flags & TRACKER_FLAG_ACTIVE)) &&
      (0 != (flags & TRACKER_FLAG_CONFIRMED)) &&
      (0 == (flags & TRACKER_FLAG_ERROR)) &&
      (0 == (flags & TRACKER_FLAG_XCORR_SUSPECT))) {
    gnss_signal_t sid = mesid2sid(info.mesid, info.glo_orbit_slot);
    ndb_op_code_t res = ndb_ephemeris_read(sid, ephe);

    /* TTFF shortcut: accept also unconfirmed ephemeris candidate when there
     * is no confirmed candidate */
    if ((NDB_ERR_NONE != res) && (NDB_ERR_UNCONFIRMED_DATA != res)) {
      ephe = NULL;
    }

    /* Load information from SID cache */
    flags |= get_tracking_channel_sid_flags(sid, info.tow_ms, ephe);

    tracking_channel_measurement_get(
        ref_tc, &info, &freq_info, &time_info, &misc_info, meas);

    /* Adjust for half phase ambiguity */
    if ((0 != (info.flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) &&
        (0 != (info.flags & TRACKER_FLAG_BIT_INVERTED))) {
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
    if ((TIME_FINE <= get_time_quality()) && (0.0 == carrier_phase_offset) &&
        (0 != (flags & TRACKER_FLAG_PLL_USE)) &&
        (0 != (flags & TRACKER_FLAG_HAS_PLOCK)) &&
        (0 != (flags & TRACKER_FLAG_TOW_VALID))) {
      cpo_ok = compute_cpo(ref_tc, &info, meas, &carrier_phase_offset);
    }
    if (0.0 != carrier_phase_offset) {
      flags |= TRACKER_FLAG_CARRIER_PHASE_OFFSET;
      meas->carrier_phase -= carrier_phase_offset;
    }
    meas->flags = compute_meas_flags(flags, cpo_ok, info.mesid);
    meas->elevation = (double)sv_elevation_degrees_get(meas->sid);
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
void get_tracking_channel_ctrl_params(u8 i, tracking_ctrl_params_t *pparams) {
  tracking_channel_ctrl_info_t tmp;

  tracking_channel_get_values(i,
                              NULL,  /* Generic info */
                              NULL,  /* Timers */
                              NULL,  /* Frequencies */
                              &tmp,  /* Loop controller values */
                              NULL); /* Misc info */
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
 * \param[in]  sid  Signal identifier.
 * \param[in]  tow_ms ToW in milliseconds. Can be #TOW_UNKNOWN
 * \param[in]  pephe  Pointer to ephemeris, or NULL if not available
 *
 * \return Flags, computed from ephemeris and other sources.
 */
u32 get_tracking_channel_sid_flags(const gnss_signal_t sid,
                                   s32 tow_ms,
                                   const ephemeris_t *pephe) {
  u32 flags = 0;

  /* Satellite elevation is above the solution mask. */
  if (sv_elevation_degrees_get(sid) >= solution_elevation_mask) {
    flags |= TRACKER_FLAG_ELEVATION;
  }

  gps_time_t t = {.wn = WN_UNKNOWN, .tow = 1e-3 * tow_ms};

  /* Ephemeris must be valid, not stale. Satellite must be healthy.
     This also acts as a sanity check on the channel TOW.*/
  if ((NULL != pephe) && (TOW_UNKNOWN != tow_ms) &&
      ephemeris_valid(pephe, &t)) {
    flags |= TRACKER_FLAG_HAS_EPHE;

    if (signal_healthy(
            pephe->valid, pephe->health_bits, pephe->ura, sid.code)) {
      flags |= TRACKER_FLAG_HEALTHY;
    }
  }

  constellation_t constellation = sid_to_constellation(sid);
  if ((CONSTELLATION_GPS == constellation) && shm_navigation_suitable(sid)) {
    flags |= TRACKER_FLAG_NAV_SUITABLE;
  } else if ((CONSTELLATION_GLO == constellation) &&
             (flags & TRACKER_FLAG_HEALTHY)) {
    flags |= TRACKER_FLAG_NAV_SUITABLE;
  }

  return flags;
}

/** Checks if tracking can be started for a given mesid.
 *
 * \param mesid ME signal ID to check.
 * \retval true mesid tracking can be started.
 * \retval false mesid tracking cannot be started.
 */
bool tracking_startup_ready(const me_gnss_signal_t mesid) {
  u16 global_index = mesid_to_global_index(mesid);
  acq_status_t *acq = &acq_status[global_index];
  return (acq->state == ACQ_PRN_ACQUIRING) && (!acq->masked);
}

/**
 * Checks if the tracker is running.
 *
 * \param[in] mesid GNSS ME signal ID to check.
 *
 * \retval true  Tracker for \a mesid is in running state
 * \retval false Tracker for \a mesid is not in running state
 */
bool tracking_is_running(const me_gnss_signal_t mesid) {
  u16 global_index = mesid_to_global_index(mesid);
  acq_status_t *acq = &acq_status[global_index];
  return (acq->state == ACQ_PRN_TRACKING);
}

/** Check if a startup request for a mesid is present in a
 *  tracking startup FIFO.
 *
 * \param fifo  tracking_startup_fifo_t struct to use.
 * \param mesid me_gnss_signal_t to use.
 *
 * \return true if the mesid is present, false otherwise.
 */
static bool tracking_startup_fifo_mesid_present(
    const tracking_startup_fifo_t *fifo, const me_gnss_signal_t mesid) {
  tracking_startup_fifo_index_t read_index = fifo->read_index;
  tracking_startup_fifo_index_t write_index = fifo->write_index;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  while (read_index != write_index) {
    const tracking_startup_params_t *p =
        &fifo->elements[read_index++ & TRACKING_STARTUP_FIFO_INDEX_MASK];
    if (mesid_is_equal(p->mesid, mesid)) {
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
u8 tracking_startup_request(const tracking_startup_params_t *startup_params) {
  u8 result = 2;
  if (chMtxTryLock(&tracking_startup_mutex)) {
    if (!tracking_startup_fifo_mesid_present(&tracking_startup_fifo,
                                             startup_params->mesid)) {
      if (tracking_startup_fifo_write(&tracking_startup_fifo, startup_params)) {
        result = 0;
      } else {
        log_warn("Tracking startup FIFO full");
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
static void manage_tracking_startup(void) {
  tracking_startup_params_t startup_params;
  while (tracking_startup_fifo_read(&tracking_startup_fifo, &startup_params)) {
    acq_status_t *acq =
        &acq_status[mesid_to_global_index(startup_params.mesid)];

    /* Make sure the SID is not already tracked and healthy */
    if (acq->state == ACQ_PRN_TRACKING ||
        (acq->state == ACQ_PRN_UNHEALTHY && is_glo_sid(acq->mesid))) {
      continue;
    }

    /* Make sure a tracking channel and a decoder channel are available */
    u8 chan = manage_track_new_acq(startup_params.mesid);
    if (chan == MANAGE_NO_CHANNELS_FREE) {
      if (code_requires_direct_acq(acq->mesid.code)) {
        float doppler_min = code_to_sv_doppler_min(acq->mesid.code) +
                            code_to_tcxo_doppler_min(acq->mesid.code);
        float doppler_max = code_to_sv_doppler_max(acq->mesid.code) +
                            code_to_tcxo_doppler_max(acq->mesid.code);

        /* No channels are free to accept our new satellite :( */
        /* TODO: Perhaps we can try to warm start this one
         * later using another fine acq.
         */
        if (startup_params.cn0_init > ACQ_RETRY_THRESHOLD) {
          acq->score[ACQ_HINT_PREV_ACQ] =
              SCORE_ACQ + (startup_params.cn0_init - ACQ_THRESHOLD);
          /* Check that reported carrier frequency is within Doppler bounds */
          float freq = startup_params.carrier_freq;
          if (freq < doppler_min) {
            freq = doppler_min;
          } else if (freq > doppler_max) {
            freq = doppler_max;
          }
          acq->dopp_hint_low = MAX(freq - ACQ_FULL_CF_STEP, doppler_min);
          acq->dopp_hint_high = MIN(freq + ACQ_FULL_CF_STEP, doppler_max);
        }
      }
      log_debug("No free tracking channel available.");
      continue;
    }

    /* Change state to TRACKING */
    acq->state = ACQ_PRN_TRACKING;

    /* Start the tracking channel */
    if (!tracker_channel_init(chan,
                              startup_params.mesid,
                              startup_params.glo_slot_id,
                              startup_params.sample_count,
                              startup_params.code_phase,
                              startup_params.carrier_freq,
                              startup_params.chips_to_correlate,
                              startup_params.cn0_init)) {
      log_error("tracker channel init failed");
      /* If starting of a channel fails, change state to ACQUIRING */
      acq->state = ACQ_PRN_ACQUIRING;
      continue;
    }

    /* TODO: Initialize elevation from ephemeris if we know it precisely */

    /* Start the decoder channel if needed */
    if (code_requires_decoder(startup_params.mesid.code) &&
        !decoder_channel_init(chan, startup_params.mesid)) {
      log_error("decoder channel init failed");
    }
  }
}

/** Initialize a tracking_startup_fifo_t struct.
 *
 * \param fifo        tracking_startup_fifo_t struct to use.
 */
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo) {
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
static bool tracking_startup_fifo_write(
    tracking_startup_fifo_t *fifo, const tracking_startup_params_t *element) {
  if (TRACKING_STARTUP_FIFO_LENGTH(fifo) < TRACKING_STARTUP_FIFO_SIZE) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    MEMCPY_S(
        &fifo->elements[fifo->write_index & TRACKING_STARTUP_FIFO_INDEX_MASK],
        sizeof(tracking_startup_params_t),
        element,
        sizeof(tracking_startup_params_t));
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
                                       tracking_startup_params_t *element) {
  if (TRACKING_STARTUP_FIFO_LENGTH(fifo) > 0) {
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    MEMCPY_S(
        element,
        sizeof(tracking_startup_params_t),
        &fifo->elements[fifo->read_index & TRACKING_STARTUP_FIFO_INDEX_MASK],
        sizeof(tracking_startup_params_t));
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->read_index++;
    return true;
  }

  return false;
}

/** Checks if SV is tracked
 *
 * \param mesid ME signal ID to check.
 * \retval true sid is tracked
 * \retval false sid is not tracked
 */
bool mesid_is_tracked(const me_gnss_signal_t mesid) {
  /* This function is used in reacquisition which runs in
     the same thread as acquisition.
     Revisit this if there will be any better way to check
     if SV is in track. */
  u16 global_index = mesid_to_global_index(mesid);
  acq_status_t *acq = &acq_status[global_index];
  return acq->state == ACQ_PRN_TRACKING;
}

/** Checks if GLONASS enabled
 *
 * @return true if GLONASS enabled, otherwise false
 */
bool is_glo_enabled(void) { return glo_enabled; }

/** \} */
