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

#include "manage.h"

#include <assert.h>
#include <inttypes.h>
#include <libsbp/piksi.h>
#include <math.h>
#include <starling/starling.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/almanac.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/glo_map.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>
#include <swiftnav/signal.h>

#include "board/nap/track_channel.h"
#include "decode/decode.h"
#include "dum/dum.h"
#include "ephemeris/ephemeris.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "hal/piksi_systime.h"
#include "main/main.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "position/position.h"
#include "reacq/reacq_manage.h"
#include "reacq/search_manager_utils.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings_client.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "soft_macq/soft_macq_main.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_sid_db.h"
#include "track/track_state.h"
#include "track/track_utils.h"

/** \defgroup manage Manage
 * Manage acquisition and tracking.
 * Manage how acquisition searches are performed, with data from almanac if
 * available. Transition from acquisition search to initialization of an
 * available tracking channel when a satellite is successfully found. Disable
 * tracking channels that have lost lock on their satellites.
 * \{ */

/** Status of acquisition for a particular ME SID. */
typedef struct {
  enum {
    ACQ_PRN_SKIP = 0,
    ACQ_PRN_ACQUIRING,
    ACQ_PRN_TRACKING,
    ACQ_PRN_UNHEALTHY
  } state;                /**< Management status of signal. */
  bool masked;            /**< Prevent acquisition. */
  float dopp_hint_low;    /**< Low bound of doppler search hint. */
  float dopp_hint_high;   /**< High bound of doppler search hint. */
  me_gnss_signal_t mesid; /**< ME signal identifier. */
} acq_status_t;

static acq_status_t acq_status[PLATFORM_ACQ_TRACK_COUNT];
static bool track_mask[ARRAY_SIZE(acq_status)];

typedef struct {
  bool visible; /** Visible flag */
  bool known;   /** Known flag */
} sm_glo_sv_vis_t;

/* The array keeps latest visibility flags of each GLO SV */
static sm_glo_sv_vis_t glo_sv_vis[NUM_SATS_GLO] = {0};

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

/* Refer to SBAS MOPS section A.4.4.1 */
#define ACQ_SBAS_MSG0_TIMEOUT_SEC (MINUTE_SECS)

/* BDS2 & GAL Unhealthy time out */
#define ACQ_UNHEALTHY_TIMEOUT_SEC (5 * MINUTE_SECS)

#define MANAGE_ACQ_THREAD_PRIORITY (LOWPRIO + 1)
#define MANAGE_ACQ_THREAD_STACK (32 * 1024)

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

typedef struct cons_cfg_s {
  const char *name;
  bool enabled;
  const bool supported;
  bool (*is_applicable)(const code_t code);
  bool (*sid_active)(const me_gnss_signal_t mesid);
} cons_cfg_t;

static cons_cfg_t cons_cfg[CONSTELLATION_COUNT] = {
    [CONSTELLATION_GPS] =
        {
            .name = "GPS",
            .enabled = true,
            .supported = true,
            .is_applicable = is_gps,
            .sid_active = NULL,
        },
    [CONSTELLATION_SBAS] =
        {
            .name = "SBAS",
            .enabled = CODE_SBAS_L1CA_SUPPORT,
            .supported = CODE_SBAS_L1CA_SUPPORT,
            .is_applicable = is_sbas,
            .sid_active = sbas_active,
        },
    [CONSTELLATION_GLO] =
        {
            .name = "GLONASS",
            .enabled = (CODE_GLO_L1OF_SUPPORT || CODE_GLO_L2OF_SUPPORT),
            .supported = (CODE_GLO_L1OF_SUPPORT || CODE_GLO_L2OF_SUPPORT),
            .is_applicable = is_glo,
            .sid_active = NULL,
        },
    [CONSTELLATION_BDS] =
        {
            .name = "BeiDou",
            .enabled = (CODE_BDS2_B1_SUPPORT || CODE_BDS2_B2_SUPPORT),
            .supported = (CODE_BDS2_B1_SUPPORT || CODE_BDS2_B2_SUPPORT),
            .is_applicable = is_bds2,
            .sid_active = bds_active,
        },
    [CONSTELLATION_QZS] =
        {
            .name = "QZSS",
            .enabled = (CODE_QZSS_L1CA_SUPPORT || CODE_QZSS_L2C_SUPPORT),
            .supported = (CODE_QZSS_L1CA_SUPPORT || CODE_QZSS_L2C_SUPPORT),
            .is_applicable = is_qzss,
            .sid_active = qzss_active,
        },
    [CONSTELLATION_GAL] =
        {
            .name = "Galileo",
            .enabled = (CODE_GAL_E1_SUPPORT || CODE_GAL_E7_SUPPORT),
            .supported = (CODE_GAL_E1_SUPPORT || CODE_GAL_E7_SUPPORT),
            .is_applicable = is_gal,
            .sid_active = gal_active,
        },
};

typedef struct {
  piksi_systime_t tick; /**< Time when SV was detected as unhealthy */
  acq_status_t *status; /**< Pointer to acq status for the SV */
} acq_timer_t;

/* The array keeps time when GLO SV was detected as unhealthy */
static acq_timer_t glo_acq_timer[NUM_SATS_GLO] = {0};

/* The array keeps time when SBAS SV was detected as unhealthy. */
static acq_timer_t sbas_acq_timer[NUM_SATS_SBAS] = {0};

/* The array keeps time when BDS2 SV was detected as unhealthy */
static acq_timer_t bds2_acq_timer[NUM_SATS_BDS] = {0};

/* The array keeps time when GAL SV was detected as unhealthy. */
static acq_timer_t gal_acq_timer[NUM_SATS_GAL] = {0};

/* The array keeps time when QZS SV was detected as unhealthy. */
static acq_timer_t qzs_acq_timer[NUM_SATS_QZS] = {0};

static u8 manage_track_new_acq(me_gnss_signal_t mesid);
static void manage_acq(void);

static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo);
static bool tracking_startup_fifo_write(
    tracking_startup_fifo_t *fifo, const tracking_startup_params_t *element);
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element);
static u32 get_tracking_channel_sid_flags(const gnss_signal_t sid,
                                          s32 tow_ms,
                                          const ephemeris_t *pephe);

static sbp_msg_callbacks_node_t almanac_callback_node;
static void almanac_callback(u16 sender_id,
                             u8 len,
                             u8 msg[], /* NOLINT */
                             void *context) {
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
    if (IS_GLO(sid)) {
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

/**
 * The function calculates and stores visibility flags for all GLO SV
 *
 * Since work time of Runge-Kutta algorithm depends on GLO SV position
 * calculation period, due to iteration number
 * (see modeling https://github.com/swift-nav/exafore_planning/issues/681)
 * we continuously calculate the position.
 */
static void calc_all_glo_visibility_flags(void) {
  if (!is_glo_enabled()) {
    return;
  }

  for (u16 glo_sat = 1; glo_sat <= NUM_SATS_GLO; glo_sat++) {
    gnss_signal_t glo_sid = construct_sid(CODE_GLO_L1OF, glo_sat);
    bool visible, known;
    sm_get_visibility_flags(glo_sid, &visible, &known);
    glo_sv_vis[glo_sat - 1].visible = visible;
    glo_sv_vis[glo_sat - 1].known = known;
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

  while (true) {
    DO_EACH_MS(10000, calc_all_glo_visibility_flags());

    if (!had_fix) {
      /* do all this computation only if we never got a fix */
      last_good_fix_t lgf;
      bool have_fix = (ndb_lgf_read(&lgf) == NDB_ERR_NONE) &&
                      lgf.position_solution.valid &&
                      (POSITION_FIX == lgf.position_quality) &&
                      ((TIME_COARSE <= get_time_quality()));
      if (have_fix) {
        had_fix = true;
        log_info("Switching to re-acq mode");
      }
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

/* The function masks/unmasks all <constellation> satellites,
 * NOTE: this function does not check if SV is already masked or not */
static int cons_enable_notify(void *ctx) {
  cons_cfg_t *cfg = ctx;

  log_debug("%s status (1 - on, 0 - off): %u", cfg->name, cfg->enabled);

  if (cfg->enabled && !cfg->supported) {
    /* user tries enable constellation on the platform that does not support it
     */
    log_error("The platform does not support %s", cfg->name);
    cfg->enabled = false;
    return SETTINGS_WR_VALUE_REJECTED;
  }

  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (!cfg->is_applicable(acq_status[i].mesid.code)) {
      continue;
    }

    if (!cfg->enabled) {
      acq_status[i].masked = true;
      continue;
    }

    if (NULL != cfg->sid_active && !cfg->sid_active(acq_status[i].mesid)) {
      acq_status[i].masked = true;
      continue;
    }
    
    acq_status[i].masked = false;
  }

  return SETTINGS_WR_OK;
}

/* Update the solution elevation mask used by the ME and by Starling. */
static int solution_elevation_mask_notify(void *ctx) {
  (void)ctx;

  log_debug("Solution elevation mask: %f", solution_elevation_mask);

  starling_set_elevation_mask(solution_elevation_mask);

  return SETTINGS_WR_OK;
}

void manage_acq_setup() {
  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    me_gnss_signal_t mesid = mesid_from_global_index(i);
    acq_status[i].state = ACQ_PRN_ACQUIRING;

    if (code_requires_direct_acq(mesid.code)) {
      acq_status[i].dopp_hint_low = code_to_sv_doppler_min(mesid.code) +
                                    code_to_tcxo_doppler_min(mesid.code);
      acq_status[i].dopp_hint_high = code_to_sv_doppler_max(mesid.code) +
                                     code_to_tcxo_doppler_max(mesid.code);
    }
    acq_status[i].mesid = mesid;

    track_mask[i] = false;
  }

  SETTING(
      "acquisition", "almanacs_enabled", almanacs_enabled, SETTINGS_TYPE_BOOL);
  SETTING_NOTIFY_CTX("acquisition",
                     "glonass_acquisition_enabled",
                     cons_cfg[CONSTELLATION_GLO].enabled,
                     SETTINGS_TYPE_BOOL,
                     cons_enable_notify,
                     &cons_cfg[CONSTELLATION_GLO]);
  SETTING_NOTIFY_CTX("acquisition",
                     "sbas_acquisition_enabled",
                     cons_cfg[CONSTELLATION_SBAS].enabled,
                     SETTINGS_TYPE_BOOL,
                     cons_enable_notify,
                     &cons_cfg[CONSTELLATION_SBAS]);
  SETTING_NOTIFY_CTX("acquisition",
                     "bds2_acquisition_enabled",
                     cons_cfg[CONSTELLATION_BDS].enabled,
                     SETTINGS_TYPE_BOOL,
                     cons_enable_notify,
                     &cons_cfg[CONSTELLATION_BDS]);
  SETTING_NOTIFY_CTX("acquisition",
                     "qzss_acquisition_enabled",
                     cons_cfg[CONSTELLATION_QZS].enabled,
                     SETTINGS_TYPE_BOOL,
                     cons_enable_notify,
                     &cons_cfg[CONSTELLATION_QZS]);
  SETTING_NOTIFY_CTX("acquisition",
                     "galileo_acquisition_enabled",
                     cons_cfg[CONSTELLATION_GAL].enabled,
                     SETTINGS_TYPE_BOOL,
                     cons_enable_notify,
                     &cons_cfg[CONSTELLATION_GAL]);

  tracking_startup_fifo_init(&tracking_startup_fifo);

  sbp_register_cbk(SBP_MSG_ALMANAC, &almanac_callback, &almanac_callback_node);

  sbp_register_cbk(
      SBP_MSG_MASK_SATELLITE, &mask_sat_callback, &mask_sat_callback_node);

  chThdCreateStatic(wa_manage_acq_thread,
                    sizeof(wa_manage_acq_thread),
                    MANAGE_ACQ_THREAD_PRIORITY,
                    manage_acq_thread,
                    NULL);
}

static acq_status_t *choose_acq_sat(void) {
  static u32 sat_idx = 0;

  if ((!code_requires_direct_acq(acq_status[sat_idx].mesid.code)) ||
      (CODE_SBAS_L1CA == acq_status[sat_idx].mesid.code) ||
      (acq_status[sat_idx].state != ACQ_PRN_ACQUIRING) ||
      (acq_status[sat_idx].masked)) {
    log_debug_mesid(acq_status[sat_idx].mesid, "skipped");

    sat_idx = (sat_idx + 1) % ARRAY_SIZE(acq_status);
    return NULL;
  }

  u32 this = sat_idx;
  log_debug_mesid(acq_status[sat_idx].mesid, "chosen");

  sat_idx = (sat_idx + 1) % ARRAY_SIZE(acq_status);
  return &acq_status[this];
}

/** Manages acquisition searches and starts tracking channels after successful
 * acquisitions. */
static void manage_acq(void) {
  /* Decide which SID to try and then start it acquiring. */
  acq_status_t *acq = choose_acq_sat();
  if (acq == NULL) {
    return;
  }

  assert((CODE_GPS_L1CA == acq->mesid.code) ||
         (CODE_GLO_L1OF == acq->mesid.code) ||
         (CODE_SBAS_L1CA == acq->mesid.code) ||
         (CODE_BDS2_B1 == acq->mesid.code) ||
         (CODE_QZS_L1CA == acq->mesid.code) ||
         (CODE_GAL_E1B == acq->mesid.code));

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
    acq_result_send(
        acq->mesid, acq_result.cn0, acq_result.cp, acq_result.df_hz);

    if (acq_result.cn0 < ACQ_THRESHOLD) {
      /* Didn't find the satellite :( */
      /* Double the size of the doppler search space for next time. */
      float dilute = (acq->dopp_hint_high - acq->dopp_hint_low) / 2;
      acq->dopp_hint_high = MIN(acq->dopp_hint_high + dilute, doppler_max);
      acq->dopp_hint_low = MAX(acq->dopp_hint_low - dilute, doppler_min);
      return;
    }

    me_gnss_signal_t mesid_trk = acq->mesid;
    float cp = acq_result.cp;
    float df_hz = acq_result.df_hz;

    tracking_startup_params_t tracking_startup_params = {
        .mesid = mesid_trk,
        .glo_slot_id = GLO_ORBIT_SLOT_UNKNOWN,
        .sample_count = acq_result.sample_count,
        .doppler_hz = df_hz,
        .code_phase = cp,
        .chips_to_correlate = code_to_chip_count(mesid_trk.code),
        .cn0_init = acq_result.cn0};

    tracking_startup_request(&tracking_startup_params);
  }
}

/** Send results of an acquisition to the host.
 *
 * \param mesid ME SID of the acquisition
 * \param cn0 Carrier to noise ratio of best point from acquisition.
 * \param cp  Code phase of best point.
 * \param df_hz  Doppler frequency of best point.
 */
void acq_result_send(const me_gnss_signal_t mesid,
                     float cn0,
                     float cp,
                     float df_hz) {
  msg_acq_result_t acq_result_msg;
  /* TODO GLO: Handle GLO orbit slot properly. */
  if (IS_GLO(mesid)) {
    return;
  }
  acq_result_msg.sid = sid_to_sbp(mesid2sid(mesid, GLO_ORBIT_SLOT_UNKNOWN));
  acq_result_msg.cn0 = cn0;
  acq_result_msg.cp = cp;
  acq_result_msg.cf = df_hz;

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
    if (code_requires_decoder(mesid.code) && tracker_available(i, mesid) &&
        decoder_channel_available(i, mesid)) {
      return i;
    }
    if (!code_requires_decoder(mesid.code) && tracker_available(i, mesid)) {
      return i;
    }
  }

  return MANAGE_NO_CHANNELS_FREE;
}

static void revert_expired_unhealthiness(acq_timer_t *timer,
                                         size_t size,
                                         u32 timeout_s) {
  for (u8 i = 0; i < size; i++) {
    if (NULL == timer[i].status) {
      continue;
    }
    if (ACQ_PRN_UNHEALTHY != timer[i].status->state) {
      continue;
    }
    if (piksi_systime_elapsed_since_s(&timer[i].tick) <= timeout_s) {
      continue;
    }

    timer[i].status->state = ACQ_PRN_ACQUIRING;
    log_info_mesid(timer[i].status->mesid, "is back to aquisition");
  }
}

/** Check SV unhealthy flags and clear after constellation specific timeout */
void check_clear_unhealthy(void) {
  if (is_glo_enabled()) {
    revert_expired_unhealthiness(
        glo_acq_timer, ARRAY_SIZE(glo_acq_timer), ACQ_GLO_EPH_VALID_TIME_SEC);
  }
  if (is_sbas_enabled()) {
    revert_expired_unhealthiness(
        sbas_acq_timer, ARRAY_SIZE(sbas_acq_timer), ACQ_SBAS_MSG0_TIMEOUT_SEC);
  }
  if (is_bds2_enabled()) {
    revert_expired_unhealthiness(
        bds2_acq_timer, ARRAY_SIZE(bds2_acq_timer), ACQ_UNHEALTHY_TIMEOUT_SEC);
  }
  if (is_galileo_enabled()) {
    revert_expired_unhealthiness(
        gal_acq_timer, ARRAY_SIZE(gal_acq_timer), ACQ_UNHEALTHY_TIMEOUT_SEC);
  }
  if (is_qzss_enabled()) {
    revert_expired_unhealthiness(
        qzs_acq_timer, ARRAY_SIZE(qzs_acq_timer), ACQ_UNHEALTHY_TIMEOUT_SEC);
  }
}

void me_settings_setup(void) {
  SETTING(
      "track", "elevation_mask", tracking_elevation_mask, SETTINGS_TYPE_FLOAT);
  SETTING_NOTIFY("solution",
                 "elevation_mask",
                 solution_elevation_mask,
                 SETTINGS_TYPE_FLOAT,
                 solution_elevation_mask_notify);
}

/**
 * Expose the elevation mask setting
 */
float get_solution_elevation_mask(void) { return solution_elevation_mask; }

/** Updates acq hints using last observed Doppler value */
void update_acq_hints(tracker_t *tracker) {
  me_gnss_signal_t mesid = tracker->mesid;
  if (!code_requires_direct_acq(mesid.code)) {
    return;
  }

  const u32 flags = tracker->flags;
  bool had_locks =
      (0 != (flags & (TRACKER_FLAG_HAD_PLOCK | TRACKER_FLAG_HAD_FLOCK)));
  if (!had_locks) {
    return;
  }

  u64 time_in_track_ms = tracker_timer_ms(&tracker->age_timer);
  bool long_in_track = time_in_track_ms > TRACK_REACQ_MS;
  if (!long_in_track) {
    return;
  }

  u64 unlocked_ms = tracker_timer_ms(&tracker->unlocked_timer);
  bool long_unlocked = unlocked_ms > TRACK_REACQ_MS;
  if (long_unlocked) {
    return;
  }

  bool was_xcorr = (flags & TRACKER_FLAG_DROP_CHANNEL) &&
                   (CH_DROP_REASON_XCORR == tracker->ch_drop_reason);
  if (was_xcorr) {
    return;
  }

  double doppler_hz = tracker->doppler_at_lock_hz;
  float doppler_min_hz =
      code_to_sv_doppler_min(mesid.code) + code_to_tcxo_doppler_min(mesid.code);
  float doppler_max_hz =
      code_to_sv_doppler_max(mesid.code) + code_to_tcxo_doppler_max(mesid.code);
  if ((doppler_hz < doppler_min_hz) || (doppler_hz > doppler_max_hz)) {
    log_error_mesid(
        mesid, "Acq: bogus doppler freq: %lf. Rejected.", doppler_hz);
  } else {
    acq_status_t *acq = &acq_status[mesid_to_global_index(mesid)];
    acq->dopp_hint_low = MAX(doppler_hz - ACQ_FULL_CF_STEP, doppler_min_hz);
    acq->dopp_hint_high = MIN(doppler_hz + ACQ_FULL_CF_STEP, doppler_max_hz);
  }
}

/**
 * Restores acquisition for a satellite that has been disposed.
 *
 * \return
 */
void restore_acq(const tracker_t *tracker) {
  me_gnss_signal_t mesid = tracker->mesid;
  acq_status_t *acq = &acq_status[mesid_to_global_index(mesid)];

  /* Now restore satellite acq */
  acq->state = ACQ_PRN_ACQUIRING;

  if (0 == (tracker->flags & TRACKER_FLAG_UNHEALTHY)) {
    return;
  }

  /* Set quarantine timer for unhealthy SVs */
  if (IS_GLO(mesid)) {
    if (tracker->glo_orbit_slot != GLO_ORBIT_SLOT_UNKNOWN) {
      /* GLO acq quarantine timer is only armed for GLO L1OF
         as it is the only direct acq GLO signal we care about in acq module */
      mesid = construct_mesid(CODE_GLO_L1OF, mesid.sat);
      acq = &acq_status[mesid_to_global_index(mesid)];
      acq->state = ACQ_PRN_UNHEALTHY;
      u16 index = tracker->glo_orbit_slot - 1;
      assert(index < ARRAY_SIZE(glo_acq_timer));
      glo_acq_timer[index].status = acq;
      piksi_systime_get(&glo_acq_timer[index].tick); /* channel drop time */
    }
  } else if (IS_SBAS(mesid)) {
    acq->state = ACQ_PRN_UNHEALTHY;
    u16 index = tracker->mesid.sat - SBAS_FIRST_PRN;
    assert(index < ARRAY_SIZE(sbas_acq_timer));
    sbas_acq_timer[index].status = acq;
    piksi_systime_get(&sbas_acq_timer[index].tick); /* channel drop time */
  } else if (IS_BDS2(mesid)) {
    mesid = construct_mesid(CODE_BDS2_B1, mesid.sat);
    acq = &acq_status[mesid_to_global_index(mesid)];
    acq->state = ACQ_PRN_UNHEALTHY;
    u16 index = tracker->mesid.sat - BDS_FIRST_PRN;
    assert(index < ARRAY_SIZE(bds2_acq_timer));
    bds2_acq_timer[index].status = acq;
    piksi_systime_get(&bds2_acq_timer[index].tick); /* channel drop time */
  } else if (IS_GAL(mesid)) {
    mesid = construct_mesid(CODE_GAL_E1B, mesid.sat);
    acq = &acq_status[mesid_to_global_index(mesid)];
    acq->state = ACQ_PRN_UNHEALTHY;
    u16 index = tracker->mesid.sat - GAL_FIRST_PRN;
    assert(index < ARRAY_SIZE(gal_acq_timer));
    gal_acq_timer[index].status = acq;
    piksi_systime_get(&gal_acq_timer[index].tick); /* channel drop time */
  } else if (IS_QZSS(mesid)) {
    mesid = construct_mesid(CODE_QZS_L1CA, mesid.sat);
    acq = &acq_status[mesid_to_global_index(mesid)];
    acq->state = ACQ_PRN_UNHEALTHY;
    u16 index = tracker->mesid.sat - QZS_FIRST_PRN;
    assert(index < ARRAY_SIZE(qzs_acq_timer));
    qzs_acq_timer[index].status = acq;
    piksi_systime_get(&qzs_acq_timer[index].tick); /* channel drop time */
  }
}

/** Get GLO SV visibility flags. Function simply copies previously calculated
 * visibility flags for GLO SV
 *
 * \param[in] sat GLO SV orbital slot
 * \param[out] visible is set if SV is visible. Valid only if known is set
 * \param[out] known set if SV is known visible or known invisible
 */
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known) {
  *visible = glo_sv_vis[sat - 1].visible;
  *known = glo_sv_vis[sat - 1].known;
}

/** Initiates the drop of all GLO signals from tracker on leap second event */
void drop_glo_signals_on_leap_second(void) {
  if (!leap_second_imminent()) {
    return;
  }
  track_sid_db_clear_glo_tow();
  tracker_set_leap_second_flag();
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
bool leap_second_imminent(void) {
  /* Check if GPS time is known.
   * If GPS time is not known,
   * leap second event cannot be detected. */
  time_quality_t tq = get_time_quality();
  if (TIME_UNKNOWN == tq) {
    return false;
  }

  bool leap_second_event = false;
  utc_params_t utc_params;
  gps_time_t gps_time = get_current_time();

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

/** Disable the tracking channel if it has errored, too weak, lost phase lock
 * or bit sync, or is flagged as cross-correlation, etc.
 * Keep tracking unhealthy (except GLO) and low-elevation satellites for
 * cross-correlation purposes. */
void sanitize_tracker(tracker_t *tracker) {
  /*! Addressing the problem where we try to disable a channel that is
   * not busy in the first place. It remains to check
   * why `TRACKING_CHANNEL_FLAG_ACTIVE` might not be effective here?
   * */
  if (!(tracker->busy)) {
    return;
  }

  u32 flags = tracker->flags;
  me_gnss_signal_t mesid = tracker->mesid;

  /* Skip channels that aren't in use */
  if (0 == (flags & TRACKER_FLAG_ACTIVE)) {
    return;
  }

  if (0 != (flags & TRACKER_FLAG_DROP_CHANNEL)) {
    tp_drop_channel(tracker, tracker->ch_drop_reason);
    return;
  }

  /* Is tracking masked? */
  u16 global_index = mesid_to_global_index(mesid);
  if (track_mask[global_index]) {
    tp_drop_channel(tracker, CH_DROP_REASON_MASKED);
    return;
  }

  /* Give newly-initialized channels a chance to converge. */
  if (!tracker_timer_expired(&tracker->init_settle_timer)) {
    return;
  }

  /* Do we not have nav bit sync yet? */
  if (0 == (flags & TRACKER_FLAG_BIT_SYNC)) {
    tp_drop_channel(tracker, CH_DROP_REASON_NO_BIT_SYNC);
    return;
  }

  /* PLL/FLL pessimistic lock detector "unlocked" for a while?
     We could get rid of this check altogether if not the
     observed cases, when tracker could not achieve the pessimistic
     lock state for a long time (minutes?) and yet managed to pass
     CN0 sanity checks.*/
  u64 unlocked_ms = tracker_timer_ms(&tracker->unlocked_timer);
  if (unlocked_ms > TRACK_DROP_UNLOCKED_MS) {
    tp_drop_channel(tracker, CH_DROP_REASON_NO_PLOCK);
    return;
  }

  /* CN0 below threshold for a while? */
  u64 cn0_drop_ms = tracker_timer_ms(&tracker->cn0_below_drop_thres_timer);
  if (cn0_drop_ms > TRACK_DROP_CN0_MS) {
    tp_drop_channel(tracker, CH_DROP_REASON_LOW_CN0);
    return;
  }
}

/**
 * Computes carrier phase offset.
 *
 * \param[in]  ref_tc Reference time
 * \param[in]  meas   Pre-populated channel measurement
 * \param[out] carrier_phase_offset Result
 *
 * \retval true Carrier phase offset is computed and \a carrier_phase_offset
 *              updated
 * \retval false Error in computation.
 */
static bool compute_cpo(u64 ref_tc,
                        const channel_measurement_t *meas,
                        s32 *carrier_phase_offset) {
  /* compute the pseudorange for this signal */
  double raw_pseudorange;
  if (!tracker_calc_pseudorange(ref_tc, meas, &raw_pseudorange)) {
    return false;
  }

  /* remove subsecond part of the clock error */
  double cpo_correction = sub_2ms_cpo_correction(ref_tc);
  double pseudorange_circ =
      sid_to_carr_freq(meas->sid) * (raw_pseudorange / GPS_C - cpo_correction);

  /* initialize the carrier phase offset with the pseudorange measurement */
  *carrier_phase_offset = round(pseudorange_circ - meas->carrier_phase);

  return true;
}

/**
 * Computes channel measurement flags from input.
 *
 * \param[in] flags Tracker manager flags
 * \param[in] mesid ME SID
 *
 * \return Channel measurement flags
 */
static chan_meas_flags_t compute_meas_flags(u32 flags,
                                            const me_gnss_signal_t mesid) {
  chan_meas_flags_t meas_flags = 0;

  if ((0 != (flags & TRACKER_FLAG_HAS_PLOCK)) &&
      (0 != (flags & TRACKER_FLAG_CARRIER_PHASE_OFFSET))) {
    meas_flags |= CHAN_MEAS_FLAG_PHASE_VALID;

    /* Make sense to set half cycle known flag when carrier phase is valid */
    if (0 != (flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) {
      /* Bit polarity is known */
      meas_flags |= CHAN_MEAS_FLAG_HALF_CYCLE_KNOWN;
    }
  }

  /* sanity check */
  if ((0 != (flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) &&
      (0 == (flags & TRACKER_FLAG_HAS_PLOCK))) {
    /* Somehow we managed to decode TOW when phase lock lost.
     * This should not happen, so print out warning. */
    log_warn_mesid(mesid, "Half cycle known, but no phase lock!");
  }

  if ((0 != (flags & TRACKER_FLAG_HAS_PLOCK)) ||
      (0 != (flags & TRACKER_FLAG_HAS_FLOCK))) {
    meas_flags |= CHAN_MEAS_FLAG_CODE_VALID;
    meas_flags |= CHAN_MEAS_FLAG_MEAS_DOPPLER_VALID;
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
  memset(meas, 0, sizeof(*meas));

  tracker_info_t info;
  tracker_freq_info_t freq_info;

  tracker_get_state(i, &info, &freq_info);
  u32 flags = info.flags;
  if (IS_GLO(info.mesid) && !glo_slot_id_is_valid(info.glo_orbit_slot)) {
    memset(meas, 0, sizeof(*meas));
    return flags | TRACKER_FLAG_MASKED;
  }

  if ((0 != (flags & TRACKER_FLAG_ACTIVE)) &&
      (0 != (flags & TRACKER_FLAG_CONFIRMED)) &&
      (0 == (flags & TRACKER_FLAG_DROP_CHANNEL)) &&
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

    tracker_measurement_get(ref_tc, &info, &freq_info, meas);

    /* Adjust for half phase ambiguity */
    if ((0 != (info.flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) &&
        (0 != (info.flags & TRACKER_FLAG_BIT_INVERTED))) {
      meas->carrier_phase += 0.5;
    }

    if (CODE_GAL_E1B == meas->sid.code) {
      meas->carrier_phase += 0.5;
    } else if (CODE_GAL_E7I == meas->sid.code) {
      meas->carrier_phase -= 0.25;
    }

    /* Adjust carrier phase initial integer offset to be approximately equal to
     * pseudorange.
     *
     * The initial integer offset shall be adjusted only when conditions that
     * have caused initial offset reset are not longer present. See callers of
     * tracker_ambiguity_unknown() for more details.
     */
    s32 carrier_phase_offset = freq_info.cpo.value;

    /* try to compute cpo if it is not computed yet but could be */
    if ((0 == carrier_phase_offset) &&
        (0 != (flags & TRACKER_FLAG_HAS_PLOCK)) &&
        (0 != (flags & TRACKER_FLAG_TOW_VALID)) &&
        (0 != (flags & TRACKER_FLAG_CN0_USABLE)) &&
        (TIME_FINE <= get_time_quality())) {
      if (compute_cpo(ref_tc, meas, &carrier_phase_offset)) {
        tracker_set_carrier_phase_offset(&info, carrier_phase_offset);
      }
    }
    /* apply the cpo if it is available */
    if (0 != carrier_phase_offset) {
      flags |= TRACKER_FLAG_CARRIER_PHASE_OFFSET;
      meas->carrier_phase += (double)carrier_phase_offset;
    }
    meas->flags = compute_meas_flags(flags, info.mesid);
  } else {
    memset(meas, 0, sizeof(*meas));
  }

  return flags;
}

/**
 * Compute extended tracking flags for GNSS signal.
 *
 * The method computes additional channel flags by using non-tracking data
 * sources. This is done to prevent potential dead-locking and reduce the
 * size of tracking lock congestion.
 *
 * \param[in]  sid  Signal identifier.
 * \param[in]  tow_ms ToW in milliseconds. Can be \a TOW_UNKNOWN
 * \param[in]  pephe  Pointer to ephemeris, or NULL if not available
 *
 * \return Flags, computed from ephemeris and other sources.
 */
static u32 get_tracking_channel_sid_flags(const gnss_signal_t sid,
                                          s32 tow_ms,
                                          const ephemeris_t *pephe) {
  u32 flags = 0;

  /* Satellite elevation is either unknown or above the solution mask. */
  double elevation;
  if (!track_sid_db_elevation_degrees_get(sid, &elevation) ||
      elevation >= solution_elevation_mask) {
    flags |= TRACKER_FLAG_ELEVATION;
  }

  gps_time_t t = {.wn = WN_UNKNOWN, .tow = 1e-3 * tow_ms};

  /* Ephemeris must be valid, not stale. Satellite must be healthy.
     This also acts as a sanity check on the channel TOW.*/
  if ((NULL != pephe) && (TOW_UNKNOWN != tow_ms) &&
      ephemeris_valid(pephe, &t)) {
    flags |= TRACKER_FLAG_HAS_EPHE;
  }

  if (shm_navigation_suitable(sid)) {
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

/**
 * Calculates how many SV of the given code are in track
 * \param code code type
 * \return the count of tracked SVs of the given code
 */
u8 code_track_count(code_t code) {
  u8 sv_tracked = 0;
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if ((acq_status[i].mesid.code == code) &&
        (ACQ_PRN_TRACKING == acq_status[i].state)) {
      sv_tracked++;
    }
  }
  return sv_tracked;
}

/**
 * The function calculates how many SV of defined GNSS are in track
 * \param[in] gnss GNSS constellation type
 */
u8 constellation_track_count(constellation_t gnss) {
  u8 sv_tracked = 0;
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if ((ACQ_PRN_TRACKING == acq_status[i].state) &&
        (mesid_to_constellation(acq_status[i].mesid) == gnss)) {
      sv_tracked++;
    }
  }
  return sv_tracked;
}

/** Read tracking startup requests from the FIFO and attempt to start
 * tracking and decoding.
 */
void manage_tracking_startup(void) {
  tracking_startup_params_t startup_params;
  while (tracking_startup_fifo_read(&tracking_startup_fifo, &startup_params)) {
    acq_status_t *acq =
        &acq_status[mesid_to_global_index(startup_params.mesid)];

    if (ACQ_PRN_TRACKING == acq->state || ACQ_PRN_UNHEALTHY == acq->state) {
      continue;
    }

    if (IS_SBAS(acq->mesid)) {
      if (constellation_track_count(CONSTELLATION_SBAS) >= SBAS_SV_NUM_LIMIT) {
        continue;
      }
    }

    /* Make sure a tracking channel and a decoder channel are available */
    u8 chan = manage_track_new_acq(startup_params.mesid);

    if (chan == MANAGE_NO_CHANNELS_FREE) {
      if (code_requires_direct_acq(acq->mesid.code)) {
        float doppler_min_hz = code_to_sv_doppler_min(acq->mesid.code) +
                               code_to_tcxo_doppler_min(acq->mesid.code);
        float doppler_max_hz = code_to_sv_doppler_max(acq->mesid.code) +
                               code_to_tcxo_doppler_max(acq->mesid.code);

        /* No channels are free to accept our new satellite :( */
        /* TODO: Perhaps we can try to warm start this one
         * later using another fine acq.
         */
        if (startup_params.cn0_init > ACQ_RETRY_THRESHOLD) {
          /* Check that reported Doppler frequency is within Doppler bounds */
          float doppler_hz = startup_params.doppler_hz;
          if (doppler_hz < doppler_min_hz) {
            doppler_hz = doppler_min_hz;
          } else if (doppler_hz > doppler_max_hz) {
            doppler_hz = doppler_max_hz;
          }
          acq->dopp_hint_low =
              MAX(doppler_hz - ACQ_FULL_CF_STEP, doppler_min_hz);
          acq->dopp_hint_high =
              MIN(doppler_hz + ACQ_FULL_CF_STEP, doppler_max_hz);
        }
      }
      log_info_mesid(startup_params.mesid,
                     "No free tracking channel available.");
      continue;
    }

    /* Start the decoder channel if needed.
     * Starting the decoder before the tracking is necessary so that
     * if tracking starts and fails the decoder isn't initialized,
     * leading to a stale unrecoverable state */
    if (code_requires_decoder(startup_params.mesid.code) &&
        !decoder_channel_init(chan, startup_params.mesid)) {
      log_error("decoder channel init failed");
      continue;
    }

    /* Change state to TRACKING.
     * This has to be done here as the acquisition thread has very low priority
     * so a tracker could start and fail and the acquisition mark the state to
     * tracking after it all happened, leading to a stale acquisition state */
    acq->state = ACQ_PRN_TRACKING;

    /* Start the tracking channel */
    if (!tracker_init(chan,
                      startup_params.mesid,
                      startup_params.glo_slot_id,
                      startup_params.sample_count,
                      startup_params.code_phase,
                      startup_params.doppler_hz,
                      startup_params.chips_to_correlate,
                      startup_params.cn0_init)) {
      log_error("tracker channel init failed");
      /* If starting of a channel fails, change state to ACQUIRING */
      acq->state = ACQ_PRN_ACQUIRING;
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
  return (ACQ_PRN_TRACKING == acq_status[global_index].state);
}

/** Checks if GLONASS enabled
 *
 * @return true if GLONASS enabled, otherwise false
 */
bool is_glo_enabled(void) { return cons_cfg[CONSTELLATION_GLO].enabled; }

/** Checks if SBAS enabled
 *
 * @return true if SBAS enabled, otherwise false
 */
bool is_sbas_enabled(void) { return cons_cfg[CONSTELLATION_SBAS].enabled; }

/** Checks if BDS2 enabled
 *
 * @return true if BDS2 enabled, otherwise false
 */
bool is_bds2_enabled(void) { return cons_cfg[CONSTELLATION_BDS].enabled; }

/** Checks if QZSS enabled
 *
 * @return true if QZSS enabled, otherwise false
 */
bool is_qzss_enabled(void) { return cons_cfg[CONSTELLATION_QZS].enabled; }

/** Checks if GAL enabled
 *
 * @return true if GAL enabled, otherwise false
 */
bool is_galileo_enabled(void) { return cons_cfg[CONSTELLATION_GAL].enabled; }

/**
 * The function retrieves the GLO orbit slot, if the mapping to a FCN exists
 * and the SV is visible.
 *
 * @param[in]  fcn  Frequency slot to be checked
 *
 * @return GLO orbit slot
 */
u16 get_orbit_slot(const u16 fcn) {
  u16 glo_orbit_slot = GLO_ORBIT_SLOT_UNKNOWN;
  u16 slot_id1, slot_id2;
  /* check if we have the fcn mapped already to some slot id */
  u8 num_si = glo_map_get_slot_id(fcn, &slot_id1, &slot_id2);
  switch (num_si) {
    case 1: {
      bool vis, kn = false;
      sm_get_glo_visibility_flags(slot_id1, &vis, &kn);
      /* the fcn mapped to one slot id only,
       * so use it as glo prn to be tracked if it's visible at the moment */
      if (vis & kn) {
        glo_orbit_slot = slot_id1;
      }
    } break;
    case 2: {
      bool vis, kn = false;
      /* we have 2 slot ids mapped to one fcn */
      /* check if SV with slot id 1 is visible */
      sm_get_glo_visibility_flags(slot_id1, &vis, &kn);
      if (vis && kn) {
        /* SV with the FIRST slot ID is visible, track it */
        glo_orbit_slot = slot_id1;
      } else {
        /* check the second slot id */
        sm_get_glo_visibility_flags(slot_id2, &vis, &kn);
        if (vis & kn) {
          /* SV with the SECOND slot ID is visible, track it */
          glo_orbit_slot = slot_id2;
        }
      }
    } break;
    default:
      glo_orbit_slot = GLO_ORBIT_SLOT_UNKNOWN;
      break;
  }
  return glo_orbit_slot;
}

/** \} */
