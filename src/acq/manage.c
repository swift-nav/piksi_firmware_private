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
#include <libswiftnav/almanac.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/glo_map.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/signal.h>

#include "board/nap/track_channel.h"
#include "calc/starling_threads.h"
#include "decode.h"
#include "dum/dum.h"
#include "ephemeris/ephemeris.h"
#include "gnss_capabilities/gnss_capabilities.h"
#include "main.h"
#include "manage.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "piksi_systime.h"
#include "position/position.h"
#include "reacq/reacq_api.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "soft_macq/soft_macq_main.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"
#include "track/track_api.h"
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

#define MANAGE_ACQ_THREAD_PRIORITY (LOWPRIO)
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
/** Flag if GLONASS enabled */
static bool glo_enabled = CODE_GLO_L1OF_SUPPORT || CODE_GLO_L2OF_SUPPORT;
/** Flag if SBAS enabled */
static bool sbas_enabled = CODE_SBAS_L1CA_SUPPORT;
/** Flag if BEIDOU2 enabled */
static bool bds2_enabled = CODE_BDS2_B1_SUPPORT || CODE_BDS2_B2_SUPPORT;
/** Flag if QZSS enabled */
static bool qzss_enabled = CODE_QZSS_L1CA_SUPPORT || CODE_QZSS_L2C_SUPPORT;
/** Flag if Galileo enabled */
static bool galileo_enabled = CODE_GAL_E1_SUPPORT || CODE_GAL_E7_SUPPORT;

typedef struct {
  piksi_systime_t tick; /**< Time when SV was detected as unhealthy */
  acq_status_t *status; /**< Pointer to acq status for the SV */
} acq_timer_t;

/* The array keeps time when GLO SV was detected as unhealthy */
static acq_timer_t glo_acq_timer[NUM_SATS_GLO] = {0};

/* The array keeps time when SBAS SV was detected as unhealthy. */
static acq_timer_t sbas_acq_timer[NUM_SATS_SBAS] = {0};

static u8 manage_track_new_acq(const me_gnss_signal_t mesid);
static void manage_acq(void);

static void manage_tracking_startup(void);
static void tracking_startup_fifo_init(tracking_startup_fifo_t *fifo);
static bool tracking_startup_fifo_write(
    tracking_startup_fifo_t *fifo, const tracking_startup_params_t *element);
static bool tracking_startup_fifo_read(tracking_startup_fifo_t *fifo,
                                       tracking_startup_params_t *element);

void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known);

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
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  log_debug("GLONASS status (1 - on, 0 - off): %u", glo_enabled);
  if (glo_enabled && !(CODE_GLO_L1OF_SUPPORT || CODE_GLO_L2OF_SUPPORT)) {
    /* user tries enable GLONASS on the platform that does not support it */
    log_error("The platform does not support GLONASS");
    glo_enabled = false;
    return false;
  }
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (IS_GLO(acq_status[i].mesid)) {
      acq_status[i].masked = !glo_enabled;
    }
  }
  return true;
}

/* The function masks/unmasks all SBAS satellites,
 * NOTE: this function does not check if SBAS SV is already masked or not */
static bool sbas_enable_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  log_debug("SBAS status (1 - on, 0 - off): %u", sbas_enabled);
  if (sbas_enabled && !(CODE_SBAS_L1CA_SUPPORT)) {
    /* user tries enable SBAS on the platform that does not support it */
    log_error("The platform does not support SBAS");
    sbas_enabled = false;
    return false;
  }
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (IS_SBAS(acq_status[i].mesid)) {
      acq_status[i].masked = !sbas_enabled || !sbas_active(acq_status[i].mesid);
    }
  }
  return true;
}

/* The function masks/unmasks all Beidou satellites,
 * NOTE: this function does not check if BDS2 SV is already masked or not */
static bool bds2_enable_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  log_debug("BEIDOU status (1 - on, 0 - off): %u", bds2_enabled);
  if (bds2_enabled && !(CODE_BDS2_B1_SUPPORT || CODE_BDS2_B2_SUPPORT)) {
    /* user tries enable Beidou2 on the platform that does not support it */
    log_error("The platform does not support BDS2");
    bds2_enabled = false;
    return false;
  }
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (IS_BDS2(acq_status[i].mesid)) {
      acq_status[i].masked = !bds2_enabled || !bds_active(acq_status[i].mesid);
    }
  }
  return true;
}

/* The function masks/unmasks all QZSS satellites,
 * NOTE: this function does not check if QZSS SV is already masked or not */
static bool qzss_enable_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  log_debug("QZSS status (1 - on, 0 - off): %u", qzss_enabled);
  if (qzss_enabled && !(CODE_QZSS_L1CA_SUPPORT || CODE_QZSS_L2C_SUPPORT)) {
    /* user tries enable QZSS on the platform that does not support it */
    log_error("The platform does not support QZSS");
    qzss_enabled = false;
    return false;
  }
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (IS_QZSS(acq_status[i].mesid)) {
      acq_status[i].masked = !qzss_enabled || !qzss_active(acq_status[i].mesid);
    }
  }
  return true;
}

/* The function masks/unmasks all Galileo satellites,
 * NOTE: this function does not check if Galileo SV is already masked or not */
static bool galileo_enable_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  log_debug("Galileo status (1 - on, 0 - off): %u", galileo_enabled);
  if (galileo_enabled && !(CODE_GAL_E1_SUPPORT || CODE_GAL_E7_SUPPORT)) {
    /* user tries enable Galileo on the platform that does not support it */
    log_error("The platform does not support Galileo");
    galileo_enabled = false;
    return false;
  }
  for (u16 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    if (IS_GAL(acq_status[i].mesid)) {
      acq_status[i].masked =
          !galileo_enabled || !gal_active(acq_status[i].mesid);
    }
  }
  return true;
}

/* Update the solution elevation mask used by the ME and by Starling. */
static bool solution_elevation_mask_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  log_debug("Solution elevation mask: %f", solution_elevation_mask);
  starling_set_elevation_mask(solution_elevation_mask);
  return true;
}

void manage_acq_setup() {
  SETTING("acquisition", "almanacs_enabled", almanacs_enabled, TYPE_BOOL);
  SETTING_NOTIFY("acquisition",
                 "glonass_acquisition_enabled",
                 glo_enabled,
                 TYPE_BOOL,
                 glo_enable_notify);
  SETTING_NOTIFY("acquisition",
                 "sbas_acquisition_enabled",
                 sbas_enabled,
                 TYPE_BOOL,
                 sbas_enable_notify);
  SETTING_NOTIFY("acquisition",
                 "bds2_acquisition_enabled",
                 bds2_enabled,
                 TYPE_BOOL,
                 bds2_enable_notify);
  SETTING_NOTIFY("acquisition",
                 "qzss_acquisition_enabled",
                 qzss_enabled,
                 TYPE_BOOL,
                 qzss_enable_notify);
  SETTING_NOTIFY("acquisition",
                 "galileo_acquisition_enabled",
                 galileo_enabled,
                 TYPE_BOOL,
                 galileo_enable_notify);

  tracking_startup_fifo_init(&tracking_startup_fifo);

  for (u32 i = 0; i < ARRAY_SIZE(acq_status); i++) {
    me_gnss_signal_t mesid = mesid_from_global_index(i);
    acq_status[i].state = ACQ_PRN_ACQUIRING;
    if (IS_GLO(mesid)) {
      acq_status[i].masked = !glo_enabled;
    }
    if (IS_SBAS(mesid)) {
      acq_status[i].masked = !sbas_enabled || !sbas_active(mesid);
    }
    if (IS_BDS2(mesid)) {
      acq_status[i].masked = !bds2_enabled || !bds_active(mesid);
    }
    if (IS_QZSS(mesid)) {
      acq_status[i].masked = !qzss_enabled || !qzss_active(mesid);
    }
    if (IS_GAL(mesid)) {
      acq_status[i].masked = !galileo_enabled || !gal_active(mesid);
    }

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
         (CODE_BDS2_B11 == acq->mesid.code) ||
         (CODE_QZS_L1CA == acq->mesid.code) ||
         (CODE_GAL_E1X == acq->mesid.code));

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
      return;
    }

    me_gnss_signal_t mesid_trk = acq->mesid;
    float cp = acq_result.cp;
    float cf = acq_result.cf;

    if (CODE_GAL_E1X == acq->mesid.code) {
      mesid_trk.code = CODE_GAL_E7X;
      cp = fmodf(cp * 10.0f, code_to_chip_count(CODE_GAL_E7X));
      cf = cf * GAL_E7_HZ / GAL_E1_HZ;
    }

    tracking_startup_params_t tracking_startup_params = {
        .mesid = mesid_trk,
        .glo_slot_id = GLO_ORBIT_SLOT_UNKNOWN,
        .sample_count = acq_result.sample_count,
        .carrier_freq = cf,
        .code_phase = cp,
        .chips_to_correlate = code_to_chip_count(mesid_trk.code),
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
  if (IS_GLO(mesid)) {
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
    if (code_requires_decoder(mesid.code) && tracker_available(i, mesid) &&
        decoder_channel_available(i, mesid)) {
      return i;
    } else if (!code_requires_decoder(mesid.code) &&
               tracker_available(i, mesid)) {
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
}

void me_settings_setup(void) {
  SETTING("track", "elevation_mask", tracking_elevation_mask, TYPE_FLOAT);
  SETTING_NOTIFY("solution",
                 "elevation_mask",
                 solution_elevation_mask,
                 TYPE_FLOAT,
                 solution_elevation_mask_notify);
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
    case CH_DROP_REASON_SV_UNHEALTHY:
      str = "SV is unhealthy, dropping";
      break;
    case CH_DROP_REASON_LEAP_SECOND:
      str = "Leap second event, dropping GLO signal";
      break;
    case CH_DROP_REASON_OUTLIER:
      str = "SV measurement outlier, dropping";
      break;
    case CH_DROP_REASON_SBAS_PROVIDER_CHANGE:
      str = "SBAS provider change, dropping";
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
 * \param[in,out] tracker Tracker channel data
 * \param[in] reason     Channel drop reason
 */
static void drop_channel(tracker_t *tracker, ch_drop_reason_t reason) {
  /* Read the required parameters from the tracking channel first to ensure
   * that the tracking channel is not restarted in the mean time.
   */
  const u32 flags = tracker->flags;
  me_gnss_signal_t mesid = tracker->mesid;
  u64 now_ms = timing_getms();
  u32 time_in_track_ms = (u32)(now_ms - tracker->init_timestamp_ms);

  /* Log message with appropriate priority. */
  if ((CH_DROP_REASON_ERROR == reason) ||
      (CH_DROP_REASON_NO_UPDATES == reason)) {
    log_error_mesid(mesid,
                    "[+%" PRIu32 "ms] nap_channel = %" PRIu8 " %s",
                    time_in_track_ms,
                    tracker->nap_channel,
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

  acq_status_t *acq = &acq_status[mesid_to_global_index(mesid)];

  if (code_requires_direct_acq(mesid.code)) {
    bool had_locks =
        (0 != (flags & (TRACKER_FLAG_HAD_PLOCK | TRACKER_FLAG_HAD_FLOCK)));
    bool long_in_track = time_in_track_ms > TRACK_REACQ_MS;
    u32 unlocked_time_ms =
        update_count_diff(tracker, &tracker->ld_pess_change_count);
    bool long_unlocked = unlocked_time_ms > TRACK_REACQ_MS;
    bool was_xcorr = (flags & TRACKER_FLAG_DROP_CHANNEL) &&
                     (CH_DROP_REASON_XCORR == tracker->ch_drop_reason);

    if (long_in_track && had_locks && !long_unlocked && !was_xcorr) {
      double carrier_freq = tracker->carrier_freq_at_lock;
      float doppler_min = code_to_sv_doppler_min(mesid.code) +
                          code_to_tcxo_doppler_min(mesid.code);
      float doppler_max = code_to_sv_doppler_max(mesid.code) +
                          code_to_tcxo_doppler_max(mesid.code);
      if ((carrier_freq < doppler_min) || (carrier_freq > doppler_max)) {
        log_error_mesid(
            mesid, "Acq: bogus carr freq: %lf. Rejected.", carrier_freq);
      } else {
        /* FIXME other constellations/bands */
        acq->dopp_hint_low = MAX(carrier_freq - ACQ_FULL_CF_STEP, doppler_min);
        acq->dopp_hint_high = MIN(carrier_freq + ACQ_FULL_CF_STEP, doppler_max);
      }
    }
  }

  /* Disable the decoder and tracking channels */
  decoder_channel_disable(tracker->nap_channel);
  tracker_disable(tracker->nap_channel);
}

/**
 * Restores acquisition for a satellite that has been disposed.
 *
 * \return
 */
void restore_acq(const tracker_t *tracker) {
  u32 flags = tracker->flags;
  me_gnss_signal_t mesid = tracker->mesid;
  acq_status_t *acq = &acq_status[mesid_to_global_index(mesid)];

  /* Now restore satellite acq */
  acq->state = ACQ_PRN_ACQUIRING;
  if (IS_GLO(mesid)) {
    bool glo_health_decoded = (0 != (flags & TRACKER_FLAG_GLO_HEALTH_DECODED));
    if (glo_health_decoded && (SV_UNHEALTHY == tracker->health) &&
        (tracker->glo_orbit_slot != GLO_ORBIT_SLOT_UNKNOWN)) {
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
    if (SV_UNHEALTHY == tracker->health) {
      acq->state = ACQ_PRN_UNHEALTHY;
      u16 index = tracker->mesid.sat - SBAS_FIRST_PRN;
      assert(index < ARRAY_SIZE(sbas_acq_timer));
      sbas_acq_timer[index].status = acq;
      piksi_systime_get(&sbas_acq_timer[index].tick); /* channel drop time */
    }
  }
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
void sanitize_tracker(tracker_t *tracker, u64 now_ms) {
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
    drop_channel(tracker, tracker->ch_drop_reason);
    return;
  }

  /* Is tracking masked? */
  u16 global_index = mesid_to_global_index(mesid);
  if (track_mask[global_index]) {
    drop_channel(tracker, CH_DROP_REASON_MASKED);
    return;
  }

  /* Give newly-initialized channels a chance to converge. */
  u32 age_ms = now_ms - tracker->init_timestamp_ms;
  if (age_ms < tracker->settle_time_ms) {
    return;
  }
  /*
    if (now_ms > tracker->update_timestamp_ms) {
      u32 update_delay_ms = now_ms - tracker->update_timestamp_ms;
      if (update_delay_ms > NAP_CORR_LENGTH_MAX_MS) {
        drop_channel(tracker, CH_DROP_REASON_NO_UPDATES);
        return;
      }
    }
  */
  /* Do we not have nav bit sync yet? */
  if (0 == (flags & TRACKER_FLAG_BIT_SYNC)) {
    drop_channel(tracker, CH_DROP_REASON_NO_BIT_SYNC);
    return;
  }

  /* PLL/FLL pessimistic lock detector "unlocked" for a while?
     We could get rid of this check altogether if not the
     observed cases, when tracker could not achieve the pessimistic
     lock state for a long time (minutes?) and yet managed to pass
     CN0 sanity checks.*/
  u32 unlocked_ms = 0;
  if ((0 == (flags & TRACKER_FLAG_HAS_PLOCK)) &&
      (0 == (flags & TRACKER_FLAG_HAS_FLOCK))) {
    unlocked_ms = update_count_diff(tracker, &tracker->ld_pess_change_count);
  }
  if (unlocked_ms > TRACK_DROP_UNLOCKED_MS) {
    drop_channel(tracker, CH_DROP_REASON_NO_PLOCK);
    return;
  }

  /* CN0 below threshold for a while? */
  u32 cn0_drop_ms =
      update_count_diff(tracker, &tracker->cn0_above_drop_thres_count);
  if (cn0_drop_ms > TRACK_DROP_CN0_MS) {
    drop_channel(tracker, CH_DROP_REASON_LOW_CN0);
    return;
  }

  /* Drop GLO if the SV is unhealthy */
  if (IS_GLO(mesid)) {
    bool glo_health_decoded = (0 != (flags & TRACKER_FLAG_GLO_HEALTH_DECODED));
    if (glo_health_decoded && (SV_UNHEALTHY == tracker->health)) {
      drop_channel(tracker, CH_DROP_REASON_SV_UNHEALTHY);
      return;
    }
  } else if (IS_SBAS(mesid)) {
    if (SV_UNHEALTHY == tracker->health) {
      drop_channel(tracker, CH_DROP_REASON_SV_UNHEALTHY);
      return;
    }
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
  double cpo_correction = subsecond_cpo_correction(ref_tc);
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
  tracker_time_info_t time_info;
  tracker_freq_info_t freq_info;
  tracker_misc_info_t misc_info;

  tracker_get_state(i, &info, &time_info, &freq_info, &misc_info);
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

    tracker_measurement_get(
        ref_tc, &info, &freq_info, &time_info, &misc_info, meas);

    /* Adjust for half phase ambiguity */
    if ((0 != (info.flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) &&
        (0 != (info.flags & TRACKER_FLAG_BIT_INVERTED))) {
      meas->carrier_phase += 0.5;
    }

    if (CODE_GAL_E1X == sid.code) {
      sid.code = CODE_GAL_E1B;
      meas->carrier_phase += 0.5;
    } else if (CODE_GAL_E7X == sid.code) {
      sid.code = CODE_GAL_E7I;
      meas->carrier_phase -= 0.25;
    }

    /* Adjust carrier phase initial integer offset to be approximately equal to
     * pseudorange.
     *
     * The initial integer offset shall be adjusted only when conditions that
     * have caused initial offset reset are not longer present. See callers of
     * tracker_ambiguity_unknown() for more details.
     */
    s32 carrier_phase_offset = misc_info.carrier_phase_offset.value;

    /* try to compute cpo if it is not computed yet but could be */
    if ((0 == carrier_phase_offset) &&
        (0 != (flags & TRACKER_FLAG_HAS_PLOCK)) &&
        (0 != (flags & TRACKER_FLAG_TOW_VALID)) &&
        (0 != (flags & TRACKER_FLAG_CN0_SHORT)) &&
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
    meas->elevation = (double)track_sid_db_elevation_degrees_get(meas->sid);
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
  if (track_sid_db_elevation_degrees_get(sid) >= solution_elevation_mask) {
    flags |= TRACKER_FLAG_ELEVATION;
  }

  gps_time_t t = {.wn = WN_UNKNOWN, .tow = 1e-3 * tow_ms};

  /* Ephemeris must be valid, not stale. Satellite must be healthy.
     This also acts as a sanity check on the channel TOW.*/
  if ((NULL != pephe) && (TOW_UNKNOWN != tow_ms) &&
      ephemeris_valid(pephe, &t)) {
    flags |= TRACKER_FLAG_HAS_EPHE;

    if (shm_ephe_healthy(pephe, sid.code)) {
      flags |= TRACKER_FLAG_HEALTHY;
    }
  }

  if ((IS_GPS(sid) || IS_BDS2(sid) || IS_GAL(sid)) &&
      shm_navigation_suitable(sid)) {
    flags |= TRACKER_FLAG_NAV_SUITABLE;
  } else if (IS_GLO(sid) && (flags & TRACKER_FLAG_HEALTHY)) {
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
static void manage_tracking_startup(void) {
  tracking_startup_params_t startup_params;
  while (tracking_startup_fifo_read(&tracking_startup_fifo, &startup_params)) {
    acq_status_t *acq =
        &acq_status[mesid_to_global_index(startup_params.mesid)];

    if (ACQ_PRN_TRACKING == acq->state) {
      continue;
    }

    if (IS_SBAS(acq->mesid)) {
      if (constellation_track_count(CONSTELLATION_SBAS) >= SBAS_SV_NUM_LIMIT) {
        continue;
      }
      if (ACQ_PRN_UNHEALTHY == acq->state) {
        continue;
      }
    } else if (IS_GLO(acq->mesid)) {
      if (ACQ_PRN_UNHEALTHY == acq->state) {
        continue;
      }
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
      log_info_mesid(startup_params.mesid,
                     "No free tracking channel available.");
      continue;
    }

    /* Change state to TRACKING */
    acq->state = ACQ_PRN_TRACKING;

    /* Start the tracking channel */
    if (!tracker_init(chan,
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

/** Checks if SBAS enabled
 *
 * @return true if SBAS enabled, otherwise false
 */
bool is_sbas_enabled(void) { return sbas_enabled; }

/** Checks if BDS2 enabled
 *
 * @return true if BDS2 enabled, otherwise false
 */
bool is_bds2_enabled(void) { return bds2_enabled; }

/** Checks if QZSS enabled
 *
 * @return true if QZSS enabled, otherwise false
 */
bool is_qzss_enabled(void) { return qzss_enabled; }

/** Checks if GAL enabled
 *
 * @return true if GAL enabled, otherwise false
 */
bool is_galileo_enabled(void) { return galileo_enabled; }

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

/** Return GLO fractional 2ms FCN residual for signal at given NAP count
 * \param sid gnss_signal_t to use
 * \param ref_tc NAP counter the measurements are referenced to
 * \return The residual in cycles
 */
double glo_2ms_fcn_residual(const gnss_signal_t sid, u64 ref_tc) {
  if (!IS_GLO(sid)) {
    return 0.0;
  }

  double carr_fcn_hz = 0;
  if (CODE_GLO_L1OF == sid.code) {
    carr_fcn_hz = (glo_map_get_fcn(sid) - GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ;
  } else if (CODE_GLO_L2OF == sid.code) {
    carr_fcn_hz = (glo_map_get_fcn(sid) - GLO_FCN_OFFSET) * GLO_L2_DELTA_HZ;
  }

  u64 tc_2ms_boundary = FCN_NCO_RESET_COUNT * (ref_tc / FCN_NCO_RESET_COUNT);
  return -carr_fcn_hz * (ref_tc - tc_2ms_boundary) /
         NAP_FRONTEND_SAMPLE_RATE_Hz;
}

/** \} */
