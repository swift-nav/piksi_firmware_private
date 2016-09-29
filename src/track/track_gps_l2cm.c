/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* skip weak attributes for L2C API implementation */
#define TRACK_GPS_L2CM_INTERNAL
#include "track_gps_l2cm.h"
#include "track_api.h"
#include "track_cn0.h"
#include "manage.h"
#include "track.h"
#include "ndb.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>
#include "math.h"

#include "settings.h"
#include "signal.h"
#include "track_profile_utils.h"

/** L2C coherent integration time [ms] */
#define L2C_COHERENT_INTEGRATION_TIME_MS 20

/** C/N0 estimator to use */
#define L2C_CN0_ESTIMATOR TRACK_CN0_EST_BL
#define L2C_CN0_INIT_ESTIMATOR TRACK_CN0_EST_BL

/** Finite state machine data structure */
struct fsm_states {
  float t_diff_s; /**< the time difference used by the alias lock detector [s]*/
  u32 startup_int_time; /**< start-up integration time [chips] */
  u32 alias_acc_length; /**< alias detector accumulation rounds number */
  struct {
    u32 int_time;      /**< integration time [chips] */
    u8 next_fsm_state; /**< next index in fsm_states::states array */
    u32 flags;         /**< bit-field of TP_CFLAG_... flags */
  } states[];
};

static const struct fsm_states fsm_states = {
  (5. * 1023) / GPS_CA_CHIPPING_RATE, /* alias time delta  */
  1023,  /* first integration time [chips] */
  200,  /* alias accumulation rounds. Needs to fit into u8. */
  {
    /* start-up case */
    {3 * 1023, 1, 0},                                           /* 0, 1023 */
    {5 * 1023, 3, 0},                                           /* 1, 2*1023 */

    /* normal case */
    {5 * 1023, 3, 0},                                           /* 2, 1023 */
    {5 * 1023, 4, TP_CFLAG_ALIAS_FIRST},                        /* 3, 5*1023 */
    {5 * 1023, 5, TP_CFLAG_ALIAS_FIRST | TP_CFLAG_ALIAS_SECOND},/* 4, 10*1023 */
    {    1023, 6, TP_CFLAG_ALIAS_FIRST | TP_CFLAG_ALIAS_SECOND},/* 5, 15*1023 */
    {4 * 1023, 2, TP_CFLAG_ALIAS_SECOND | TP_CFLAG_EPL_USE},    /* 6, 20*1023 */
  }
};

#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/*                        code: nbw zeta  k  carr_to_code
                       carrier:                     nbw zeta  k  fll_aid */
#define LOOP_PARAMS_MED "(20 ms, (.5, 0.7, 1, 1200), (13, 0.7, 1, 5))"

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS          "0.0247, 1.5, 50, 240"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, fll_bw;
  u8 coherent_ms;
} loop_params_stage;

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 37.0; /* dBHz */
static float track_cn0_drop_thres = 31.0; /* dBHz */

static char loop_params_string[120] = LOOP_PARAMS_MED;
static char lock_detect_params_string[24] = LD_PARAMS;
static bool use_alias_detection = true;
static bool show_unconfirmed_trackers = false;

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  track_cn0_state_t cn0_est;   /**< C/N0 Estimator. */
  u8 int_ms;                   /**< Integration length. */
  u8 confirmed;                /**< Confirmed tracking flag */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
  u8 fsm_state;                /**< The index of fsm_states::states array. */
} gps_l2cm_tracker_data_t;

static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params(struct setting *s, const char *val);
static bool parse_lock_detect_params(struct setting *s, const char *val);

static const tracker_interface_t tracker_interface_gps_l2cm = {
  .code =         CODE_GPS_L2CM,
  .init =         tracker_gps_l2cm_init,
  .disable =      tracker_gps_l2cm_disable,
  .update =       tracker_gps_l2cm_update,
  .trackers =     gps_l2cm_trackers,
  .num_trackers = NUM_GPS_L2CM_TRACKERS
};

static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cm = {
    .interface = &tracker_interface_gps_l2cm,
    .next = 0
  };

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "loop_params",
                 loop_params_string,
                 TYPE_STRING, parse_loop_params);

  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "lock_detect_params",
                 lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_use",
          track_cn0_use_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_drop",
          track_cn0_drop_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "alias_detect",
          use_alias_detection, TYPE_BOOL);

  SETTING(L2CM_TRACK_SETTING_SECTION, "show_unconfirmed",
          show_unconfirmed_trackers, TYPE_BOOL);

  for (u32 i = 0; i < NUM_GPS_L2CM_TRACKERS; i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sample_count NAP sample count
 * \param sat L1C/A Satellite ID
 * \param code_phase L1CA code phase [chips]
 * \param carrier_freq The current Doppler frequency for the L1 C/A channel
 * \param cn0 CN0 estimate for the L1 C/A channel
 */
void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              float code_phase,
                              double carrier_freq,
                              float cn0_init)
{
  /* compose SID: same SV, but code is L2 CM */
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(sid)) {
    return; /* L2C signal from the SV is already in track */
  }

  u32 capb;
  ndb_gps_l2cm_l2c_cap_read(&capb);
  if (0 == (capb & ((u32)1 << (sat - 1)))) {
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn_sid(sid, "Unexpected L1C/A to L2C handover code phase: %f",
                 code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = GPS_L2C_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .sid                = sid,
    .sample_count       = sample_count,
    /* recalculate doppler freq for L2 from L1*/
    .carrier_freq       = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
    .code_phase         = code_phase,
    .chips_to_correlate = fsm_states.startup_int_time,
    /* get initial cn0 from parent L1 channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_sid(sid, "L2 CM handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_sid(sid, "Failed to start L2C tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage;

  assert(L2C_COHERENT_INTEGRATION_TIME_MS == l->coherent_ms);
  data->int_ms = l->coherent_ms;

  aided_tl_init(&(data->tl_state), 1e3 / data->int_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->fll_bw);

  float cn0 = 0;
  if (show_unconfirmed_trackers) {
    cn0 = common_data->cn0;
    data->confirmed = 1;
  } else {
    cn0 = track_cn0_drop_thres - 2;
    data->confirmed = 0;
  }

  /* Initialize C/N0 estimator and filter */
  track_cn0_init(channel_info->sid,
                 data->int_ms,              /* C/N0 period in ms */
                 &data->cn0_est,            /* C/N0 estimator state */
                 cn0,                       /* Initial C/N0 value */
                 TRACK_CN0_FLAG_FAST_TYPE); /* Fast type */

  if (!show_unconfirmed_trackers) {
    /* C/N0 is not reported until estimator shows value above drop threshold */
    data->cn0_est.cn0_0 = common_data->cn0;
    common_data->cn0 = -1;
  }

  /* Initialize lock detector */
  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  alias_detect_init(&data->alias_detect,
                    fsm_states.alias_acc_length, fsm_states.t_diff_s);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(channel_info->context, 0);

  data->fsm_state = 0;

  for (int i = 0; i < 3; i++) {
    data->cs[i].I = 0;
    data->cs[i].Q = 0;
  }
}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;
  u32 flags;
  corr_t cs[3];
  u32 sample_count;        /* Total num samples channel has tracked for. */
  double code_phase_early; /* Early code phase. */
  double carrier_phase;    /* Carrier phase in NAP register units. */
  u8 fsm_state = data->fsm_state;

  tracker_correlations_read(channel_info->context, cs,
                            &sample_count,
                            &code_phase_early,
                            &carrier_phase);

  for (int i = 0; i < 3; i++) {
    data->cs[i].I += cs[i].I;
    data->cs[i].Q += cs[i].Q;
  }

  flags = fsm_states.states[fsm_state].flags;

  data->fsm_state = fsm_states.states[fsm_state].next_fsm_state;

  if (0 != (flags & TP_CFLAG_ALIAS_SECOND)) {
    s16 err = 0;

    if (use_alias_detection &&
        data->lock_detect.outp && data->lock_detect.outo)
      err = tp_tl_detect_alias(&data->alias_detect, cs[1].I, cs[1].Q);

    if (err) {
      tracker_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      common_data->mode_change_count = common_data->update_count;

      aided_tl_adjust(&data->tl_state, err);

      log_warn_sid(channel_info->sid,
                 "False phase lock detected. Err: %" PRId16 "Hz. Corrected", err);
    }
  }

  if (0 != (flags & TP_CFLAG_ALIAS_FIRST))
    alias_detect_first(&data->alias_detect, cs[1].I, cs[1].Q);

  if (0 == (flags & TP_CFLAG_EPL_USE)) {
    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate,
                   fsm_states.states[fsm_state].int_time);
    return;
  }

  common_data->sample_count = sample_count;
  common_data->code_phase_early = code_phase_early;
  common_data->carrier_phase = carrier_phase;

  common_data->update_count += data->int_ms;

  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           data->int_ms);

  /* Call the bit sync update API to do data decoding */
  tracker_bit_sync_update(channel_info->context, data->int_ms, data->cs[1].I);

  /* Update C/N0 estimate */
  track_cn0_est_e estimator = 0 == data->confirmed ? L2C_CN0_INIT_ESTIMATOR :
                                                 L2C_CN0_ESTIMATOR;
  float cn0 = track_cn0_update(channel_info->sid,
                               estimator,
                               &data->cn0_est,
                               data->cs[1].I, data->cs[1].Q);

  if (cn0 > track_cn0_drop_thres) {
    common_data->cn0_above_drop_thres_count = common_data->update_count;
    if (0 == data->confirmed && data->lock_detect.outo) {
      data->confirmed = 1; /* Enabled C/N0 reporting if not enabled before */
      cn0 = data->cn0_est.cn0_0;
      /* Reinitialize C/N0 estimator and filter */
      track_cn0_init(channel_info->sid,         /* SV signal */
                     data->cn0_est.cn0_ms,      /* C/N0 period in ms */
                     &data->cn0_est,            /* C/N0 estimator state */
                     cn0,                       /* Initial C/N0 value */
                     TRACK_CN0_FLAG_FAST_TYPE); /* Fast type */
    }
  }

  /* Report C/N0 when stage is not initial */
  if (data->confirmed)
    common_data->cn0 = cn0;

  if (cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, data->cs[1].I, data->cs[1].Q,
                     data->int_ms);
  if (data->lock_detect.outo) {
    common_data->ld_opti_locked_count = common_data->update_count;
  }
  if (!data->lock_detect.outp) {
    common_data->ld_pess_unlocked_count = common_data->update_count;
  }

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    log_info_sid(channel_info->sid, "PLL stress");
    tracker_ambiguity_unknown(channel_info->context);
  }

  /* Run the loop filters. */

  /* Output I/Q correlations using SBP if enabled for this channel */
  tracker_correlations_send(channel_info->context, data->cs);

  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = data->cs[2-i].I;
    cs2[i].Q = data->cs[2-i].Q;
  }

  aided_tl_update(&data->tl_state, cs2);
  common_data->carrier_freq = data->tl_state.carr_freq;
  common_data->code_phase_rate = data->tl_state.code_freq +
                                 GPS_CA_CHIPPING_RATE;

  for(int i = 0; i < 3; i++) {
    data->cs[i].I = 0;
    data->cs[i].Q = 0;
  }

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 fsm_states.states[fsm_state].int_time);
}

/** Parse a string describing the tracking loop filter parameters into
 *  the loop_params_stage struct.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for one stage */

  struct loop_params loop_params_parse;

  const char *str = val;
  struct loop_params *l = &loop_params_parse;

  unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

  if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) ",
             &tmp,
             &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
             &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->fll_bw
             ) < 9) {
    log_error("Ill-formatted tracking loop param string: %20s", str);
    return false;
  }
  l->coherent_ms = tmp;

  if (l->coherent_ms != L2C_COHERENT_INTEGRATION_TIME_MS) {
    log_error("Invalid coherent integration length for L2CM: %" PRIu8,
              l->coherent_ms);
    return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&loop_params_stage, &loop_params_parse, sizeof(loop_params_stage));

  return true;
}

/** Parse a string describing the tracking loop phase lock detector
 *  parameters into the lock_detect_params structs.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string: %20s", val);
      return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));

  return true;
}
