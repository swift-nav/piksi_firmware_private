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

#ifndef SWIFTNAV_TRACK_H
#define SWIFTNAV_TRACK_H

#include <libsbp/tracking.h>

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "me_constants.h"
#include "track/tracker.h"

/** Tracker modes supported by the measuremente engine */
enum tracker_mode {
  TRACKER_MODE_ROVER, /* rover tracker mode */
  TRACKER_MODE_BASE   /* base station tracker mode */
};

/**
 * Common tracker configuration container.
 */
typedef struct {
  bool confirm_early; /**< Flag to control reporting of unconfirmed
                                   *   tracker channels */
  float xcorr_delta;  /**< Frequency delta error for cross-correlation [hz] */
  float
      xcorr_cof; /**< LPF cut-off frequency for cross-correlation filter [hz] */
  float xcorr_time; /**< Cross-correlation time threshold [s] */
  lp1_filter_params_t
      xcorr_f_params; /**< Cross-correlation filter parameters */
} tp_tracker_config_t;

/**
 * Macro for default tracker parameters initialization
 */
#define TP_TRACKER_DEFAULT_CONFIG    \
  {                                  \
    false, 10.f, 0.1f, 1.f, { 0, 0 } \
  }

/**
 * Tracking loop data.
 *
 * This structure contains tracking parameters required for profile changing
 * decision making.
 */
typedef struct {
  float cn0;   /**< Computed C/N0 (filtered) in dB/Hz */
  u32 time_ms; /**< Time elapsed since last report [ms] */
} tp_report_t;

void tp_profile_init(tracker_t *tracker, const tp_report_t *data);

void tp_profile_update_config(tracker_t *tracker);
void tp_profile_apply_config(tracker_t *tracker, bool init);
void tp_profile_switch(tracker_t *tracker);

void tp_profile_get_cn0_thres(const tp_profile_t *profile,
                              tp_cn0_thres_t *cn0_thres);
bool tp_profile_has_new_profile(tracker_t *tracker);
void tp_profile_report_data(tp_profile_t *profile, const tp_report_t *data);

u8 tp_next_cycle_counter(tp_tm_e tracking_mode, u8 cycle_no);
u32 tp_get_cycle_flags(tracker_t *tracker, u8 cycle_no);
u32 tp_compute_cycle_parameters(tp_tm_e tracking_mode, u8 cycle_no);

u8 tp_get_cycle_count(tp_tm_e tracking_mode);
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no);
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no);
u8 tp_get_cn0_ms(tp_tm_e tracking_mode);
u8 tp_get_ld_ms(tp_tm_e tracking_mode);
float tp_get_alias_ms(tp_tm_e tracking_mode);
float tp_get_flld_ms(tp_tm_e tracking_mode);
u8 tp_get_fpll_ms(tp_tm_e tracking_mode);
u8 tp_get_bit_ms(tp_tm_e tracking_mode);
u8 tp_get_dll_ms(tp_tm_e tracking_mode);
u8 tp_get_fpll_decim(tp_tm_e tracking_mode);
const char *tp_get_mode_str(tp_tm_e v);

void tp_update_correlators(u32 cycle_flags,
                           const tp_epl_corr_t *restrict cs_now,
                           tp_corr_state_t *restrict corr_state);

void tp_tl_init(tp_tl_state_t *s,
                tp_ctrl_e ctrl,
                const tl_rates_t *rates,
                const tl_config_t *config);

void tp_tl_retune(tp_tl_state_t *s, tp_ctrl_e ctrl, const tl_config_t *config);

void tp_tl_adjust(tp_tl_state_t *s, float err);
void tp_tl_get_rates(const tp_tl_state_t *s, tl_rates_t *rates);
void tp_tl_get_config(const tp_loop_params_t *l, tl_config_t *config);

void tp_tl_update_dll_discr(tp_tl_state_t *s, const tp_epl_corr_t *cs);
void tp_tl_update_dll(tp_tl_state_t *s);

void tp_tl_update_fll_discr(tp_tl_state_t *s, corr_t cs, bool halfq);
void tp_tl_update_fpll(tp_tl_state_t *s, const tp_epl_corr_t *cs, bool costas);

float tp_tl_get_fll_error(const tp_tl_state_t *s);

/* Generic tracker functions */
void tp_tracker_init(tracker_t *tracker, const tp_tracker_config_t *config);
void tp_tracker_disable(tracker_t *tracker);
u32 tp_tracker_update(tracker_t *tracker, const tp_tracker_config_t *config);

tp_tm_e tp_profile_get_next_track_mode(const tp_profile_t *profile,
                                       me_gnss_signal_t mesid);

void tp_set_base_station_mode(void);
void tp_set_rover_mode(void);
bool tp_tracker_has_new_mode(void);
void tp_tracker_apply_new_mode(void);
bool tp_is_rover_mode(void);
bool tp_is_base_station_mode(void);

#endif
