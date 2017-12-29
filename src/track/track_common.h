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

#include "track/tracker.h"

/**
 * Common tracker configuration container.
 */
typedef struct {
  bool show_unconfirmed_trackers; /**< Flag to control reporting of unconfirmed
                                   *   tracker channels */
  float xcorr_delta; /**< Frequency delta error for cross-correlation [hz] */
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
  double code_phase_rate; /**< Code frequency in Hz */
  double carr_freq;       /**< Carrier frequency in Hz */
  float acceleration;     /**< Acceleration in Hz/s */
  float cn0;              /**< Computed C/N0 (filtered) in dB/Hz */
  float cn0_raw;          /**< Computed C/N0 (raw) in dB/Hz */
  u32 plock : 1;          /**< Pessimistic phase lock flag */
  u32 flock : 1;          /**< Pessimistic frequency lock flag */
  u32 bsync : 1;          /**< Bit sync flag */
  u32 time_ms : 8;        /**< Time in milliseconds */
  u32 sample_count;       /**< Channel sample count */
} tp_report_t;

/**
 * Tracking profile result codes.
 */
typedef enum {
  TP_RESULT_SUCCESS = 0, /**< Successful operation. */
  TP_RESULT_ERROR = -1,  /**< Error during operation */
  TP_RESULT_NO_DATA = 1, /**< Profile has changed */
} tp_result_e;

tp_result_e tp_init(void);
void tp_profile_init(tracker_t *tracker_channel, const tp_report_t *data);

void tp_profile_update_config(tracker_t *tracker_channel);
void tp_profile_apply_config(tracker_t *tracker_channel, bool init);
void tp_profile_switch(tracker_t *tracker_channel);

tp_result_e tp_profile_get_cn0_params(const tp_profile_t *profile,
                                      tp_cn0_params_t *cn0_params);
bool tp_profile_has_new_profile(tracker_t *tracker_channel);
u8 tp_profile_get_next_loop_params_ms(const me_gnss_signal_t mesid,
                                      const tp_profile_t *profile);
void tp_profile_report_data(tracker_t *tracker_channel,
                            tp_profile_t *profile,
                            const tp_report_t *data);

u8 tp_next_cycle_counter(tp_tm_e tracking_mode, u8 cycle_no);
u32 tp_get_cycle_flags(tracker_t *tracker_channel, u8 cycle_no);
u32 tp_compute_cycle_parameters(tp_tm_e tracking_mode, u8 cycle_no);

u8 tp_get_cycle_count(tp_tm_e tracking_mode);
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no);
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no);
u8 tp_get_cn0_ms(tp_tm_e tracking_mode);
u8 tp_get_ld_ms(tp_tm_e tracking_mode);
float tp_get_alias_ms(tp_tm_e tracking_mode);
float tp_get_flld_ms(tp_tm_e tracking_mode);
u8 tp_get_flll_ms(tp_tm_e tracking_mode);
u8 tp_get_bit_ms(tp_tm_e tracking_mode);
u8 tp_get_pll_ms(tp_tm_e tracking_mode);
u8 tp_get_dll_ms(tp_tm_e tracking_mode);
const char *tp_get_mode_str(tp_tm_e v);
bool tp_is_pll_ctrl(tp_ctrl_e ctrl);
bool tp_is_fll_ctrl(tp_ctrl_e ctrl);

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
void tp_tl_update(tp_tl_state_t *s, const tp_epl_corr_t *cs, bool costas);
float tp_tl_get_fll_error(const tp_tl_state_t *s);
float tp_tl_get_dll_error(const tp_tl_state_t *s);
bool tp_tl_is_pll(const tp_tl_state_t *s);
bool tp_tl_is_fll(const tp_tl_state_t *s);
void tp_tl_fll_update_first(tp_tl_state_t *s, corr_t cs, bool halfq);
void tp_tl_fll_update_second(tp_tl_state_t *s, corr_t cs, bool halfq);
void tp_tl_fll_update(tp_tl_state_t *s);

/* Generic tracker functions */
void tp_tracker_init(tracker_t *tracker_channel,
                     const tp_tracker_config_t *config);
void tp_tracker_disable(tracker_t *tracker_channel);
u32 tp_tracker_update(tracker_t *tracker_channel,
                      const tp_tracker_config_t *config);
void tp_tracker_update_correlators(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_bsync(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_cn0(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_locks(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_fll(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_pll_dll(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_alias(tracker_t *tracker_channel, u32 cycle_flags);
void tp_tracker_filter_doppler(tracker_t *tracker_channel,
                               u32 cycle_flags,
                               const tp_tracker_config_t *config);
void tp_tracker_update_mode(tracker_t *tracker_channel);
u32 tp_tracker_compute_rollover_count(tracker_t *tracker_channel);
void tp_tracker_update_cycle_counter(tracker_t *tracker_channel);

#endif
