/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_PROFILE_UTILS_H_
#define SWIFTNAV_TRACK_PROFILE_UTILS_H_

#include "track_profiles.h"

#include <nap/nap_common.h>
#include <libswiftnav/track.h>
#include <track_api.h>
#include <settings.h>

/* Integration tweaks: short / long interval markings */
#define TP_CFLAG_SHORT_CYCLE     ((u32)1 << 0)
#define TP_CFLAG_LONG_CYCLE      ((u32)1 << 1)

/* Bit synchronization and data decoding */
#define TP_CFLAG_BSYNC_SET       ((u32)1 << 2)
#define TP_CFLAG_BSYNC_ADD       ((u32)1 << 3)
#define TP_CFLAG_BSYNC_UPDATE    ((u32)1 << 4)

/* C/N0 estimator control */
#define TP_CFLAG_CN0_SET         ((u32)1 << 5)
#define TP_CFLAG_CN0_ADD         ((u32)1 << 6)
#define TP_CFLAG_CN0_USE         ((u32)1 << 7)

/* FLL control */
#define TP_CFLAG_FLL_SET         ((u32)1 << 8)
#define TP_CFLAG_FLL_ADD         ((u32)1 << 9)
#define TP_CFLAG_FLL_USE         ((u32)1 << 10)
#define TP_CFLAG_FLL_FIRST       ((u32)1 << 11)
#define TP_CFLAG_FLL_SECOND      ((u32)1 << 12)

/* DLL/PLL control */
#define TP_CFLAG_EPL_SET         ((u32)1 << 13)
#define TP_CFLAG_EPL_ADD         ((u32)1 << 14)
#define TP_CFLAG_EPL_ADD_INV     ((u32)1 << 15)
#define TP_CFLAG_EPL_INV_ADD     ((u32)1 << 16)
#define TP_CFLAG_EPL_USE         ((u32)1 << 17)

/* False lock detector control */
#define TP_CFLAG_ALIAS_SET       ((u32)1 << 18)
#define TP_CFLAG_ALIAS_ADD       ((u32)1 << 19)
#define TP_CFLAG_ALIAS_FIRST     ((u32)1 << 20)
#define TP_CFLAG_ALIAS_SECOND    ((u32)1 << 21)

/* Lock detector control */
#define TP_CFLAG_LD_SET          ((u32)1 << 22)
#define TP_CFLAG_LD_ADD          ((u32)1 << 23)
#define TP_CFLAG_LD_USE          ((u32)1 << 24)

#define TP_DLL_PLL_MEAS_DIM 3

/*
 * Main tracking: PLL loop selection
 */

/* FLL-assisted PLL. FLL is first order and PLL is second order */
#define tl_pll2_state_t        aided_tl_state_fll1_pll2_t
#define tl_pll2_init           aided_tl_fll1_pll2_init
#define tl_pll2_retune         aided_tl_fll1_pll2_retune
#define tl_pll2_update_fll     aided_tl_fll1_pll2_update_fll
#define tl_pll2_update_dll     aided_tl_fll1_pll2_update_dll
#define tl_pll2_adjust         aided_tl_fll1_pll2_adjust
#define tl_pll2_get_dll_error  aided_tl_fll1_pll2_get_dll_error
#define tl_pll2_discr_update   aided_tl_fll1_pll2_discr_update
#define tl_pll2_get_rates      aided_tl_fll1_pll2_get_rates

/*
 * 3rd order PLL loop selection.
 * The third order PLL loop selection is mutually exclusive from the three
 * available implementations:
 *
 * TRACK_PLL_MODE3_BL
 * PLL-assisted DLL. FLL and DLL are second order, PLL is third order
 * Note: Bilinear transform integrator implementation
 *
 * TRACK_PLL_MODE3_BC
 * PLL-assisted DLL. FLL and DLL are second order, PLL is third order
 * Note: Boxcar integrator implementation
 *
 * TRACK_PLL_MODE3_FLL
 * FLL-assisted PLL. FLL is second order and PLL is third order
 * Note: Bilinear transform integrator implementation
 *
 */
#define TRACK_PLL_MODE3_BL  1
#define TRACK_PLL_MODE3_BC  2
#define TRACK_PLL_MODE3_FLL 3

#ifndef TRACK_PLL_MODE3
#define TRACK_PLL_MODE3 TRACK_PLL_MODE3_BL
#endif

#if TRACK_PLL_MODE3 == TRACK_PLL_MODE3_BL
#define tl_pll3_state_t        aided_tl_state3_t
#define tl_pll3_init           aided_tl_init3
#define tl_pll3_retune         aided_tl_retune3
#define tl_pll3_update_fll     aided_tl_update_fll3
#define tl_pll3_update_dll     aided_tl_update_dll3
#define tl_pll3_adjust         aided_tl_adjust3
#define tl_pll3_get_dll_error  aided_tl_get_dll_error3
#define tl_pll3_discr_update   aided_tl_discr_update3
#define tl_pll3_get_rates      aided_tl_get_rates3
#elif TRACK_PLL_MODE3 == TRACK_PLL_MODE3_BC
#define tl_pll3_state_t        aided_tl_state3b_t
#define tl_pll3_init           aided_tl_init3b
#define tl_pll3_retune         aided_tl_retune3b
#define tl_pll3_update_fll     aided_tl_update_fll3b
#define tl_pll3_update_dll     aided_tl_update_dll3b
#define tl_pll3_adjust         aided_tl_adjust3b
#define tl_pll3_get_dll_error  aided_tl_get_dll_error3b
#define tl_pll3_discr_update   aided_tl_discr_update3b
#define tl_pll3_get_rates      aided_tl_get_rates3b
#elif TRACK_PLL_MODE3 == TRACK_PLL_MODE3_FLL
#define tl_pll3_state_t        aided_tl_state_fll2_pll3_t
#define tl_pll3_init           aided_tl_fll2_pll3_init
#define tl_pll3_retune         aided_tl_fll2_pll3_retune
#define tl_pll3_update_fll     aided_tl_fll2_pll3_update_fll
#define tl_pll3_update_dll     aided_tl_fll2_pll3_update_dll
#define tl_pll3_adjust         aided_tl_fll2_pll3_adjust
#define tl_pll3_get_dll_error  aided_tl_fll2_pll3_get_dll_error
#define tl_pll3_discr_update   aided_tl_fll2_pll3_discr_update
#define tl_pll3_get_rates      aided_tl_fll2_pll3_get_rates
#else
#error Unsupported 3rd order PLL Mode
#endif

/*
 * Main tracking: FLL loop selection
 */

/* FLL-assisted DLL. FLL is first order and DLL is second order */
#define tl_fll1_state_t        aided_tl_state_fll1_t
#define tl_fll1_init           aided_tl_fll1_init
#define tl_fll1_retune         aided_tl_fll1_retune
#define tl_fll1_update_fll     aided_tl_fll1_update_fll
#define tl_fll1_update_dll     aided_tl_fll1_update_dll
#define tl_fll1_adjust         aided_tl_fll1_adjust
#define tl_fll1_get_dll_error  aided_tl_fll1_get_dll_error
#define tl_fll1_discr_update   aided_tl_fll1_discr_update
#define tl_fll1_get_rates      aided_tl_fll1_get_rates

/* FLL-assisted DLL. FLL and DLL are both second order */
#define tl_fll2_state_t        aided_tl_state_fll2_t
#define tl_fll2_init           aided_tl_fll2_init
#define tl_fll2_retune         aided_tl_fll2_retune
#define tl_fll2_update_fll     aided_tl_fll2_update_fll
#define tl_fll2_update_dll     aided_tl_fll2_update_dll
#define tl_fll2_adjust         aided_tl_fll2_adjust
#define tl_fll2_get_dll_error  aided_tl_fll2_get_dll_error
#define tl_fll2_discr_update   aided_tl_fll2_discr_update
#define tl_fll2_get_rates      aided_tl_fll2_get_rates

/**
 * EPL accumulator structure.
 */
typedef struct
{
  union {
    corr_t epl[TP_DLL_PLL_MEAS_DIM]; /**< EPL accumulators as a vector */
    struct {
      corr_t early;                  /**< Early accumulator */
      corr_t prompt;                 /**< Prompt accumulator */
      corr_t late;                   /**< Late accumulator */
    };
  };
} tp_epl_corr_t;

/**
 * Tracker accumulators.
 *
 * Tracker uses different frequencies for the following algorithms:
 * - DLL and PLL
 * - C/N0 estimator
 * - FLL tracker
 * - Alias (false lock) detector
 * - Lock detector
 * - Bit synchronization and message decoding
 */
typedef struct
{
  tp_epl_corr_t    corr_epl; /**< EPL correlation results for DLL and PLL
                              *   in correlation period. */
  corr_t           corr_cn0; /**< C/N0 accumulator */
  corr_t           corr_fll; /**< FLL accumulator */
  corr_t           corr_ad;  /**< False lock (alias) detector accumulator */
  corr_t           corr_ld;  /**< Lock detector accumulator */
  s32              corr_bit; /**< Bit sync accumulator */
} tp_corr_state_t;

/**
 * Tracker loop state.
 */
typedef struct
{
  union {
    tl_pll2_state_t pll2; /**< Tracking loop filter state. */
    tl_pll3_state_t pll3; /**< Tracking loop filter state. */
    tl_fll1_state_t fll1; /**< Tracking loop filter state. */
    tl_fll2_state_t fll2; /**< Tracking loop filter state. */
  };
  tp_ctrl_e ctrl;
} tp_tl_state_t;

/**
 * Generic tracker data
 */
typedef struct {
  tp_tl_state_t     tl_state;               /**< Tracking loop filter state. */
  tp_corr_state_t   corrs;                  /**< Correlations */
  track_cn0_state_t cn0_est;                /**< C/N0 estimator state. */
  alias_detect_t    alias_detect;           /**< Alias lock detector. */
  lock_detect_t     lock_detect;            /**< Lock detector state. */
  u8                tracking_mode: 3;       /**< Tracking mode */
  u8                cycle_no: 5;            /**< Cycle index inside current
                                             *   integration mode. */
  u8                use_alias_detection: 1; /**< Flag for alias detection control */
  u8                has_next_params: 1;     /**< Flag if stage transition is in
                                             *   progress */
  u8                confirmed: 1;           /**< Flag if the tracking is confirmed */
} tp_tracker_data_t;

/**
 * Common tracker configuration container.
 */
typedef struct {
  bool show_unconfirmed_trackers; /**< Flag to control reporting of unconfirmed
                                   *   tracker channels */
} tp_tracker_config_t;

/**
 * Macro for default tracker parameters initialization
 */
#define TP_TRACKER_DEFAULT_CONFIG {false}

/**
 * Registers common configuration parameters for a tracker section.
 *
 * \param[in]     section Configuration section name.
 * \param[in,out] config  Tracker parameters container.
 *
 * \return None
 */
#define TP_TRACKER_REGISTER_CONFIG(section, config) \
  do {\
    SETTING((section), "show_unconfirmed", \
            (config).show_unconfirmed_trackers, TYPE_BOOL); \
  } while (0)

u8 tp_next_cycle_counter(tp_tm_e tracking_mode,
                         u8 cycle_no);

u32 tp_get_cycle_flags(tp_tm_e tracking_mode,
                       u8 cycle_no);


u32 tp_compute_cycle_parameters(tp_tm_e tracking_mode,
                                u8 cycle_no);

u8 tp_get_cycle_count(tp_tm_e tracking_mode);
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode,
                                 u8 cycle_no);
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode,
                                   u8 cycle_no);
u8 tp_get_cn0_ms(tp_tm_e tracking_mode);
u8 tp_get_ld_ms(tp_tm_e tracking_mode);
u8 tp_get_alias_ms(tp_tm_e tracking_mode);
u8 tp_get_flld_ms(tp_tm_e tracking_mode);
u8 tp_get_flll_ms(tp_tm_e tracking_mode);
u8 tp_get_bit_ms(tp_tm_e tracking_mode);
u8 tp_get_pll_ms(tp_tm_e tracking_mode);
u8 tp_get_dll_ms(tp_tm_e tracking_mode);
const char *tp_get_mode_str(tp_tm_e v);
bool tp_is_pll_ctrl(tp_ctrl_e ctrl);
bool tp_is_fll_ctrl(tp_ctrl_e ctrl);

void tp_update_correlators(u32 cycle_flags,
                           const tp_epl_corr_t * restrict cs_now,
                           tp_corr_state_t * restrict corr_state);

void tp_tl_init(tp_tl_state_t *s,
                tp_ctrl_e ctrl,
                const tl_rates_t *rates,
                const tl_config_t *config);

void tp_tl_retune(tp_tl_state_t *s,
                  tp_ctrl_e ctrl,
                  const tl_config_t *config);

void tp_tl_adjust(tp_tl_state_t *s, float err);
void tp_tl_get_rates(tp_tl_state_t *s, tl_rates_t *rates);
void tp_tl_get_config(const tp_loop_params_t *l, tl_config_t *config);
void tp_tl_update(tp_tl_state_t *s, const tp_epl_corr_t *cs);
float tp_tl_get_dll_error(tp_tl_state_t *s);
bool tp_tl_is_pll(const tp_tl_state_t *s);
bool tp_tl_is_fll(const tp_tl_state_t *s);
void tp_tl_fll_update_first(tp_tl_state_t *s, corr_t cs);
void tp_tl_fll_update_second(tp_tl_state_t *s, corr_t cs);
void tp_tl_fll_update(tp_tl_state_t *s);

/* Generic tracker functions */
void tp_tracker_register_parameters(const char *section,
                                    tp_tracker_config_t *config);

void tp_tracker_init(const tracker_channel_info_t *channel_info,
                     tracker_common_data_t *common_data,
                     tp_tracker_data_t *data,
                     const tp_tracker_config_t *config);
void tp_tracker_disable(const tracker_channel_info_t *channel_info,
                        tracker_common_data_t *common_data);
u32 tp_tracker_update(const tracker_channel_info_t *channel_info,
                      tracker_common_data_t *common_data,
                      tp_tracker_data_t *data);
void tp_tracker_update_parameters(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tp_tracker_data_t *data,
                                  const tp_config_t *next_params,
                                  bool init);
void tp_tracker_update_correlators(const tracker_channel_info_t *channel_info,
                                   tracker_common_data_t *common_data,
                                   tp_tracker_data_t *data,
                                   u32 cycle_flags);
void tp_tracker_update_bsync(const tracker_channel_info_t *channel_info,
                             tp_tracker_data_t *data,
                             u32 cycle_flags);
void tp_tracker_update_tow(const tracker_channel_info_t *channel_info,
                           tracker_common_data_t *common_data,
                           tp_tracker_data_t *data,
                           u32 cycle_flags,
                           u64 sample_time_tk);
void tp_tracker_update_cn0(const tracker_channel_info_t *channel_info,
                           tracker_common_data_t *common_data,
                           tp_tracker_data_t *data,
                           u32 cycle_flags);
void tp_tracker_update_locks(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             tp_tracker_data_t *data,
                             u32 cycle_flags);
void tp_tracker_update_fll(tp_tracker_data_t *data, u32 cycle_flags);
void tp_tracker_update_pll_dll(const tracker_channel_info_t *channel_info,
                               tracker_common_data_t *common_data,
                               tp_tracker_data_t *data,
                               u32 cycle_flags);
void tp_tracker_update_alias(const tracker_channel_info_t *channel_info,
                             tracker_common_data_t *common_data,
                             tp_tracker_data_t *data,
                             u32 cycle_flags);
void tp_tracker_update_mode(const tracker_channel_info_t *channel_info,
                            tracker_common_data_t *common_data,
                            tp_tracker_data_t *data);
u32 tp_tracker_compute_rollover_count(const tracker_channel_info_t *channel_info,
                                      const tp_tracker_data_t *data);
void tp_tracker_update_cycle_counter(tp_tracker_data_t *data);

#endif /* SWIFTNAV_TRACK_PROFILE_UTILS_H_ */
