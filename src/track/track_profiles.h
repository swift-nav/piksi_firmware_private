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

#ifndef SWIFTNAV_TRACK_PROFILES_H_
#define SWIFTNAV_TRACK_PROFILES_H_

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include "track_cn0.h"

/** \addtogroup track
 * \{ */

/** \addtogroup track_loop
 * \{ */

/**
 * Tracking mode enumeration.
 */
typedef enum
{
  TP_TM_INITIAL,      /**< Initial tracking mode (same as pipelining otherwise) */
  TP_TM_ONE_PLUS_N,   /**< Integration period split: 1+N */
  TP_TM_ONE_PLUS_N5,  /**< Integration period split: 1+4+5N */
  TP_TM_ONE_PLUS_N10, /**< Integration period split: 1+9+10N */
  TP_TM_ONE_PLUS_N20, /**< Integration period split: 1+19+20N */
} tp_tm_e;

/**
 * Controller types.
 */
typedef enum
{
  TP_CTRL_PLL2,      /**< First order FLL, second order PLL */
  TP_CTRL_PLL3,      /**< Third order PLL. */
  TP_CTRL_FLL1,      /**< First order FLL. */
  TP_CTRL_FLL2,      /**< Second order FLL. */
} tp_ctrl_e;

/**
 * Tracking loop parameters.
 */
typedef struct
{
  float   code_bw;            /**< Code tracking noise bandwidth in Hz */
  float   code_zeta;          /**< Code tracking loop damping ratio */
  float   code_k;             /**< Code tracking loop gain coefficient */
  float   carr_to_code;       /**< */
  float   carr_bw;            /**< Carrier tracking loop noise bandwidth in Hz */
  float   carr_zeta;          /**< Carrier tracking loop damping ratio */
  float   carr_k;             /**< Carrier tracking loop gain coefficient */
  float   carr_fll_aid_gain;  /**< FLL assistance coefficient */
  u8      coherent_ms;        /**< Coherent integration period in ms */
  tp_tm_e mode;               /**< Operation mode */
  tp_ctrl_e ctrl;             /**< Operation mode */
} tp_loop_params_t;

/**
 * Lock detector parameters.
 */
typedef struct {
  float   k1;                 /**< LPF coefficient */
  float   k2;                 /**< I scale factor */
  u16     lp;                 /**< Pessimistic count threshold */
  u16     lo;                 /**< Optimistic count threshold */
} tp_lock_detect_params_t;

/**
 * Lock detector parameters.
 */
typedef struct {
  track_cn0_est_e est;

  float track_cn0_use_thres; /* dBHz */
  float track_cn0_drop_thres;
} tp_cn0_params_t;

/**
 * Tracking loop configuration container.
 *
 * \sa tp_get_profile
 */
typedef struct
{
  tp_loop_params_t        loop_params;        /**< Tracking loop parameters */
  tp_lock_detect_params_t lock_detect_params; /**< Lock detector parameters */
  bool                    use_alias_detection; /**< Alias detection flag */
  tp_cn0_params_t         cn0_params;
} tp_config_t;

/**
 * Tracking loop data.
 *
 * This structure contains tracking parameters required for profile changing
 * decision making.
 */
typedef struct
{
  double code_phase_rate; /**< Code frequency in Hz */
  double carr_freq;       /**< Carrier frequency in Hz */
  float  cn0;             /**< Computed C/N0 (filtered) in dB/Hz */
  float  cn0_raw;         /**< Computed C/N0 (raw) in dB/Hz */
  u32    plock:1;         /**< Pessimistic lock flag */
  u32    olock:1;         /**< Optimistic lock flag */
  u32    bsync:1;         /**< Bit sync flag */
  u32    time_ms:8;       /**< Time in milliseconds */
  u32    sample_count;    /**< Channel sample count */
  float  lock_i;          /**< Filtered I value from the lock detector */
  float  lock_q;          /**< Filtered Q value from the lock detector */
  float  lock_f;          /**< DLL/FLL lock detector */
} tp_report_t;

/**
 * Tracking profile result codes.
 */
typedef enum
{
  TP_RESULT_SUCCESS = 0, /**< Successful operation. */
  TP_RESULT_ERROR = -1,  /**< Error during operation */
  TP_RESULT_NO_DATA = 1, /**< Profile has changed */
} tp_result_e;

#ifdef __cplusplus
extern "C"
{
#endif

tp_result_e tp_init();
tp_result_e tp_tracking_start(gnss_signal_t sid, const tp_report_t *data,
                              tp_config_t *config);
tp_result_e tp_tracking_stop(gnss_signal_t sid);
tp_result_e tp_get_profile(gnss_signal_t sid, tp_config_t *config, bool commit);
tp_result_e tp_get_cn0_params(gnss_signal_t sid, tp_cn0_params_t *cn0_params);
bool        tp_has_new_profile(gnss_signal_t sid);
const tp_loop_params_t *tp_get_next_loop_params(gnss_signal_t sid);
tp_result_e tp_report_data(gnss_signal_t sid, const tp_report_t *data);

#ifdef __cplusplus
}
#endif

/** \} */
/** \} */

#endif /* SWIFTNAV_TRACK_PROFILES_H_ */
