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
#include "track_api.h"

/** \addtogroup track
 * \{ */

/** \addtogroup track_loop
 * \{ */

/** Unknown delay indicator */
#define TP_DELAY_UNKNOWN -1

/**
 * Tracking mode enumeration.
 */
typedef enum
{
  TP_TM_GPS_INITIAL, /**< Initial tracking mode (same as pipelining otherwise) */
  TP_TM_GPS_DYN,     /**< Dynamics tracking mode */
  TP_TM_GPS_5MS,     /**< GPS 5 ms PLL/DLL */
  TP_TM_GPS_10MS,    /**< GPS 10 ms PLL/DLL */
  TP_TM_GPS_20MS,    /**< GPS 20 ms PLL/DLL */
} tp_tm_e;

/**
 * Controller types.
 */
typedef enum
{
  TP_CTRL_PLL2, /**< First order FLL, second order PLL */
  TP_CTRL_PLL3, /**< Second order FLL, third order PLL */
  TP_CTRL_FLL1, /**< First order FLL. */
  TP_CTRL_FLL2, /**< Second order FLL. */
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
  float   fll_bw;             /**< FLL BW */
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
  float  acceleration;    /**< Acceleration in Hz/s */
  float  cn0;             /**< Computed C/N0 (filtered) in dB/Hz */
  float  cn0_raw;         /**< Computed C/N0 (raw) in dB/Hz */
  u32    plock:1;         /**< Pessimistic lock flag */
  u32    olock:1;         /**< Optimistic lock flag */
  u32    bsync:1;         /**< Bit sync flag */
  u32    time_ms:8;       /**< Time in milliseconds */
  u32    sample_count;    /**< Channel sample count */
  float  lock_i;          /**< Filtered I value from the lock detector */
  float  lock_q;          /**< Filtered Q value from the lock detector */
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

/**
 * Tracking profile controller data.
 *
 * The controller uses this container for managing profile switching logic.
 */
typedef struct {
  /*
   * Fields are ordered from larger to smaller for minimal memory footprint.
   */

  float cn0_offset;  /**< C/N0 offset in dB to tune thresholds */
  float filt_cn0;    /**< C/N0 value for decision logic */
  float filt_accel;  /**< SV acceleration value for decision logic [g] */

  /* Packed fields: 24 bits */
  u32   olock: 1;          /**< PLL optimistic lock flag */
  u32   plock: 1;          /**< PLL pessimistic lock flag */
  u32   bsync: 1;          /**< Bit sync flag */
  u32   bsync_sticky: 1;   /**< Bit sync flag */
  u32   profile_update: 1; /**< Flag if the profile update is required */
  u32   low_cn0_count: 5;  /**< State lock counter for C/N0 threshold */
  u32   high_cn0_count: 5; /**< State lock counter for C/N0 threshold */
  u32   accel_count: 5;    /**< State lock counter for dynamics threshold */
  u32   accel_count_idx: 2;/**< State lock value for dynamics threshold */
  u32   cn0_est: 2;        /**< C/N0 estimator type */
  u16   lock_time_ms;      /**< Profile lock count down timer */
  u8    cur_index;         /**< Active profile index [0-25] */
  u8    next_index;        /**< Next profile index [0-25] */
  u16   acceleration_ends_after_ms; /**< There is an acceleration if this
                                     *   parameter is non-zero [ms] */
  u16   print_time;        /**< Time till next debug print [ms] */
  u32   time_snapshot_ms;  /**< Time snapshot [ms] */
  s16   bs_delay_ms;       /**< Bit sync delay [ms] or TP_DELAY_UNKNOWN */
  s16   plock_delay_ms;    /**< Pessimistic lock delay [ms] or TP_DELAY_UNKNOWN */

  const struct tp_profile_entry *profiles; /**< Profiles switching table. */
} tp_profile_t;


#ifdef __cplusplus
extern "C"
{
#endif

tp_result_e tp_init(void);
tp_result_e tp_profile_init(gnss_signal_t sid,
                            tp_profile_t *profile,
                            const tp_report_t *data,
                            tp_config_t *config);
tp_result_e tp_profile_get_config(gnss_signal_t sid,
                                  tp_profile_t *profile,
                                  tp_config_t *config,
                                  bool commit);
tp_result_e tp_profile_get_cn0_params(const tp_profile_t *profile,
                                      tp_cn0_params_t *cn0_params);
bool        tp_profile_has_new_profile(gnss_signal_t sid,
                                       tp_profile_t *profile);
u8          tp_profile_get_next_loop_params_ms(const tp_profile_t *profile);
tp_result_e tp_profile_report_data(gnss_signal_t sid,
                                   tp_profile_t *profile,
                                   const tracker_common_data_t *common_data,
                                   const tp_report_t *data);

#ifdef __cplusplus
}
#endif

/** \} */
/** \} */

#endif /* SWIFTNAV_TRACK_PROFILES_H_ */
