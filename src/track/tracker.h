/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACKER_H
#define SWIFTNAV_TRACKER_H

#include "bit_sync/bit_sync.h"
#include "board/nap/nap_common.h"
#include "hal/piksi_systime.h"
#include "lock_detector/lock_detector.h"
#include "nav_bit_fifo/nav_bit_fifo.h"
#include "nav_data_sync/nav_data_sync.h"
#include "timing/timing.h"
#include "track_cfg.h"
#include "track_cn0.h"
#include "track_loop/trk_loop_common.h"
#include "track_timer.h"

#define TP_CORR_DIM 3
#define TP_CORR_DATAPILOT_DIM (2 * TP_CORR_DIM)

#define THRESH_SENS_DBHZ 25
#define THRESH_20MS_DBHZ 32

/**
 * Tracking mode enumeration.
 */
typedef enum {
  TP_TM_INITIAL = 0, /**< Initial tracking mode */

  TP_TM_1MS_20MS, /**< 1 ms */
  TP_TM_1MS_10MS, /**< 1 ms */
  TP_TM_1MS_2MS,  /**< 1 ms */
  TP_TM_1MS_SC4,  /**< 1 ms */

  TP_TM_2MS_20MS, /**< 2 ms */
  TP_TM_2MS_10MS, /**< 2 ms */
  TP_TM_2MS_2MS,  /**< 2 ms */
  TP_TM_2MS_SC4,  /**< 2 ms */

  TP_TM_4MS_SC4, /**< 4 ms */

  TP_TM_5MS_20MS, /**< 5 ms */
  TP_TM_5MS_10MS, /**< 5 ms */

  TP_TM_10MS_20MS, /**< 10 ms */
  TP_TM_10MS_10MS, /**< 10 ms */
  TP_TM_10MS_SC4,  /**< 10 ms */

  TP_TM_20MS_20MS,      /**< 20 ms */
  TP_TM_20MS_20MS_BASE, /**< 20 ms */
  TP_TM_20MS_10MS,      /**< 20 ms */
  TP_TM_20MS_10MS_BASE, /**< 10 ms */
  TP_TM_20MS_SC4,       /**< 20 ms */
  TP_TM_20MS_SC4_BASE,  /**< 20 ms */

  TP_TM_200MS_20MS,    /**< 200 ms */
  TP_TM_200MS_10MS,    /**< 200 ms */
  TP_TM_200MS_10MS_NM, /**< 200 ms GLO No Meander */
  TP_TM_200MS_2MS,     /**< 200 ms */
  TP_TM_200MS_SC4,     /**< 200 ms */

  TP_TM_COUNT
} tp_tm_e;

/**
 * Controller types.
 */
typedef enum {
  TP_CTRL_PLL2, /**< 2nd order PLL, 1st order DLL */
  TP_CTRL_PLL3, /**< 2nd order FLL, 3rd order PLL, 2nd order DLL */
} tp_ctrl_e;

/** Supported channel drop reasons */
typedef enum {
  CH_DROP_REASON_ERROR,        /**< Tracking channel error */
  CH_DROP_REASON_MASKED,       /**< Tracking channel is disabled by mask */
  CH_DROP_REASON_NO_BIT_SYNC,  /**< Bit sync timeout */
  CH_DROP_REASON_NO_PLOCK,     /**< Pessimistic lock timeout */
  CH_DROP_REASON_LOW_CN0,      /**< Low C/N0 for too long */
  CH_DROP_REASON_XCORR,        /**< Confirmed cross-correlation */
  CH_DROP_REASON_NO_UPDATES,   /**< No tracker updates for too long */
  CH_DROP_REASON_SV_UNHEALTHY, /**< The SV is Unhealthy */
  CH_DROP_REASON_LEAP_SECOND,  /**< Leap second event is imminent,
                                    drop GLO satellites */
  CH_DROP_REASON_OUTLIER,      /**< Doppler outlier */
  CH_DROP_REASON_SBAS_PROVIDER_CHANGE, /**< SBAS provider change */
  CH_DROP_REASON_RAIM,                 /**< Signal removed by RAIM */
  CH_DROP_REASON_NEW_MODE              /**< New tracker mode */
} ch_drop_reason_t;

struct profile_vars {
  u8 index;
  float pll_bw;
  float fll_bw;
};

/**
 * Tracking loop parameters.
 */
typedef struct {
  float code_bw;      /**< Code tracking noise bandwidth in Hz */
  float code_zeta;    /**< Code tracking loop damping ratio */
  float code_k;       /**< Code tracking loop gain coefficient */
  float carr_to_code; /**< */
  float pll_bw;       /**< PLL BW [Hz] */
  float carr_zeta;    /**< Carrier tracking loop damping ratio */
  float carr_k;       /**< Carrier tracking loop gain coefficient */
  float fll_bw;       /**< FLL BW [Hz] */
  tp_tm_e mode;       /**< Operation mode */
  tp_ctrl_e ctrl;     /**< Controller type */
} tp_loop_params_t;

/**
 * Lock detector parameters.
 */
typedef struct {
  float k1; /**< LPF coefficient */
  float k2; /**< I scale factor */
  u16 lp;   /**< Pessimistic count threshold */
} tp_lock_detect_params_t;

/**
 * CN0 thresholds.
 */
typedef struct {
  float use_dbhz;
  float drop_dbhz;
  float ambiguity_dbhz;
} tp_cn0_thres_t;

/**
 * Tracking profile controller data.
 *
 * The controller uses this container for managing profile switching logic.
 */
typedef struct {
  float cn0_offset; /**< C/N0 offset in dB to tune thresholds */
  float filt_cn0;   /**< C/N0 value for decision logic */

  struct profile_vars cur;  /**< Current profile variables */
  struct profile_vars next; /**< Next profile variables */

  tp_loop_params_t loop_params; /**< Tracking loop parameters */
  /** Phase lock detector parameters */
  tp_lock_detect_params_t ld_phase_params;
  /** Freq lock detector parameters */
  tp_lock_detect_params_t ld_freq_params;
  tp_cn0_thres_t cn0_thres;

  tracker_timer_t profile_settle_timer;

  const struct tp_profile_entry *profiles; /**< Profiles switching table. */
} tp_profile_t;

/** Counter type. Value changes every time tracking mode changes. */
typedef u32 update_count_t;

/** Controller parameters for error sigma computations */
typedef struct {
  float fll_bw; /**< FLL controller NBW [Hz].
                     Single sided noise bandwidth in case of
                     FLL and FLL-assisted PLL tracking */
  float pll_bw; /**< PLL controller noise bandwidth [Hz].
                     Single sided noise bandwidth in case of
                     PLL and FLL-assisted PLL tracking */
  float dll_bw; /**< DLL controller noise bandwidth [Hz]. */
  u8 int_ms;    /**< PLL/FLL controller integration time [ms] */
} track_ctrl_params_t;

/** Parameters for half-cycle ambiguity resolution */
typedef struct {
  s8 polarity; /**< Polarity of the matching carrier phases */
  bool synced; /**< Flag for indicating half-cycle ambiguity resolution */
} cp_sync_t;

/** \addtogroup tracking
 * \{ */
/* Bitfield */
typedef enum {
  ERROR_FLAG_NONE = 0x00,
  ERROR_FLAG_MISSED_UPDATE = 0x01,
  ERROR_FLAG_INTERRUPT_WHILE_DISABLED = 0x02,
} error_flag_t;

/**
 * Generic tracking channel information for external use.
 */
typedef struct {
  u8 id;                  /**< Channel identifier */
  me_gnss_signal_t mesid; /**< ME signal identifier */
  u16 glo_orbit_slot;     /**< GLO orbital slot */
  u32 flags;              /**< Tracker flags TRACKER_FLAG_... */
  s32 tow_ms;             /**< ToW [ms] or TOW_UNKNOWN */
  s32 tow_residual_ns;    /**< Residual to tow_ms [ns] */
  float cn0;              /**< C/N0 [dB/Hz] */
  u32 sample_count;       /**< Last measurement sample counter */
  u16 lock_counter;       /**< Lock state counter */
  float xcorr_freq;       /**< Cross-correlation doppler [hz] */
  u16 xcorr_count;        /**< Cross-correlation counter */
  bool xcorr_wl;          /**< Is signal xcorr whitelisted? */
} tracker_info_t;

/**
 * Timing information from tracking channel for external use.
 */
/** Controller parameters for error sigma computations */
typedef struct {
  float fll_bw; /**< FLL controller NBW [Hz].
                     Single sided noise bandwidth in case of
                     FLL and FLL-assisted PLL tracking */
  float pll_bw; /**< PLL controller noise bandwidth [Hz].
                     Single sided noise bandwidth in case of
                     PLL and FLL-assisted PLL tracking */
  float dll_bw; /**< DLL controller noise bandwidth [Hz]. */
  u8 int_ms;    /**< PLL/FLL controller integration time [ms] */
} tracker_ctrl_info_t;

typedef struct {
  s32 value;        /**< Carrier phase offset value [cycles] */
  u64 timestamp_ms; /**< Carrier phase offset timestamp [ms] */
} tracker_cpo_t;

/**
 * Phase and frequencies information
 */
typedef struct {
  double code_phase_chips; /**< The code-phase in chips at `receiver_time` */
  double code_phase_rate;  /**< Code phase rate in chips/s */
  double carrier_phase;    /**< Carrier phase in cycles */
  double carrier_freq;     /**< Carrier frequency in Hz */
  double carrier_freq_at_lock; /**< Carrier frequency at last lock time */
  tracker_cpo_t cpo;           /**< Carrier phase offset */
} tracker_freq_info_t;

/**
 * Tracker loop state.
 */
typedef struct {
  union {
    tl_pll2_state_t pll2; /**< Tracking loop filter state */
    tl_pll3_state_t pll3; /**< Tracking loop filter state */
  };
  tp_ctrl_e ctrl;
} tp_tl_state_t;

/**
 * EPL accumulator structure.
 */
typedef struct {
  union {
    corr_t all[TP_CORR_DATAPILOT_DIM]; /**< E|P|L accumulators as a vector */
    struct {
      corr_t early;     /**< Early accumulator */
      corr_t prompt;    /**< Prompt accumulator */
      corr_t late;      /**< Late accumulator */
      corr_t dp_early;  /**< Data/Pilot Early accumulator */
      corr_t dp_prompt; /**< Data/Pilot Prompt accumulator */
      corr_t dp_late;   /**< Data/Pilot Late accumulator */
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
 * - Lock detector
 * - Bit synchronization and message decoding
 */
typedef struct {
  tp_epl_corr_t corr_main; /**< EPL correlation results for DLL and PLL
                            *   in correlation period. */
  corr_t corr_cn0;         /**< C/N0 accumulators */
  corr_t corr_fll;         /**< FLL accumulator */
  corr_t corr_ld;          /**< Lock detector accumulator */
  corr_t corr_bit;         /**< Bit sync accumulator */
} tp_corr_state_t;

/**
 * GPS L1 C/A tracker data container type.
 */
typedef struct {
  u16 xcorr_counts[NUM_SATS_GPS]; /**< L1 Cross-correlation interval counters */
  u16 xcorr_count_l2;             /**< L2 Cross-correlation interval counter */
  u16 xcorr_whitelist_counts[NUM_SATS_GPS]; /**< L1 whitelist interval counters
                                             */
  bool xcorr_whitelist[NUM_SATS_GPS]; /**< L1 Cross-correlation whitelist status
                                       */
  bool xcorr_whitelist_l2; /**< L2 Cross-correlation whitelist status */
  u8 xcorr_flag : 1;       /**< Cross-correlation flag */
} gps_l1ca_tracker_data_t;

/**
 * GPS L2C tracker data container type.
 */
typedef struct {
  u16 xcorr_count_l1;      /**< L1 Cross-correlation interval count */
  bool xcorr_whitelist;    /**< Cross-correlation whitelist status */
  bool xcorr_whitelist_l1; /**< L1 Cross-correlation whitelist status */
  u8 xcorr_flag : 1;       /**< Cross-correlation flag */
} gps_l2cm_tracker_data_t;

/** Top-level generic tracker channel. */
typedef struct {
  /* This portion of the structure is not cleaned-up at tracker channel start */
  /** State of this channel. */
  bool busy;

  /** Mutex used to permit atomic reads of channel data. */
  mutex_t mutex;

  /* The data to be cleaned-up at init must be placed below */
  int cleanup_region_start;

  u8 nap_channel;         /**< Associated NAP channel. */
  me_gnss_signal_t mesid; /**< Current ME signal being decoded. */
  u16 glo_orbit_slot;     /**< GLO orbital slot. */
  u16 glo_into_string_ms; /**< ms into GLO string. Modulo GLO_STRING_LENGTH_MS*/

  /** FIFO for navigation message bits. */
  nav_bit_fifo_t nav_bit_fifo;
  /** Used to sync time decoded from navigation message
   * back to tracking channel. */
  nav_data_sync_t nav_data_sync;
  /** Time since last nav bit was appended to the nav bit FIFO. */
  u32 nav_bit_offset_ms;
  /** Bit sync state. */
  bit_sync_t bit_sync;
  /** Polarity of nav message bits. */
  s8 bit_polarity;
  /** Increments when tracking new signal. */
  u16 lock_counter;
  /** Set if this channel should output I/Q samples on SBP. */
  bool output_iq;
  /** Flags if PRN conformity check failed */
  bool prn_check_fail;
  /** Flags if tracker is cross-correlated */
  bool xcorr_flag;

  update_count_t update_count; /**< Number of ms channel has been running */
  tracker_timer_t cn0_below_drop_thres_timer;

  tracker_timer_t unlocked_timer;
  /**< update_count value when pessimistic
       phase detector has changed last time. */
  update_count_t xcorr_change_count;
  /**< update count value when cross-correlation
       flag has changed last time */
  s32 TOW_ms;               /**< TOW in ms. */
  s32 TOW_ms_prev;          /**< previous TOW in ms. */
  s32 TOW_residual_ns;      /**< Residual to TOW_ms [ns] */
  u64 sample_count;         /**< Total num samples channel has tracked for. */
  double code_phase_prompt; /**< Prompt code phase in chips. */
  double code_phase_rate;   /**< Code phase rate in chips/s. */
  double carrier_phase;     /**< Carrier phase in cycles. */
  double carrier_freq;      /**< Carrier frequency Hz. */
  double carrier_freq_prev; /**< Carrier frequency Hz. */
  bool carrier_freq_prev_valid; /**< carrier_freq_prev is valid. */
  /** carrier_freq_prev age timer */
  tracker_timer_t carrier_freq_age_timer;

  double carrier_freq_at_lock; /**< Carrier frequency snapshot in the presence
                                    of PLL/FLL pessimistic locks [Hz]. */
  float unfiltered_freq_error; /**< Unfiltered frequency error at the FLL
                                    discriminator output [Hz]. */
  float cn0;                   /**< Current estimate of C/N0. */
  u32 flags;                   /**< Tracker flags TRACKER_FLAG_... */
  ch_drop_reason_t ch_drop_reason; /* Drop reason if TRACKER_FLAG_DROP is set */
  float xcorr_freq;                /**< Doppler for cross-correlation [Hz] */
  tracker_timer_t age_timer;       /**< Tracking channel age timer */
  tracker_timer_t update_timer;    /**< Tracking channel last update timer */
  bool updated_once;               /**< Tracker was updated at least once */

  cp_sync_t cp_sync; /**< Half-cycle ambiguity resolution */

  /** Flags if carrier phase integer offset to be reset. */
  bool reset_cpo;
  /** Carrier phase offset information */
  tracker_cpo_t cpo;

  tp_profile_t profile; /**< Profile controller state. */

  u16 bit_cnt; /**< navbit counter */

  tp_tl_state_t tl_state;    /**< Tracking loop filter state. */
  tp_corr_state_t corrs;     /**< Correlations */
  track_cn0_state_t cn0_est; /**< C/N0 estimator state. */
  lock_detect_t ld_phase;    /**< Phase lock detector state. */
  lock_detect_t ld_freq;     /**< Frequency lock detector state. */
  lp1_filter_t xcorr_filter; /**< Low-pass SV POV doppler filter */
  tp_tm_e tracking_mode;     /**< Tracking mode */
  u16 cycle_no;              /**< Cycle index inside current
                              *   integration mode. */
  u16 has_next_params : 1;   /**< Flag if stage transition is in
                              *   progress */

  /* Constellation specific data */
  union {
    gps_l1ca_tracker_data_t gps_l1ca;
    gps_l2cm_tracker_data_t gps_l2cm;
  };

  u16 fpll_cycle; /**< FPLL run cycle within current profile */

  corr_t correlators[20];

  tracker_timer_t init_settle_timer;
} tracker_t;

/** \} */

#endif /* SWIFTNAV_TRACKER_H  */
