/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 * Contact: Jacob McNamee <jacob@swiftnav.com>
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
#include <libswiftnav/nav_msg.h>
#include <libswiftnav/track.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/run_stats.h>
#include <libswiftnav/bit_sync.h>

#include <ch.h>
#include "shm.h"
#include "track_flags.h"

#include "board/nap/track_channel.h"
#include <platform_signal.h>
#include "board/nap/nap_common.h"
#include "track/track_cn0.h"
#include <settings.h>

#define NAV_BIT_FIFO_SIZE 64 /**< Size of nav bit FIFO. Must be a power of 2 */

#define TP_DLL_PLL_MEAS_DIM 5

#define NAV_BIT_FIFO_INDEX_MASK ((NAV_BIT_FIFO_SIZE) - 1)
#define NAV_BIT_FIFO_INDEX_DIFF(write_index, read_index) \
          ((nav_bit_fifo_index_t)((write_index) - (read_index)))
#define NAV_BIT_FIFO_LENGTH(p_fifo) \
          (NAV_BIT_FIFO_INDEX_DIFF((p_fifo)->write_index, (p_fifo)->read_index))

/** Unknown delay indicator */
#define TP_DELAY_UNKNOWN -1

/** How many milliseconds to wait for the tracking loops to
 * stabilize after any mode change before using obs. */
#define TRACK_STABILIZATION_T 1000

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
 * Tracking mode enumeration.
 */
typedef enum
{
  TP_TM_INITIAL, /**< Initial tracking mode (same as pipelining otherwise) */
  TP_TM_1MS,     /**< 1 ms PLL/DLL */
  TP_TM_5MS,     /**< 5 ms PLL/DLL */
  TP_TM_10MS,    /**< 10 ms PLL/DLL */
  TP_TM_20MS,    /**< 20 ms PLL/DLL */
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

typedef struct {
  s8 soft_bit;
  bool sensitivity_mode;
} nav_bit_fifo_element_t;

typedef u8 nav_bit_fifo_index_t;

typedef struct {
  nav_bit_fifo_index_t read_index;
  nav_bit_fifo_index_t write_index;
  nav_bit_fifo_element_t elements[NAV_BIT_FIFO_SIZE];
} nav_bit_fifo_t;

typedef struct {
  s32 TOW_ms;
  s32 TOW_residual_ns; /**< Residual to TOW_ms [ns] */
  s8 bit_polarity;
  u16 glo_orbit_slot;
  nav_bit_fifo_index_t read_index;
  glo_health_t glo_health;
  bool valid;
} nav_data_sync_t;

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
  u32   cn0_est: 2;        /**< C/N0 estimator type */
  u16   lock_time_ms;      /**< Profile lock count down timer */
  u8    cur_index;         /**< Active profile index [0-37] */
  u8    next_index;        /**< Next profile index [0-37] */
  u16   acceleration_ends_after_ms; /**< There is an acceleration if this
                                     *   parameter is non-zero [ms] */
  u16   print_time;        /**< Time till next debug print [ms] */
  u32   time_snapshot_ms;  /**< Time snapshot [ms] */
  s16   bs_delay_ms;       /**< Bit sync delay [ms] or TP_DELAY_UNKNOWN */
  s16   plock_delay_ms;    /**< Pessimistic lock delay [ms] or TP_DELAY_UNKNOWN */

  const struct tp_profile_entry *profiles; /**< Profiles switching table. */
} tp_profile_t;

/** \} */

/** Counter type. Value changes every time tracking mode changes. */
typedef u32 update_count_t;

/** Controller parameters for error sigma computations */
typedef struct {
  float fll_bw;  /**< FLL controller NBW [Hz].
                      Single sided noise bandwidth in case of
                      FLL and FLL-assisted PLL tracking */
  float pll_bw;  /**< PLL controller noise bandwidth [Hz].
                      Single sided noise bandwidth in case of
                      PLL and FLL-assisted PLL tracking */
  float dll_bw;  /**< DLL controller noise bandwidth [Hz]. */
  u8    int_ms;  /**< PLL/FLL controller integration time [ms] */
} track_ctrl_params_t;

/** Parameters for half-cycle ambiguity resolution */
typedef struct {
  u8 counter;  /**< Counter for matching carrier phases */
  s8 polarity; /**< Polarity of the matching carrier phases */
  bool synced; /**< Flag for indicating half-cycle ambiguity resolution */
} cp_sync_t;

/** \} */

/** \addtogroup tracking
 * \{ */

#define TRACKING_AZIMUTH_UNKNOWN 400
#define TRACKING_ELEVATION_UNKNOWN 100 /* Ensure it will be above elev. mask */
/** GPS L1 C/A cross-correlation frequency step [hz] */
#define L1CA_XCORR_FREQ_STEP 1000.f
/** GPS L1 C/A CN0 threshold for whitelisting [dB-Hz] */
#define L1CA_XCORR_WHITELIST_THRESHOLD 40.f
/** GPS L2 CM CN0 threshold for whitelisting [dB-Hz] */
#define L2CM_XCORR_WHITELIST_THRESHOLD 27.f
/** GPS L1 C/A CN0 threshold for suspected xcorr [dB-Hz] */
#define XCORR_SUSPECT_THRESHOLD -15.f
/** GPS L1 C/A CN0 threshold for confirmed xcorr [dB-Hz] */
#define XCORR_CONFIRM_THRESHOLD -20.f
/** cross-correlation update rate [Hz] */
#define XCORR_UPDATE_RATE (SECS_MS / GPS_L1CA_BIT_LENGTH_MS)
/** Carrier phases within tolerance are declared equal. [cycles]
 *  Stable PLL remains within +-15 degree from correct phase.
 *  360 * 0.08 ~= 30 degrees
*/
#define CARRIER_PHASE_TOLERANCE 0.08f
/** counter for half-cycle ambiguity resolution */
#define CARRIER_PHASE_AMBIGUITY_COUNTER 50
/** handover should occur when code phase is near zero [chips] */
#define HANDOVER_CODE_PHASE_THRESHOLD 0.5

typedef u8 tracker_channel_id_t;

/** Maximum SV azimuth/elevation age in seconds: 1 minute is about 0.5 degrees */
#define MAX_AZ_EL_AGE_SEC 60

typedef enum {
  STATE_DISABLED,
  STATE_ENABLED,
  STATE_DISABLE_REQUESTED,
  STATE_DISABLE_WAIT
} state_t;

/* Bitfield */
typedef enum {
  ERROR_FLAG_NONE =                         0x00,
  ERROR_FLAG_MISSED_UPDATE =                0x01,
  ERROR_FLAG_INTERRUPT_WHILE_DISABLED =     0x02,
} error_flag_t;

/**
 * Generic tracking channel information for external use.
 */
typedef struct {
  tracker_channel_id_t     id;           /**< Channel identifier */
  me_gnss_signal_t         mesid;        /**< ME signal identifier */
  u16                      glo_orbit_slot; /**< GLO orbital slot */
  u32                      flags;        /**< Tracker flags TRACKER_FLAG_... */
  s32                      tow_ms;       /**< ToW [ms] or TOW_UNKNOWN */
  s32                      tow_residual_ns;   /**< Residual to tow_ms [ns] */
  float                    cn0;          /**< C/N0 [dB/Hz] */
  u64                      init_timestamp_ms; /**< Tracking channel init
                                                   timestamp [ms] */
  u64                      update_timestamp_ms; /**< Tracking channel last
                                                   update timestamp [ms] */
  u32                      sample_count; /**< Last measurement sample counter */
  u16                      lock_counter; /**< Lock state counter */
  float                    xcorr_freq;   /**< Cross-correlation doppler [hz] */
  u16                      xcorr_count;  /**< Cross-correlation counter */
  bool                     xcorr_wl;     /**< Is signal xcorr whitelisted? */
} tracking_channel_info_t;

/**
 * Timing information from tracking channel for external use.
 */
typedef struct {
  u32 cn0_usable_ms;       /**< Time with C/N0 above drop threshold [ms] */
  u32 cn0_drop_ms;         /**< Time with C/N0 below drop threshold [ms] */
  u32 ld_pess_locked_ms;   /**< Time in pessimistic lock [ms] */
  u32 ld_pess_unlocked_ms; /**< Time without pessimistic lock [ms] */
  u32 last_mode_change_ms; /**< Time since last mode change [ms] */
} tracking_channel_time_info_t;

/** Controller parameters for error sigma computations */
typedef struct {
  float fll_bw;  /**< FLL controller NBW [Hz].
                      Single sided noise bandwidth in case of
                      FLL and FLL-assisted PLL tracking */
  float pll_bw;  /**< PLL controller noise bandwidth [Hz].
                      Single sided noise bandwidth in case of
                      PLL and FLL-assisted PLL tracking */
  float dll_bw;  /**< DLL controller noise bandwidth [Hz]. */
  u8    int_ms;  /**< PLL/FLL controller integration time [ms] */
} tracking_channel_ctrl_info_t;

/** Tracking channel miscellaneous info */
typedef struct {
  double pseudorange;          /**< Pseudorange [m]  */
  double pseudorange_std;      /**< Pseudorange standard deviation [m] */
  struct {
    double value;              /**< Carrier phase offset value [cycles]. */
    u64 timestamp_ms;          /**< Carrier phase offset timestamp [ms] */
  } carrier_phase_offset;      /**< Carrier phase offset */
} tracking_channel_misc_info_t;

/**
 * Phase and frequencies information
 */
typedef struct {
  double code_phase_chips;     /**< The code-phase in chips at `receiver_time`. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  double carrier_phase;        /**< Carrier phase in cycles. */
  double carrier_freq;         /**< Carrier frequency in Hz. */
  double carrier_freq_std;     /**< Carrier frequency std deviation in Hz. */
  double carrier_freq_at_lock; /**< Carrier frequency in Hz at last lock time. */
  float  acceleration;         /**< Acceleration [g] */
} tracking_channel_freq_info_t;

/**
 * Public data segment.
 *
 * Public data segment belongs to a tracking channel and is locked only for
 * a quick update or data fetch operations.
 *
 * The data is grouped according to functional blocks.
 */
typedef struct {
  /** Generic info for externals */
  volatile tracking_channel_info_t      gen_info;
  /** Timing info for externals */
  volatile tracking_channel_time_info_t time_info;
  /** Frequency info for externals */
  volatile tracking_channel_freq_info_t freq_info;
  /** Controller parameters */
  volatile tracking_channel_ctrl_info_t ctrl_info;
  /** Miscellaneous parameters */
  volatile tracking_channel_misc_info_t misc_info;
  /** Carrier frequency products */
  running_stats_t                       carr_freq_stats;
  /** Pseudorange products */
  running_stats_t                       pseudorange_stats;
} tracker_channel_pub_data_t;

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
 * EPL accumulator structure.
 */
typedef struct
{
  union {
    corr_t epl[TP_DLL_PLL_MEAS_DIM]; /**< E|P|L|VE|VL accumulators as a vector */
    struct {
      corr_t early;                  /**< Early accumulator */
      corr_t prompt;                 /**< Prompt accumulator */
      corr_t late;                   /**< Late accumulator */
      corr_t very_early;             /**< Very Early accumulator */
      corr_t very_late;              /**< Very Late accumulator */
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
  tp_epl_corr_t    corr_cn0; /**< C/N0 accumulators */
  corr_t           corr_fll; /**< FLL accumulator */
  corr_t           corr_ad;  /**< False lock (alias) detector accumulator */
  corr_t           corr_ld;  /**< Lock detector accumulator */
  s32              corr_bit; /**< Bit sync accumulator */
} tp_corr_state_t;

struct tracker_interface;

/**
 * GPS L1 C/A tracker data container type.
 */
typedef struct {
  u16 xcorr_counts[NUM_SATS_GPS];     /**< L1 Cross-correlation interval counters */
  u16 xcorr_count_l2;                 /**< L2 Cross-correlation interval counter */
  u16 xcorr_whitelist_counts[NUM_SATS_GPS]; /**< L1 whitelist interval counters */
  bool xcorr_whitelist[NUM_SATS_GPS]; /**< L1 Cross-correlation whitelist status */
  bool xcorr_whitelist_l2;            /**< L2 Cross-correlation whitelist status */
  u8  xcorr_flag: 1;                  /**< Cross-correlation flag */
} gps_l1ca_tracker_data_t;

/**
 * GPS L2C tracker data container type.
 */
typedef struct {
  u16 xcorr_count_l1;      /**< L1 Cross-correlation interval count */
  bool xcorr_whitelist;    /**< Cross-correlation whitelist status */
  bool xcorr_whitelist_l1; /**< L1 Cross-correlation whitelist status */
  u8  xcorr_flag: 1;       /**< Cross-correlation flag */
} gps_l2cm_tracker_data_t;

/** Top-level generic tracker channel. */
typedef struct {
  /* This portion of the structure is not cleaned-up at tracker channel start */

  /** State of this channel. */
  state_t state;

  /** Mutex used to permit atomic reads of channel data. */
  mutex_t mutex;
  /** Mutex used to permit atomic updates of public channel data. */
  mutex_t mutex_pub;

  /* The data to be cleaned-up at init must be placed below */

  int cleanup_region_start;

  me_gnss_signal_t mesid; /**< Current ME signal being decoded. */
  u16 glo_orbit_slot;     /**< GLO orbital slot. */
  u8 nap_channel;         /**< Associated NAP channel. */

  /** Time at which the channel was disabled. */
  systime_t disable_time;
  /** Error flags. May be set at any time by the tracking thread. */
  volatile error_flag_t error_flags;

  /** FIFO for navigation message bits. */
  nav_bit_fifo_t nav_bit_fifo;
  /** Used to sync time decoded from navigation message
   * back to tracking channel. */
  nav_data_sync_t nav_data_sync;
  /** Time since last nav bit was appended to the nav bit FIFO. */
  u32 nav_bit_TOW_offset_ms;
  /** Bit sync state. */
  bit_sync_t bit_sync;
  /** Polarity of nav message bits. */
  s8 bit_polarity;
  /** Increments when tracking new signal. */
  u16 lock_counter;
  /** Set if this channel should output I/Q samples on SBP. */
  bool output_iq;
  /** Flags if carrier phase integer offset to be reset. */
  bool reset_cpo;
  /** Flags if PRN conformity check failed */
  bool prn_check_fail;
  /** Flags if tracker is cross-correlated */
  bool xcorr_flag;

  update_count_t update_count; /**< Number of ms channel has been running */
  update_count_t mode_change_count;
                               /**< update_count at last mode change. */
  update_count_t cn0_below_use_thres_count;
                               /**< update_count value when C/N0 was
                                    last below the use threshold. */
  update_count_t cn0_above_drop_thres_count;
                               /**< update_count value when C/N0 was
                                    last above the drop threshold. */
  update_count_t ld_pess_change_count;
                               /**< update_count value when pessimistic
                                    phase detector has changed last time. */
  update_count_t xcorr_change_count;
                               /**< update count value when cross-correlation
                                    flag has changed last time */
  s32 TOW_ms;                  /**< TOW in ms. */
  s32 TOW_ms_prev;             /**< previous TOW in ms. */
  s32 TOW_residual_ns;         /**< Residual to TOW_ms [ns] */
  u32 sample_count;            /**< Total num samples channel has tracked for. */
  double code_phase_prompt;    /**< Prompt code phase in chips. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  double carrier_phase;        /**< Carrier phase in cycles. */
  double carrier_phase_prev;   /**< Previous carrier phase in cycles. */
  double carrier_freq;         /**< Carrier frequency Hz. */
  double carrier_freq_at_lock; /**< Carrier frequency snapshot in the presence
                                    of PLL/FLL pessimistic locks [Hz]. */
  float cn0;                   /**< Current estimate of C/N0. */
  u32 flags;                   /**< Tracker flags TRACKER_FLAG_... */
  track_ctrl_params_t ctrl_params; /**< Controller parameters */
  float acceleration;          /**< Acceleration [g] */
  float xcorr_freq;            /**< Doppler for cross-correlation [hz] */
  u64 init_timestamp_ms;       /**< Tracking channel init timestamp [ms] */
  u64 update_timestamp_ms;     /**< Tracking channel last update
                                    timestamp [ms] */
  bool updated_once;           /**< Tracker was updated at least once flag. */
  cp_sync_t cp_sync;           /**< Half-cycle ambiguity resolution */
  glo_health_t health;         /**< GLO SV health info */

  /** Associated tracker interface. */
  const struct tracker_interface *interface;
  /** Publicly accessible data */
  tracker_channel_pub_data_t pub_data;

  tp_profile_t      profile;                /**< Profile controller state. */

  tp_tl_state_t     tl_state;               /**< Tracking loop filter state. */
  tp_corr_state_t   corrs;                  /**< Correlations */
  track_cn0_state_t cn0_est;                /**< C/N0 estimator state. */
  alias_detect_t    alias_detect;           /**< Alias lock detector. */
  lock_detect_t     lock_detect;            /**< Lock detector state. */
  lp1_filter_t      xcorr_filter;           /**< Low-pass SV POV doppler filter */
  u16               tracking_mode: 3;       /**< Tracking mode */
  u16               cycle_no: 5;            /**< Cycle index inside current
                                             *   integration mode. */
  u16               use_alias_detection: 1; /**< Flag for alias detection control */
  u16               has_next_params: 1;     /**< Flag if stage transition is in
                                             *   progress */
  u16               confirmed: 1;           /**< Flag if the tracking is confirmed */
  u16               mode_pll: 1;            /**< PLL control flag */
  u16               mode_fll: 1;            /**< FLL control flag */
  u16               xcorr_filter_active: 1; /**< Cross-correlation filter is in use */

  /* Constellation specific data */
  union {
    gps_l1ca_tracker_data_t gps_l1ca;
    gps_l2cm_tracker_data_t gps_l2cm;
  };
} tracker_channel_t;

/** Tracker interface function template. */
typedef void (tracker_interface_function_t)(tracker_channel_t *tracker_channel);

/** Interface to a tracker implementation. */
typedef struct tracker_interface {
  /** Code type for which the implementation may be used. */
  enum code code;
  /** Init function. Called to set up tracker instance when tracking begins. */
  tracker_interface_function_t *init;
  /** Disable function. Called when tracking stops. */
  tracker_interface_function_t *disable;
  /** Update function. Called when new correlation outputs are available. */
  tracker_interface_function_t *update;
} tracker_interface_t;

/** List element passed to tracker_interface_register(). */
typedef struct tracker_interface_list_element_t {
  const tracker_interface_t *interface;
  struct tracker_interface_list_element_t *next;
} tracker_interface_list_element_t;

/**
 * Input entry for cross-correlation processing
 *
 * \sa tracking_channel_cc_data_t
 */
typedef struct {
  tracker_channel_id_t     id;    /**< Tracking channel id */
  u32                      flags; /**< Tracker flags TRACKER_FLAG_... */
  me_gnss_signal_t         mesid; /**< Tracked GNSS ME signal identifier */
  float                    freq;  /**< Doppler frequency for cross-correlation [hz] */
  float                    cn0;   /**< C/N0 level [dB/Hz] */
} tracking_channel_cc_entry_t;

/**
 * Data container for cross-correlation processing
 */
typedef struct {
  /** Entries with data for cross-correlation  */
  tracking_channel_cc_entry_t entries[NUM_TRACKER_CHANNELS];
} tracking_channel_cc_data_t;

/** \} */

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

/**
 * Common tracker configuration container.
 */
typedef struct {
  bool show_unconfirmed_trackers; /**< Flag to control reporting of unconfirmed
                                   *   tracker channels */
  float xcorr_delta;              /**< Frequency delta error for cross-correlation [hz] */
  float xcorr_cof;                /**< LPF cut-off frequency for cross-correlation filter [hz] */
  float xcorr_time;               /**< Cross-correlation time threshold [s] */
  lp1_filter_params_t xcorr_f_params; /**< Cross-correlation filter parameters */
} tp_tracker_config_t;

/**
 * Macro for default tracker parameters initialization
 */
#define TP_TRACKER_DEFAULT_CONFIG {false, 10.f, 0.1f, 1.f, {0, 0}}

/**
 * Registers common configuration parameters for a tracker section.
 *
 * \param[in]     section Configuration section name.
 * \param[in,out] config  Tracker parameters container.
 * \param[in]     proxy   Function pointer for handling parameter updates.
 *
 * \return None
 */
#define TP_TRACKER_REGISTER_CONFIG(section, config, proxy) \
  do {\
    SETTING((section), "show_unconfirmed", \
            (config).show_unconfirmed_trackers, TYPE_BOOL); \
    SETTING_NOTIFY((section), "xcorr_cof", \
                   (config).xcorr_cof, TYPE_FLOAT, (proxy)); \
    SETTING((section), "xcorr_delta", \
            (config).xcorr_delta, TYPE_FLOAT); \
    SETTING((section), "xcorr_time", \
            (config).xcorr_time, TYPE_FLOAT); \
  } while (0)


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

  float track_cn0_use_thres_dbhz;
  float track_cn0_drop_thres_dbhz;
  float track_cn0_ambiguity_thres_dbhz;
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

extern u16 max_pll_integration_time_ms;

tp_result_e tp_init(void);
tp_result_e tp_profile_init(const me_gnss_signal_t mesid,
                            tp_profile_t *profile,
                            const tp_report_t *data,
                            tp_config_t *config);
tp_result_e tp_profile_get_config(const me_gnss_signal_t mesid,
                                  tp_profile_t *profile,
                                  tp_config_t *config,
                                  bool commit);
tp_result_e tp_profile_get_cn0_params(const tp_profile_t *profile,
                                      tp_cn0_params_t *cn0_params);
bool        tp_profile_has_new_profile(const me_gnss_signal_t mesid,
                                       tp_profile_t *profile);
u8          tp_profile_get_next_loop_params_ms(const me_gnss_signal_t mesid,
                                               const tp_profile_t *profile);
void tp_profile_report_data(tracker_channel_t *tracker_channel,
                            tp_profile_t *profile,
                            const tp_report_t *data);

u8 tp_next_cycle_counter(tp_tm_e tracking_mode, u8 cycle_no);
u32 tp_get_cycle_flags(tracker_channel_t *tracker_channel, u8 cycle_no);
u32 tp_compute_cycle_parameters(tp_tm_e tracking_mode, u8 cycle_no);

u8 tp_get_cycle_count(tp_tm_e tracking_mode);
u8 tp_get_current_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no);
u32 tp_get_rollover_cycle_duration(tp_tm_e tracking_mode, u8 cycle_no);
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
void tp_tl_update(tp_tl_state_t *s, const tp_epl_corr_t *cs, bool costas);
float tp_tl_get_dll_error(tp_tl_state_t *s);
bool tp_tl_is_pll(const tp_tl_state_t *s);
bool tp_tl_is_fll(const tp_tl_state_t *s);
void tp_tl_fll_update_first(tp_tl_state_t *s, corr_t cs);
void tp_tl_fll_update_second(tp_tl_state_t *s, corr_t cs);
void tp_tl_fll_update(tp_tl_state_t *s);

/* Generic tracker functions */
void tp_tracker_register_parameters(const char *section,
                                    tp_tracker_config_t *config);

void tp_tracker_init(tracker_channel_t *tracker_channel,
                     const tp_tracker_config_t *config);
void tp_tracker_disable(tracker_channel_t *tracker_channel);
u32 tp_tracker_update(tracker_channel_t *tracker_channel,
                      const tp_tracker_config_t *config);
void tp_tracker_update_parameters(tracker_channel_t *tracker_channel,
                                  const tp_config_t *next_params,
                                  bool init);
void tp_tracker_update_correlators(tracker_channel_t *tracker_channel,
                                   u32 cycle_flags);
void tp_tracker_update_bsync(tracker_channel_t *tracker_channel,
                             u32 cycle_flags);
void tp_tracker_update_cn0(tracker_channel_t *tracker_channel,
                           u32 cycle_flags);
void tp_tracker_update_locks(tracker_channel_t *tracker_channel,
                             u32 cycle_flags);
void tp_tracker_update_fll(tracker_channel_t *tracker_channel, u32 cycle_flags);
void tp_tracker_update_pll_dll(tracker_channel_t *tracker_channel,
                               u32 cycle_flags);
void tp_tracker_update_alias(tracker_channel_t *tracker_channel,
                             u32 cycle_flags);
void tp_tracker_filter_doppler(tracker_channel_t *tracker_channel,
                               u32 cycle_flags,
                               const tp_tracker_config_t *config);
void tp_tracker_update_mode(tracker_channel_t *tracker_channel);
u32 tp_tracker_compute_rollover_count(tracker_channel_t *tracker_channel);
void tp_tracker_update_cycle_counter(tracker_channel_t *tracker_channel);
void tp_tracker_update_common_flags(tracker_channel_t *tracker_channel);
void set_xcorr_suspect_flag(tracker_channel_t *tracker_channel,
                            bool xcorr_suspect,
                            bool sensitivity_mode);

void track_setup(void);

void tracking_send_state(void);
void tracking_send_detailed_state(void);

double propagate_code_phase(const me_gnss_signal_t mesid,
                            double code_phase,
                            double carrier_freq,
                            u32 n_samples);

/* Update interface */
void tracking_channels_update(u64 channels_mask);
void tracking_channels_process(void);
void tracking_channels_missed_update_error(u32 channels_mask);

/* State management interface */
bool tracker_channel_available(tracker_channel_id_t id,
                               const me_gnss_signal_t mesid);
bool tracker_channel_init(tracker_channel_id_t id,
                          const me_gnss_signal_t mesid,
                          u16 glo_orbit_slot,
                          u32 ref_sample_count,
                          double code_phase,
                          float carrier_freq,
                          u32 chips_to_correlate,
                          float cn0_init);
bool tracker_channel_disable(tracker_channel_id_t id);

/* Tracking parameters interface. */

void tracking_channel_measurement_get(u64 ref_tc,
                                 const tracking_channel_info_t *info,
                                 const tracking_channel_freq_info_t *freq_info,
                                 const tracking_channel_time_info_t *time_info,
                                 const tracking_channel_misc_info_t *misc_info,
                                 channel_measurement_t *meas);

bool tracking_channel_calc_pseudorange(u64 ref_tc,
                                       const channel_measurement_t *meas,
                                       double *raw_pseudorange);

void tracking_channel_get_values(tracker_channel_id_t id,
                                 tracking_channel_info_t *info,
                                 tracking_channel_time_info_t *time_info,
                                 tracking_channel_freq_info_t *freq_info,
                                 tracking_channel_ctrl_info_t *ctrl_params,
                                 tracking_channel_misc_info_t *misc_params,
                                 bool reset_stats);
double tracking_channel_get_lock_time(const tracking_channel_time_info_t *time_info,
                                      const tracking_channel_misc_info_t *misc_info);
u16 tracking_channel_load_cc_data(tracking_channel_cc_data_t *cc_data);

void tracking_channel_set_carrier_phase_offset(const tracking_channel_info_t *info,
                                               double carrier_phase_offset);
void tracking_channel_carrier_phase_offsets_adjust(double dt);

tracker_channel_t *tracker_channel_get_by_mesid(const me_gnss_signal_t mesid);
void tracking_channel_drop_l2cl(const me_gnss_signal_t mesid);
void tracking_channel_drop_unhealthy_glo(const me_gnss_signal_t mesid);

bool handover_valid(double code_phase_chips, double max_chips);
bool sv_azel_degrees_set(gnss_signal_t sid, u16 azimuth,
                         s8 elevation, u64 timestamp);
u16 sv_azimuth_degrees_get(gnss_signal_t sid);
s8 sv_elevation_degrees_get(gnss_signal_t sid);

/* Decoder interface */
bool tracking_channel_nav_bit_get(tracker_channel_id_t id, s8 *soft_bit,
                                  bool *sensitivity_mode);
bool tracking_channel_health_sync(tracker_channel_id_t id, u8 health);
void tracking_channel_data_sync_init(nav_data_sync_t *data_sync);
void tracking_channel_gps_data_sync(tracker_channel_id_t id,
                                    nav_data_sync_t *from_decoder);
void tracking_channel_glo_data_sync(tracker_channel_id_t id,
                                    nav_data_sync_t *from_decoder);
void tracking_channel_set_prn_fail_flag(const me_gnss_signal_t mesid, bool val);
tracker_channel_t * tracker_channel_get(tracker_channel_id_t id);
void tracking_channel_set_xcorr_flag(const me_gnss_signal_t mesid);

void track_internal_setup(void);

tracker_interface_list_element_t ** tracker_interface_list_ptr_get(void);

void nav_bit_fifo_init(nav_bit_fifo_t *fifo);
bool nav_bit_fifo_full(nav_bit_fifo_t *fifo);
bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                        const nav_bit_fifo_element_t *element);
bool nav_bit_fifo_read(nav_bit_fifo_t *fifo, nav_bit_fifo_element_t *element);
void nav_data_sync_init(nav_data_sync_t *sync);
bool nav_data_sync_set(nav_data_sync_t *to_tracker,
                       const nav_data_sync_t *from_decoder);
bool nav_data_sync_get(nav_data_sync_t *to_tracker,
                       nav_data_sync_t *from_decoder);
s8 nav_bit_quantize(s32 bit_integrate);

u16 tracking_lock_counter_increment(const me_gnss_signal_t mesid);
u16 tracking_lock_counter_get(gnss_signal_t sid);

void tracker_interface_register(tracker_interface_list_element_t *element);

/* Tracker instance API functions. Must be called from within an
 * interface function. */
void tracker_correlations_read(u8 nap_channel,
                               corr_t *cs,
                               u32 *sample_count,
                               double *code_phase,
                               double *carrier_phase);
void tracker_retune(tracker_channel_t *tracker_channel, u32 chips_to_correlate);
s32 tracker_tow_update(tracker_channel_t *tracker_channel,
                       s32 current_TOW_ms,
                       u32 int_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow);
void tracker_bit_sync_set(tracker_channel_t *tracker_channel, s8 bit_phase_ref);
void tracker_bit_sync_update(tracker_channel_t *tracker_channel,
                             u32 int_ms,
                             s32 corr_prompt_real,
                             s32 corr_prompt_imag,
                             bool sensitivity_mode);
u8 tracker_bit_length_get(tracker_channel_t *tracker_channel);
bool tracker_bit_aligned(tracker_channel_t *tracker_channel);
bool tracker_has_bit_sync(tracker_channel_t *tracker_channel);
bool tracker_next_bit_aligned(tracker_channel_t *tracker_channel, u32 int_ms);
void tracker_ambiguity_unknown(tracker_channel_t *tracker_channel);
bool tracker_ambiguity_resolved(tracker_channel_t *tracker_channel);
void tracker_ambiguity_set(tracker_channel_t *tracker_channel, s8 polarity);
u16 tracker_glo_orbit_slot_get(tracker_channel_t *tracker_channel);
glo_health_t tracker_glo_sv_health_get(tracker_channel_t *tracker_channel);
void tracker_correlations_send(tracker_channel_t *tracker_channel, const corr_t *cs);
bool tracker_check_prn_fail_flag(tracker_channel_t *tracker_channel);
bool tracker_check_xcorr_flag(tracker_channel_t *tracker_channel);
update_count_t update_count_diff(const tracker_channel_t *
                                 tracker_channel,
                                 const update_count_t *val);

#endif
