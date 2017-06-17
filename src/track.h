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

#include "board/nap/track_channel.h"
#include <platform_signal.h>
#include "board/nap/nap_common.h"

#define NAV_BIT_FIFO_SIZE 64 /**< Size of nav bit FIFO. Must be a power of 2 */

#define NAV_BIT_FIFO_INDEX_MASK ((NAV_BIT_FIFO_SIZE) - 1)
#define NAV_BIT_FIFO_INDEX_DIFF(write_index, read_index) \
          ((nav_bit_fifo_index_t)((write_index) - (read_index)))
#define NAV_BIT_FIFO_LENGTH(p_fifo) \
          (NAV_BIT_FIFO_INDEX_DIFF((p_fifo)->write_index, (p_fifo)->read_index))

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

typedef struct {
  /** FIFO for navigation message bits. */
  nav_bit_fifo_t nav_bit_fifo;
  /** Used to sync time decoded from navigation message
   * back to tracking channel. */
  nav_data_sync_t nav_data_sync;
  /** Time since last nav bit was appended to the nav bit FIFO. */
  u32 nav_bit_TOW_offset_ms;
  /** Bit sync state. */
  bit_sync_t bit_sync;
  /** GLO orbital slot. */
  u16 glo_orbit_slot;
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
  /** GLO SV health info */
  glo_health_t health;
} tracker_internal_data_t;

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

/** Tracker flag: tracker is in confirmed mode */
#define TRACK_CMN_FLAG_CONFIRMED   (1 << 0)
/** Tracker flag: tracker is using PLL (possibly with FLL) */
#define TRACK_CMN_FLAG_PLL_USE     (1 << 1)
/** Tracker flag: tracker is using FLL (possibly with PLL) */
#define TRACK_CMN_FLAG_FLL_USE     (1 << 2)
/** Tracker flag: tracker is using PLL and has pessimistic phase lock */
#define TRACK_CMN_FLAG_HAS_PLOCK   (1 << 3)
/** Tracker flag: tracker is using PLL and has optimistic phase lock */
#define TRACK_CMN_FLAG_HAS_OLOCK   (1 << 4)
/** Tracker flag: tracker is using FLL and has frequency lock */
#define TRACK_CMN_FLAG_HAS_FLOCK   (1 << 5)
/** Tracker flag: tracker has ever had PLL pessimistic lock */
#define TRACK_CMN_FLAG_HAD_PLOCK   (1 << 6)
/** Tracker flag: tracker has ever had FLL pessimistic lock */
#define TRACK_CMN_FLAG_HAD_FLOCK   (1 << 7)
/** Tracker flag: tracker has decoded TOW.
    Overrides #TRACK_CMN_FLAG_TOW_PROPAGATED. */
#define TRACK_CMN_FLAG_TOW_DECODED (1 << 8)
/** Tracker flag: tracker has propagated TOW */
#define TRACK_CMN_FLAG_TOW_PROPAGATED (1 << 9)
/** Tracker flag: tracker is a cross-correlate confirmed */
#define TRACK_CMN_FLAG_XCORR_CONFIRMED (1 << 10)
/** Tracker flag: tracker is a cross-correlate suspect */
#define TRACK_CMN_FLAG_XCORR_SUSPECT (1 << 11)
/** Tracker flag: tracker xcorr doppler filter is active */
#define TRACK_CMN_FLAG_XCORR_FILTER_ACTIVE (1 << 12)
/** Tracker flag: L2CL tracker has resolved half-cycle ambiguity */
#define TRACK_CMN_FLAG_L2CL_AMBIGUITY (1 << 13)
/** Tracker flag: tracker has decoded health information */
#define TRACK_CMN_FLAG_HEALTH_DECODED (1 << 14)
/** Tracker flag: Doppler outlier */
#define TRACK_CMN_FLAG_OUTLIER        (1 << 15)

/** Sticky flags mask */
#define TRACK_CMN_FLAG_STICKY_MASK (TRACK_CMN_FLAG_HAD_PLOCK | \
                                    TRACK_CMN_FLAG_HAD_FLOCK | \
                                    TRACK_CMN_FLAG_TOW_DECODED | \
                                    TRACK_CMN_FLAG_TOW_PROPAGATED | \
                                    TRACK_CMN_FLAG_XCORR_CONFIRMED | \
                                    TRACK_CMN_FLAG_XCORR_SUSPECT | \
                                    TRACK_CMN_FLAG_XCORR_FILTER_ACTIVE | \
                                    TRACK_CMN_FLAG_L2CL_AMBIGUITY | \
                                    TRACK_CMN_FLAG_OUTLIER | \
                                    TRACK_CMN_FLAG_HEALTH_DECODED)

/**
 * Common tracking feature flags.
 *
 * Flags is a combination of the following values:
 * - #TRACK_CMN_FLAG_CONFIRMED
 * - #TRACK_CMN_FLAG_PLL_USE
 * - #TRACK_CMN_FLAG_FLL_USE
 * - #TRACK_CMN_FLAG_HAS_PLOCK
 * - #TRACK_CMN_FLAG_HAS_OLOCK
 * - #TRACK_CMN_FLAG_HAS_FLOCK
 * - #TRACK_CMN_FLAG_HAD_PLOCK
 * - #TRACK_CMN_FLAG_HAD_FLOCK
 * - #TRACK_CMN_FLAG_TOW_DECODED
 * - #TRACK_CMN_FLAG_TOW_PROPAGATED
 * - #TRACK_CMN_FLAG_XCORR_CONFIRMED
 * - #TRACK_CMN_FLAG_XCORR_SUSPECT
 *
 * \sa tracker_common_data_t
 */
typedef u16 track_cmn_flags_t;

/** Parameters for half-cycle ambiguity resolution */
typedef struct {
  u8 counter;  /**< Counter for matching carrier phases */
  s8 polarity; /**< Polarity of the matching carrier phases */
  bool synced; /**< Flag for indicating half-cycle ambiguity resolution */
} cp_sync_t;

typedef struct {
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
  track_cmn_flags_t flags;     /**< Tracker flags */
  track_ctrl_params_t ctrl_params; /**< Controller parameters */
  float acceleration;          /**< Acceleration [g] */
  float xcorr_freq;            /**< Doppler for cross-correlation [hz] */
  u64 init_timestamp_ms;       /**< Tracking channel init timestamp [ms] */
  u64 update_timestamp_ms;     /**< Tracking channel last update
                                    timestamp [ms] */
  bool updated_once;           /**< Tracker was updated at least once flag. */
  cp_sync_t cp_sync;           /**< Half-cycle ambiguity resolution */
  glo_health_t health;         /**< GLO SV health info */
} tracker_common_data_t;

typedef void tracker_data_t;
typedef void tracker_context_t;

/** Instance of a tracker implementation. */
typedef struct {
  /** true if tracker is in use. */
  bool active;
  /** Pointer to implementation-specific data used by tracker instance. */
  tracker_data_t *data;
} tracker_t;

/** Info associated with a tracker channel. */
typedef struct {
  me_gnss_signal_t mesid;       /**< Current ME signal being decoded. */
  u8 nap_channel;               /**< Associated NAP channel. */
  tracker_context_t *context;   /**< Current context for library functions. */
} tracker_channel_info_t;

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

/** Tracking channel flag: tracker is active  */
#define TRACKING_CHANNEL_FLAG_ACTIVE         (1u << 0)
/** Tracking channel flag: tracker doesn't have an error */
#define TRACKING_CHANNEL_FLAG_NO_ERROR       (1u << 1)
/** Tracking channel flag: tracker has confirmed flag */
#define TRACKING_CHANNEL_FLAG_CONFIRMED      (1u << 2)
/** Tracking channel flag: tracker has usable C/N0 for a shorter period (SPP) */
#define TRACKING_CHANNEL_FLAG_CN0_SHORT      (1u << 3)
/** Tracking channel flag: tracker has usable C/N0 for a longer period (RTK) */
#define TRACKING_CHANNEL_FLAG_CN0_LONG       (1u << 4)
/** Tracking channel flag: tracker has confirmed PLL lock */
#define TRACKING_CHANNEL_FLAG_CONFIRMED_LOCK (1u << 5)
/** Tracking channel flag: tracker has not changed modes for some time */
#define TRACKING_CHANNEL_FLAG_STABLE         (1u << 6)
/** Tracking channel flag: tracker has ToW */
#define TRACKING_CHANNEL_FLAG_TOW            (1u << 7)
/** Tracking channel flag: tracker has bit polarity resolved */
#define TRACKING_CHANNEL_FLAG_BIT_POLARITY   (1u << 8)
/** Tracking channel flag: is PLL in use. Can also be combined with FLL flag */
#define TRACKING_CHANNEL_FLAG_PLL_USE        (1u << 9)
/** Tracking channel flag: is FLL in use */
#define TRACKING_CHANNEL_FLAG_FLL_USE        (1u << 10)
/** Tracking channel flag: is PLL optimistic lock present */
#define TRACKING_CHANNEL_FLAG_PLL_OLOCK      (1u << 11)
/** Tracking channel flag: is PLL pessimistic lock present */
#define TRACKING_CHANNEL_FLAG_PLL_PLOCK      (1u << 12)
/** Tracking channel flag: is FLL lock present */
#define TRACKING_CHANNEL_FLAG_FLL_LOCK       (1u << 13)
/** Tracking channel flag: tracker has bit sync resolved */
#define TRACKING_CHANNEL_FLAG_BIT_SYNC       (1u << 14)
/** Tracking channel flag: tracker has bit sync resolved */
#define TRACKING_CHANNEL_FLAG_HAD_LOCKS      (1u << 15)
/** Tracking channel flag: tracker has bit sync resolved */
#define TRACKING_CHANNEL_FLAG_BIT_INVERTED   (1u << 16)
/** Tracking channel flag: tracker has decoded TOW */
#define TRACKING_CHANNEL_FLAG_TOW_DECODED    (1u << 17)
/** Tracking channel flag: tracker has propagated TOW */
#define TRACKING_CHANNEL_FLAG_TOW_PROPAGATED (1u << 18)
/** Tracking channel flag: tracker has valid pseudorange */
#define TRACKING_CHANNEL_FLAG_PSEUDORANGE    (1u << 19)
/** Tracking channel flag: tracker has subframe sync (L1C/A) */
#define TRACKING_CHANNEL_FLAG_SUBFRAME_SYNC  (1u << 20)
/** Tracking channel flag: tracker has message sync (L2C) */
#define TRACKING_CHANNEL_FLAG_MESSAGE_SYNC    TRACKING_CHANNEL_FLAG_SUBFRAME_SYNC
/** Tracking channel flag: tracker has word sync (L1C/A) */
#define TRACKING_CHANNEL_FLAG_WORD_SYNC      (1u << 21)
/** Tracking channel flag: tracker is a cross-correlation confirmed */
#define TRACKING_CHANNEL_FLAG_XCORR_CONFIRMED (1u << 22)
/** Tracking channel flag: tracker is a cross-correlation suspect */
#define TRACKING_CHANNEL_FLAG_XCORR_SUSPECT (1u << 23)
/** Tracking channel flag: tracker xcorr doppler filter is active */
#define TRACKING_CHANNEL_FLAG_XCORR_FILTER_ACTIVE (1u << 24)
/** Tracking channel flag: L2CL tracker has resolved half-cycle ambiguity */
#define TRACKING_CHANNEL_FLAG_L2CL_AMBIGUITY_SOLVED (1u << 25)
/** Tracking channel flag: tracker has health info */
#define TRACKING_CHANNEL_FLAG_HEALTH_DECODED (1u << 26)
/** Tracking channel flag: healthy status -- 0 SV is unhealty, 1 SV is healthy */
#define TRACKING_CHANNEL_FLAG_HEALTHY    (1u << 27)
/** Tracking channel flag: error, Doppler went out of bounds */
#define TRACKING_CHANNEL_FLAG_OUTLIER    (1u << 28)

/** Maximum SV azimuth/elevation age in seconds: 1 minute is about 0.5 degrees */
#define MAX_AZ_EL_AGE_SEC 60

/** Bit mask of tracking channel flags */
typedef u32 tracking_channel_flags_t;

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
  tracking_channel_flags_t flags;        /**< Channel flags */
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
  /** Mutex used to permit atomic updates of public channel data. */
  mutex_t info_mutex;
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

struct tracker_interface;

/** Top-level generic tracker channel. */
typedef struct {
  /** State of this channel. */
  state_t state;
  /** Time at which the channel was disabled. */
  systime_t disable_time;
  /** Error flags. May be set at any time by the tracking thread. */
  volatile error_flag_t error_flags;
  /** Info associated with this channel. */
  tracker_channel_info_t info;

  me_gnss_signal_t mesid;       /**< Current ME signal being decoded. */
  u8 nap_channel;               /**< Associated NAP channel. */
  tracker_context_t *context;   /**< Current context for library functions. */

  /** Data common to all tracker implementations. RW from channel interface
   * functions. RO from functions in this module. */
  tracker_common_data_t common_data;
  /** Data used by the API for all tracker implementations. RW from API
   * functions called within channel interface functions. RO from functions
   * in this module. */
  tracker_internal_data_t internal_data;
  /** Mutex used to permit atomic reads of channel data. */
  mutex_t mutex;
  /** Associated tracker interface. */
  const struct tracker_interface *interface;
  /** Associated tracker instance. */
  tracker_t *tracker;
  /** Publicly accessible data */
  tracker_channel_pub_data_t pub_data;
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
  /** Array of tracker instances used by this interface. */
  tracker_t *trackers;
  /** Number of tracker instances in trackers array. */
  u8 num_trackers;
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
  tracking_channel_flags_t flags; /**< Tracking channel flags */
  me_gnss_signal_t         mesid; /**< Tracked GNSS ME signal identifier */
  float                    freq;  /**< Doppler frequency for cross-correlation [hz] */
  float                    cn0;   /**< C/N0 level [dB/Hz] */
  u16                      count; /**< Cross-correlation counter */
  bool                     wl;    /**< Is signal xcorr whitelisted? */
} tracking_channel_cc_entry_t;

/**
 * Data container for cross-correlation processing
 */
typedef struct {
  /** Entries with data for cross-correlation  */
  tracking_channel_cc_entry_t entries[NUM_TRACKER_CHANNELS];
} tracking_channel_cc_data_t;

/** \} */

void track_setup(void);

void tracking_send_state(void);
void tracking_send_detailed_state(void);

double propagate_code_phase(const me_gnss_signal_t mesid,
                            double code_phase,
                            double carrier_freq,
                            u32 n_samples);

/* Update interface */
void tracking_channels_update(u32 channels_mask);
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

void tracker_internal_context_resolve(tracker_context_t *tracker_context,
                                      const tracker_channel_info_t **channel_info,
                                      tracker_internal_data_t **internal_data);
void internal_data_init(tracker_internal_data_t *internal_data,
                        const me_gnss_signal_t mesid,
                        u16 glo_orbit_slot);

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
void tracker_correlations_read(tracker_context_t *context,
                               corr_t *cs,
                               u32 *sample_count,
                               double *code_phase,
                               double *carrier_phase);
void tracker_retune(tracker_context_t *context,
                    double doppler_freq_hz,
                    double code_phase_rate,
                    u32 chips_to_correlate);
s32 tracker_tow_update(tracker_context_t *context,
                       s32 current_TOW_ms,
                       u32 int_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow);
void tracker_bit_sync_set(tracker_context_t *context, s8 bit_phase_ref);
void tracker_bit_sync_update(tracker_context_t *context,
                             u32 int_ms,
                             s32 corr_prompt_real,
                             s32 corr_prompt_imag,
                             bool sensitivity_mode);
u8 tracker_bit_length_get(tracker_context_t *context);
bool tracker_bit_aligned(tracker_context_t *context);
bool tracker_has_bit_sync(tracker_context_t *context);
bool tracker_next_bit_aligned(tracker_context_t *context, u32 int_ms);
void tracker_ambiguity_unknown(tracker_context_t *context);
bool tracker_ambiguity_resolved(tracker_context_t *context);
void tracker_ambiguity_set(tracker_context_t *context, s8 polarity);
u16 tracker_glo_orbit_slot_get(tracker_context_t *context);
glo_health_t tracker_glo_sv_health_get(tracker_context_t *context);
void tracker_correlations_send(tracker_context_t *context, const corr_t *cs);
bool tracker_check_prn_fail_flag(tracker_context_t *context);
bool tracker_check_xcorr_flag(tracker_context_t *context);

#endif
