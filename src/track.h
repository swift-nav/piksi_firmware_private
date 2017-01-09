/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
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

#include <ch.h>

#include "board/nap/track_channel.h"
#include <platform_signal.h>

/** \addtogroup tracking
 * \{ */

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
 *  Stable PLL remains within 15 degree from correct phase.
 *  360 * 0.05 = 18 degrees
*/
#define CARRIER_PHASE_TOLERANCE 0.05f
/** counter for half-cycle ambiguity resolution */
#define CARRIER_PHASE_AMBIGUITY_COUNTER 20
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

/** Bit mask of tracking channel flags */
typedef u32 tracking_channel_flags_t;

/**
 * Generic tracking channel information for external use.
 */
typedef struct {
  tracker_channel_id_t     id;           /**< Channel identifier */
  gnss_signal_t            sid;          /**< Signal identifier */
  tracking_channel_flags_t flags;        /**< Channel flags */
  s32                      tow_ms;       /**< ToW [ms] or TOW_UNKNOWN */
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

/** Parameters for half-cycle ambiguity resolution */
typedef struct {
  double cp;   /**< Current carrier phase [cycles]. */
  double cp_p; /**< Previous carrier phase [cycles]. */
  s32 TOW;     /**< Current Time of Week tag [ms]. */
  s32 TOW_p;   /**< Previous Time of Week tag [ms]. */
  u8 counter;  /**< Counter for matching carrier phases */
  s8 polarity; /**< Polarity of the matching carrier phases */
  bool synced; /**< Flag for indicating half-cycle ambiguity resolution */
  bool drop;   /**< Flag for dropping the tracking channel */
} cp_sync_t;

/** Tracking channel miscellaneous info */
typedef struct {
  double pseudorange;          /**< Pseudorange [m]  */
  double pseudorange_std;      /**< Pseudorange standard deviation [m] */
  struct {
    double value;              /**< Carrier phase offset value [cycles]. */
    u64 timestamp_ms;          /**< Carrier phase offset timestamp [ms] */
  } carrier_phase_offset;      /**< Carrier phase offset */
  cp_sync_t cp_sync;           /**< Half-cycle ambiguity resolution */
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
 * Input entry for cross-correlation processing
 *
 * \sa tracking_channel_cc_data_t
 */
typedef struct {
  tracker_channel_id_t     id;    /**< Tracking channel id */
  tracking_channel_flags_t flags; /**< Tracking channel flags */
  gnss_signal_t            sid;   /**< Tracked GNSS signal identifier */
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

double propagate_code_phase(double code_phase, double carrier_freq,
                            u32 n_samples, code_t code);

/* Update interface */
void tracking_channels_update(u32 channels_mask);
void tracking_channels_process(void);
void tracking_channels_missed_update_error(u32 channels_mask);

/* State management interface */
bool tracker_channel_available(tracker_channel_id_t id, gnss_signal_t sid);
bool tracker_channel_init(tracker_channel_id_t id, gnss_signal_t sid,
                          u32 ref_sample_count, double code_phase,
                          float carrier_freq, u32 chips_to_correlate,
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
void tracking_channel_cp_sync_update(gnss_signal_t sid, double cp, s32 TOW);
bool tracking_channel_load_data(gnss_signal_t sid,
                                float *own_cp, float *parent_cp,
                                float *own_cp_p, float *parent_cp_p,
                                s32 *own_TOW, s32 *parent_TOW,
                                s32 *own_TOW_p, s32 *parent_TOW_p,
                                u8 *count);
bool tracking_channel_find_matching_tow(gnss_signal_t sid,
                                        float own_cp, float parent_cp,
                                        float own_cp_p, float parent_cp_p,
                                        s32 own_TOW, s32 parent_TOW,
                                        s32 own_TOW_p, s32 parent_TOW_p,
                                        float *own_cp_test,
                                        float *parent_cp_test);
bool tracking_channel_compare_cp(gnss_signal_t sid,
                                 float own_cp_test, float parent_cp_test,
                                 s8 *polarity);
void tracking_channel_increment_cp_counter(gnss_signal_t sid,
                                           u8 count, s8 polarity);
void tracking_channel_drop_l2cl(gnss_signal_t sid);
s8 tracking_channel_read_ambiguity_status(gnss_signal_t sid);
void tracking_channel_cp_sync_match(gnss_signal_t sid, bool fll_mode);

bool sv_elevation_degrees_set(gnss_signal_t sid, s8 elevation, u64 timestamp);
s8 sv_elevation_degrees_get(gnss_signal_t sid);

/* Decoder interface */
bool tracking_channel_nav_bit_get(tracker_channel_id_t id, s8 *soft_bit,
                                  bool *sensitivity_mode);
bool tracking_channel_time_sync(tracker_channel_id_t id, s32 TOW_ms,
                                s8 bit_polarity);
void tracking_channel_set_prn_fail_flag(gnss_signal_t sid, bool val);
#endif
