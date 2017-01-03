/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_API_H
#define SWIFTNAV_TRACK_API_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "board/nap/nap_common.h"
#include "board/nap/track_channel.h"

/** \addtogroup track_api
 * \{ */

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
/** Sticky flags mask */
#define TRACK_CMN_FLAG_STICKY_MASK (TRACK_CMN_FLAG_HAD_PLOCK | \
                                    TRACK_CMN_FLAG_HAD_FLOCK | \
                                    TRACK_CMN_FLAG_TOW_DECODED | \
                                    TRACK_CMN_FLAG_TOW_PROPAGATED | \
                                    TRACK_CMN_FLAG_XCORR_CONFIRMED | \
                                    TRACK_CMN_FLAG_XCORR_SUSPECT | \
                                    TRACK_CMN_FLAG_XCORR_FILTER_ACTIVE)

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
  u32 sample_count;            /**< Total num samples channel has tracked for. */
  double code_phase_prompt;    /**< Prompt code phase in chips. */
  double code_phase_rate;      /**< Code phase rate in chips/s. */
  double carrier_phase;        /**< Carrier phase in cycles. */
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
  gnss_signal_t sid;            /**< Current signal being decoded. */
  u8 nap_channel;               /**< Associated NAP channel. */
  tracker_context_t *context;   /**< Current context for library functions. */
} tracker_channel_info_t;

/** Tracker interface function template. */
typedef void (tracker_interface_function_t)(
                 const tracker_channel_info_t *channel_info,
                 tracker_common_data_t *common_data,
                 tracker_data_t *tracker_data);

/** Interface to a tracker implementation. */
typedef struct {
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

/** \} */

void tracker_interface_register(tracker_interface_list_element_t *element);

/* Tracker instance API functions. Must be called from within an
 * interface function. */
void tracker_correlations_read(tracker_context_t *context, corr_t *cs,
                               u32 *sample_count, double *code_phase,
                               double *carrier_phase);
void tracker_retune(tracker_context_t *context, double carrier_freq,
                    double code_phase_rate, u32 chips_to_correlate);
s32 tracker_tow_update(tracker_context_t *context, s32 current_TOW_ms,
                       u32 int_ms, bool *decoded_tow);
void tracker_bit_sync_set(tracker_context_t *context, s8 bit_phase_ref);
void tracker_bit_sync_update(tracker_context_t *context, u32 int_ms,
                             s32 corr_prompt_real, bool sensitivity_mode);
u8 tracker_bit_length_get(tracker_context_t *context);
bool tracker_bit_aligned(tracker_context_t *context);
bool tracker_has_bit_sync(tracker_context_t *context);
bool tracker_next_bit_aligned(tracker_context_t *context, u32 int_ms);
void tracker_ambiguity_unknown(tracker_context_t *context);
bool tracker_ambiguity_status(tracker_context_t *context);
void tracker_correlations_send(tracker_context_t *context, const corr_t *cs);
bool tracker_check_prn_fail_flag(tracker_context_t *context);
#endif
