/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_MANAGE_H
#define SWIFTNAV_MANAGE_H

#include <ch.h>
#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/track.h>
#include "board/acq.h"

/** \addtogroup manage
 * \{ */

/** C/N0 threshold long interval [ms] */
#define TRACK_CN0_THRES_COUNT_LONG 2000

/** C/N0 threshold short interval [ms] */
#define TRACK_CN0_THRES_COUNT_SHORT 100

/** How many ms to allow tracking channel to converge after
    initialization before we consider dropping it.
    Applied to all signals other than GPS L2CL */
#define TRACK_INIT_T 2500

/** How many ms to allow L2CL tracking channel to converge after
    initialization before we consider dropping it */
#define TRACK_INIT_T_L2CL 500

/** If a channel is dropped but was running successfully for at least
    this long, mark it for prioritized reacquisition. */
#define TRACK_REACQ_T 5000

/** If C/N0 is below track_cn0_threshold for >= TRACK_DROP_CN0_T ms,
    drop the channel. */
#define TRACK_DROP_CN0_T 500

/** If optimistic phase lock detector shows "unlocked" for >=
    TRACK_DROP_UNLOCKED_T ms, drop the channel. */
#define TRACK_DROP_UNLOCKED_T 1500

/** If pessimistic phase lock detector shows "locked" for >=
    TRACK_USE_LOCKED_T ms, use the channel. */
#define TRACK_USE_LOCKED_T 100

/** How many milliseconds to wait for the tracking loops to
 * stabilize after any mode change before using obs. */
#define TRACK_STABILIZATION_T 1000

#define ACQ_FULL_CF_STEP  acq_bin_width()

#define MANAGE_NO_CHANNELS_FREE 255

#define MANAGE_ACQ_THREAD_PRIORITY (NORMALPRIO-4)
#define MANAGE_ACQ_THREAD_STACK    16384

#define MANAGE_TRACK_THREAD_PRIORITY (NORMALPRIO-2)
#define MANAGE_TRACK_THREAD_STACK   16384

/* Tracking channel state flags */
/** Tracking channel flag: tracker is active  */
#define MANAGE_TRACK_FLAG_ACTIVE         (1u << 0)
/** Tracking channel flag: tracker doesn't have an error */
#define MANAGE_TRACK_FLAG_NO_ERROR       (1u << 1)
/** Tracking channel flag: tracker has confirmed flag */
#define MANAGE_TRACK_FLAG_CONFIRMED      (1u << 2)
/** Tracking channel flag: tracker has usable C/N0 for a shorter period (SPP) */
#define MANAGE_TRACK_FLAG_CN0_SHORT      (1u << 3)
/** Tracking channel flag: tracker has usable C/N0 for a longer period (RTK) */
#define MANAGE_TRACK_FLAG_CN0_LONG       (1u << 4)
/** Tracking channel flag: tracker has usable elevation */
#define MANAGE_TRACK_FLAG_ELEVATION      (1u << 5)
/** Tracking channel flag: tracker has confirmed PLL lock */
#define MANAGE_TRACK_FLAG_CONFIRMED_LOCK (1u << 6)
/** Tracking channel flag: tracker has not changed modes for some time */
#define MANAGE_TRACK_FLAG_STABLE         (1u << 8)
/** Tracking channel flag: tracker has ToW */
#define MANAGE_TRACK_FLAG_TOW            (1u << 9)
/** Tracking channel flag: tracker has bit polarity resolved */
#define MANAGE_TRACK_FLAG_BIT_POLARITY   (1u << 10)
/** Tracking channel flag: tracked SV has decoded ephemeris */
#define MANAGE_TRACK_FLAG_HAS_EPHE       (1u << 11)
/** Tracking channel flag: tracked SV has healthy status */
#define MANAGE_TRACK_FLAG_HEALTHY        (1u << 12)
/** Tracking channel flag: tracked SV is suitable for navigation */
#define MANAGE_TRACK_FLAG_NAV_SUITABLE   (1u << 13)
/** Tracking channel flag: is PLL in use. Can also be combined with FLL flag */
#define MANAGE_TRACK_FLAG_PLL_USE        (1u << 14)
/** Tracking channel flag: is FLL in use */
#define MANAGE_TRACK_FLAG_FLL_USE        (1u << 15)
/** Tracking channel flag: is PLL optimistic lock present */
#define MANAGE_TRACK_FLAG_PLL_OLOCK      (1u << 16)
/** Tracking channel flag: is PLL pessimistic lock present */
#define MANAGE_TRACK_FLAG_PLL_PLOCK      (1u << 17)
/** Tracking channel flag: is FLL lock present */
#define MANAGE_TRACK_FLAG_FLL_LOCK       (1u << 18)
/** Tracking channel flag: carrier phase offset was estimated */
#define MANAGE_TRACK_FLAG_CARRIER_PHASE_OFFSET (1u << 19)
/** Tracking channel flag: is cross-correlated */
#define MANAGE_TRACK_FLAG_XCORR_CONFIRMED (1u << 20)
/** Tracking channel flag: is cross-correlation suspect */
#define MANAGE_TRACK_FLAG_XCORR_SUSPECT (1u << 21)
/** Tracking channel flag: L2CL has half-cycle ambiguity resolved */
#define MANAGE_TRACK_FLAG_L2CL_AMBIGUITY (1u << 22)

/* Tracking channel state masks */

/** Tracking channel mask for use with reporting */
#define MANAGE_TRACK_STATUS_FLAGS \
  (MANAGE_TRACK_FLAG_ACTIVE | MANAGE_TRACK_FLAG_NO_ERROR | \
   MANAGE_TRACK_FLAG_CONFIRMED | MANAGE_TRACK_FLAG_CN0_SHORT | \
   MANAGE_TRACK_FLAG_ELEVATION | MANAGE_TRACK_FLAG_TOW | \
   MANAGE_TRACK_FLAG_HAS_EPHE | MANAGE_TRACK_FLAG_HEALTHY | \
   MANAGE_TRACK_FLAG_NAV_SUITABLE)

/** Tracking channel flags mask. */
typedef u32 manage_track_flags_t;

typedef struct {
  me_gnss_signal_t mesid; /**< ME signal identifier. */
  u16 glo_slot_id;        /**< GLO orbital slot. */
  u32 sample_count;       /**< Reference NAP sample count. */
  float carrier_freq;     /**< Carrier frequency Doppler (Hz). */
  double code_phase;      /**< Code phase (chips). */
  u32 chips_to_correlate; /**< Chips to integrate over. */
  float cn0_init;         /**< C/N0 estimate (dBHz). */
  s8 elevation;           /**< Elevation (deg). */
} tracking_startup_params_t;

/**
 * Tracking controller parameters.
 */
typedef struct {
  float fll_bw;  /**< FLL controller NBW [Hz].
                      Single sided noise bandwidth in case of
                      FLL and FLL-assisted PLL tracking */
  float pll_bw;  /**< PLL controller noise bandwidth [Hz].
                      Single sided noise bandwidth in case of
                      PLL and FLL-assisted PLL tracking */
  float dll_bw;  /**< DLL controller noise bandwidth [Hz]. */
  u8    int_ms;  /**< PLL/FLL controller integration time [ms] */
} tracking_ctrl_params_t;

/** \} */

void manage_acq_setup(void);

void manage_set_obs_hint(gnss_signal_t sid);

void manage_track_setup(void);

float get_solution_elevation_mask(void);
void acq_result_send(const me_gnss_signal_t mesid, float cn0, float cp, float cf);

manage_track_flags_t get_tracking_channel_flags(u8 i);
manage_track_flags_t get_tracking_channel_meas(u8 i,
                                               u64 ref_tc,
                                               channel_measurement_t *meas,
                                               ephemeris_t *ephe);
void get_tracking_channel_ctrl_params(u8 i, tracking_ctrl_params_t *pparams);
manage_track_flags_t get_tracking_channel_sid_flags(const me_gnss_signal_t mesid,
                                                    s32 tow_ms,
                                                    const ephemeris_t *pephe);
u8 tracking_channels_ready(manage_track_flags_t required_flags);

bool tracking_startup_ready(const me_gnss_signal_t mesid);
bool tracking_is_running(const me_gnss_signal_t mesid);
u8 tracking_startup_request(const tracking_startup_params_t *startup_params);

bool l1ca_l2cm_handover_reserve(u8 sat);
void l1ca_l2cm_handover_release(u8 sat);
bool mesid_is_tracked(const me_gnss_signal_t mesid);
#endif
