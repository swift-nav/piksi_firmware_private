/*
 * Copyright (C) 2011-2014,2016 Swift Navigation Inc.
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
#include "board/acq.h"

/** \addtogroup manage
 * \{ */

#define ACQ_THRESHOLD 37.0
#define ACQ_RETRY_THRESHOLD 38.0

/** C/N0 threshold long interval [ms] */
#define TRACK_CN0_THRES_COUNT_LONG 2000

/** C/N0 threshold short interval [ms] */
#define TRACK_CN0_THRES_COUNT_SHORT 100

/** How many ms to allow tracking channel to converge after
    initialization before we consider dropping it */
#define TRACK_INIT_T 2500

/** If a channel is dropped but was running successfully for at least
    this long, mark it for prioritized reacquisition. */
#define TRACK_REACQ_T 5000

/** If C/N0 is below track_cn0_threshold for >= TRACK_DROP_CN0_T ms,
    drop the channel. */
#define TRACK_DROP_CN0_T 5000

/** If optimistic phase lock detector shows "unlocked" for >=
    TRACK_DROP_UNLOCKED_T ms, drop the channel. */
#define TRACK_DROP_UNLOCKED_T 5000

/** If pessimistic phase lock detector shows "locked" for >=
    TRACK_USE_LOCKED_T ms, use the channel. */
#define TRACK_USE_LOCKED_T 100

/** How many milliseconds to wait for the tracking loops to
 * stabilize after any mode change before using obs. */
#define TRACK_STABILIZATION_T 1000

#define ACQ_FULL_CF_MIN  -8500
#define ACQ_FULL_CF_MAX   8500
#define ACQ_FULL_CF_STEP  acq_bin_width()

#define MANAGE_NO_CHANNELS_FREE 255

#define MANAGE_ACQ_THREAD_PRIORITY (NORMALPRIO-3)
#define MANAGE_ACQ_THREAD_STACK    2314

#define MANAGE_TRACK_THREAD_PRIORITY (NORMALPRIO-2)
#define MANAGE_TRACK_THREAD_STACK   1400

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


/* Tracking channel state masks */
/** Legacy tracking channel mask..
 *
 * Legacy flags include the following conditions:
 * - Tracker is active
 * - There is no error
 * - Tracker is in confirmed state
 * - C/N0 is above threshold for a shorter period of time
 * - SV elevation is above threshold
 * - ToW for SV is known
 * - Ephemeris is present in the database
 * - SV health status is OK
 * - SV navigation health status is OK
 * - Tracker is PLL mode, and has pessimistic lock for some time
 */
#define MANAGE_TRACK_LEGACY_USE_FLAGS \
  (MANAGE_TRACK_FLAG_ACTIVE | MANAGE_TRACK_FLAG_NO_ERROR | \
   MANAGE_TRACK_FLAG_CONFIRMED | MANAGE_TRACK_FLAG_CN0_SHORT | \
   MANAGE_TRACK_FLAG_ELEVATION | MANAGE_TRACK_FLAG_TOW | \
   MANAGE_TRACK_FLAG_PLL_USE | MANAGE_TRACK_FLAG_PLL_PLOCK | \
   MANAGE_TRACK_FLAG_CONFIRMED_LOCK | MANAGE_TRACK_FLAG_HAS_EPHE | \
   MANAGE_TRACK_FLAG_HEALTHY | MANAGE_TRACK_FLAG_NAV_SUITABLE)

/** Tracking channel mask for use with reporting */
#define MANAGE_TRACK_STATUS_FLAGS MANAGE_TRACK_LEGACY_USE_FLAGS

/** Tracking channel flags mask. */
typedef u32 manage_track_flags_t;

typedef struct {
  gnss_signal_t sid;      /**< Signal identifier. */
  u32 sample_count;       /**< Reference NAP sample count. */
  float carrier_freq;     /**< Carrier frequency Doppler (Hz). */
  float code_phase;       /**< Code phase (chips). */
  u32 chips_to_correlate; /**< Chips to integrate over. */
  float cn0_init;         /**< C/N0 estimate (dBHz). */
  s8 elevation;           /**< Elevation (deg). */
} tracking_startup_params_t;

/** \} */

void manage_acq_setup(void);

void manage_set_obs_hint(gnss_signal_t sid);

void manage_track_setup(void);

manage_track_flags_t get_tracking_channel_flags(u8 i);
manage_track_flags_t get_tracking_channel_sid_flags(gnss_signal_t sid,
                                                    s32 tow_ms,
                                                    ephemeris_t *pephe);
s8 use_tracking_channel(u8 i);
bool tracking_channel_is_usable(u8 i, manage_track_flags_t required_flags);
u8 tracking_channels_ready(manage_track_flags_t required_flags);

bool tracking_startup_ready(gnss_signal_t sid);
bool tracking_is_running(gnss_signal_t sid);
u8 tracking_startup_request(const tracking_startup_params_t *startup_params);

bool l1ca_l2cm_handover_reserve(u8 sat);
void l1ca_l2cm_handover_release(u8 sat);

#endif
