/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef TRACK_FLAGS_H
#define TRACK_FLAGS_H

#include <swiftnav/signal.h>

#include "tracker.h"

/** Tracker flag: tracker is in confirmed mode */
#define TRACKER_FLAG_ACTIVE (1 << 0)
#define TRACKER_FLAG_CONFIRMED (1 << 1)
/** Tracker flag: tracker is using PLL (possibly with FLL) */
#define TRACKER_FLAG_PLL_USE (1 << 2)
/** Tracker flag: tracker is using FLL (possibly with PLL) */
#define TRACKER_FLAG_FLL_USE (1 << 3)
/** Tracker flag: tracker is using PLL and has pessimistic phase lock */
#define TRACKER_FLAG_HAS_PLOCK (1 << 4)
/** Tracker flag: tracker is using FLL and has frequency lock */
#define TRACKER_FLAG_HAS_FLOCK (1 << 5)
/** Tracker flag: tracker has ever had PLL pessimistic lock */
#define TRACKER_FLAG_HAD_PLOCK (1 << 6)
/** Tracker flag: tracker has ever had FLL pessimistic lock */
#define TRACKER_FLAG_HAD_FLOCK (1 << 7)
/** Tracker flag: tracker has decoded TOW. */
#define TRACKER_FLAG_TOW_DECODED (1 << 8)
#define TRACKER_FLAG_TOW_VALID (1 << 9)

#define TRACKER_FLAG_USE_ALIAS_DETECTION (1 << 10)

/** Tracker flag: tracker is a cross-correlate suspect */
#define TRACKER_FLAG_XCORR_SUSPECT (1 << 11)
/** Tracker flag: tracker xcorr doppler filter is active */
#define TRACKER_FLAG_XCORR_FILTER_ACTIVE (1 << 12)
/** Tracker flag: GLO string synchronization achieved */
#define TRACKER_FLAG_GLO_STRING_SYNC (1 << 13)

#define TRACKER_FLAG_CN0_FILTER_INIT (1 << 14)

/** Tracker flag: signal is unhealthy. */
#define TRACKER_FLAG_UNHEALTHY (1 << 15)

#define TRACKER_FLAG_BIT_POLARITY_KNOWN (1 << 18)
#define TRACKER_FLAG_BIT_INVERTED (1 << 19)
#define TRACKER_FLAG_BIT_SYNC (1 << 20)

#define TRACKER_FLAG_CN0_USABLE (1 << 22)
#define TRACKER_FLAG_NAV_SUITABLE (1 << 23)
#define TRACKER_FLAG_HAS_EPHE (1 << 24)
#define TRACKER_FLAG_ELEVATION (1 << 25)
#define TRACKER_FLAG_CARRIER_PHASE_OFFSET (1 << 26)
#define TRACKER_FLAG_MASKED (1 << 27)

#define TRACKER_FLAG_RECOVERY_MODE (1 << 28)
#define TRACKER_FLAG_SENSITIVITY_MODE (1 << 29)
#define TRACKER_FLAG_DROP_CHANNEL (1 << 31)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void tracker_flag_drop(tracker_t *tracker, ch_drop_reason_t reason);
void tracker_set_prn_fail_flag(const me_gnss_signal_t mesid, bool val);
void tracker_set_sbas_provider_change_flag(void);
void tracker_set_leap_second_flag(void);
void tracker_reset_all_phase_offsets(void);
void tracker_set_raim_flag(const gnss_signal_t sid);
void tracker_set_xcorr_flag(const me_gnss_signal_t mesid);
void tracker_set_xcorr_suspect_flag(tracker_t *tracker,
                                    bool xcorr_suspect,
                                    bool sensitivity_mode);

void tracker_update_bit_polarity_flags(tracker_t *tracker);

bool tracker_get_prn_fail_flag(tracker_t *tracker);
bool tracker_get_xcorr_flag(tracker_t *tracker);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif
