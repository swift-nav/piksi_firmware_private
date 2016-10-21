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
#include "track_api.h"

/** \addtogroup tracking
 * \{ */

#define TRACKING_ELEVATION_UNKNOWN 100 /* Ensure it will be above elev. mask */

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

/** Bit mask of tracking channel flags */
typedef u16 tracking_channel_flags_t;

/** \} */

void track_setup(void);

void tracking_send_state(void);

double propagate_code_phase(double code_phase, double carrier_freq,
                            u32 n_samples, code_t code);

/* Update interface */
void tracking_channels_update(u32 channels_mask);
void tracking_channels_process(void);
void tracking_channels_missed_update_error(u32 channels_mask);

/* State management interface */
bool tracker_channel_available(tracker_channel_id_t id, gnss_signal_t sid);
bool tracker_channel_init(tracker_channel_id_t id, gnss_signal_t sid,
                          u32 ref_sample_count, float code_phase,
                          float carrier_freq, u32 chips_to_correlate,
                          float cn0_init);
bool tracker_channel_disable(tracker_channel_id_t id);

/* Tracking parameters interface.
 * Lock should be acquired for atomicity. */
void tracking_channel_lock(tracker_channel_id_t id);
void tracking_channel_unlock(tracker_channel_id_t id);

tracking_channel_flags_t tracking_channel_get_flags(tracker_channel_id_t id);
bool tracking_channel_running(tracker_channel_id_t id);
bool tracking_channel_error(tracker_channel_id_t id);
float tracking_channel_cn0_get(tracker_channel_id_t id);
u32 tracking_channel_running_time_ms_get(tracker_channel_id_t id);
u32 tracking_channel_cn0_useable_ms_get(tracker_channel_id_t id);
u32 tracking_channel_cn0_drop_ms_get(tracker_channel_id_t id);
u32 tracking_channel_ld_pess_locked_ms_get(tracker_channel_id_t id);
u32 tracking_channel_ld_pess_unlocked_ms_get(tracker_channel_id_t id);
u32 tracking_channel_last_mode_change_ms_get(tracker_channel_id_t id);
gnss_signal_t tracking_channel_sid_get(tracker_channel_id_t id);
double tracking_channel_carrier_freq_get(tracker_channel_id_t id);
s32 tracking_channel_tow_ms_get(tracker_channel_id_t id);
bool tracking_channel_bit_sync_resolved(tracker_channel_id_t id);
bool tracking_channel_bit_polarity_resolved(tracker_channel_id_t id);
void tracking_channel_measurement_get(tracker_channel_id_t id, u64 ref_tc,
                                      channel_measurement_t *meas);
void tracking_channel_carrier_phase_offsets_adjust(double dt);

bool tracking_channel_elevation_degrees_set(gnss_signal_t sid, s8 elevation);
s8 tracking_channel_elevation_degrees_get(gnss_signal_t sid);

/* Decoder interface */
bool tracking_channel_nav_bit_get(tracker_channel_id_t id, s8 *soft_bit);
bool tracking_channel_time_sync(tracker_channel_id_t id, s32 TOW_ms,
                                s8 bit_polarity);

#endif
