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

#ifndef SWIFTNAV_TRACK_STATE_H
#define SWIFTNAV_TRACK_STATE_H

#include "tracker.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void track_setup(void);

tracker_t *tracker_get(u8 id);

/* State management interface */
bool tracker_available(const u8 id, const me_gnss_signal_t mesid);
bool tracker_init(const u8 id,
                  const me_gnss_signal_t mesid,
                  u16 glo_orbit_slot,
                  u64 ref_sample_count,
                  double code_phase,
                  float carrier_freq,
                  u32 chips_to_correlate,
                  float cn0_init);
void tracker_disable(const u8 id);

void tracker_get_state(u8 id,
                       tracker_info_t *info,
                       tracker_freq_info_t *freq_info);

/* Update interface */
void trackers_update(u32 channels_mask, const u8 start_chan);
void trackers_missed(u32 channels_mask, const u8 start_chan);
void stale_trackers_cleanup(void);

/* Send state */
void tracking_send_state(void);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_STATE_H  */
