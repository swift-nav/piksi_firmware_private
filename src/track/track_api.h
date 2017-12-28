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

#ifndef SWIFTNAV_TRACK_API_H
#define SWIFTNAV_TRACK_API_H

#include "track.h"

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
                       bool *decoded_tow,
                       bool *decoded_health);
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
void tracker_correlations_send(tracker_channel_t *tracker_channel,
                               const corr_t *cs);

update_count_t update_count_diff(const tracker_channel_t *tracker_channel,
                                 const update_count_t *val);
void update_bit_polarity_flags(tracker_channel_t *tracker_channel);
void tracker_cleanup(tracker_channel_t *tracker_channel);

void tracker_tow_cache(tracker_channel_t *tracker_channel);

void tracker_lock(tracker_channel_t *tracker_channel);
void tracker_unlock(tracker_channel_t *tracker_channel);

#endif /* SWIFTNAV_TRACK_API_H  */
