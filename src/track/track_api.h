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

#include "tracker.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void track_internal_setup(void);

/* Tracker instance API functions. Must be called from within an
 * interface function. */
void tracker_correlations_read(u8 nap_channel,
                               corr_t *cs,
                               u32 *sample_count,
                               double *code_phase,
                               double *carrier_phase);
void tracker_retune(tracker_t *tracker, u32 chips_to_correlate);
s32 tracker_tow_update(tracker_t *tracker,
                       s32 current_TOW_ms,
                       u32 int_ms,
                       s32 *TOW_residual_ns,
                       bool *decoded_tow);
void tracker_bit_sync_set(tracker_t *tracker, s8 bit_phase_ref);
void tracker_bit_sync_update(tracker_t *tracker,
                             u32 int_ms,
                             bool sensitivity_mode);

bool nap_sc_wipeoff(const tracker_t *tracker);
u8 tracker_bit_length_get(tracker_t *tracker);
bool tracker_bit_aligned(tracker_t *tracker);
bool tracker_has_bit_sync(const tracker_t *tracker);
bool tracker_has_all_locks(const tracker_t *tracker);
bool tracker_next_bit_aligned(tracker_t *tracker, u32 int_ms);
void tracker_ambiguity_unknown(tracker_t *tracker);
bool tracker_ambiguity_resolved(tracker_t *tracker);
void tracker_ambiguity_set(tracker_t *tracker, s8 polarity);
u16 tracker_glo_orbit_slot_get(tracker_t *tracker);
void tracker_correlations_send(tracker_t *tracker, const corr_t *cs);

void tracker_cleanup(tracker_t *tracker);

void tracker_tow_cache(tracker_t *tracker);

void tracker_lock(tracker_t *tracker);
void tracker_unlock(tracker_t *tracker);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_API_H  */
