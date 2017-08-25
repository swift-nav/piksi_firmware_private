/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_CHANNEL_H
#define SWIFTNAV_TRACK_CHANNEL_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

#include "nap_common.h"

/** \addtogroup track_channel
 * \{ */

extern u8 nap_track_n_channels;

/** \} */

void nap_track_init(u8 channel,
                    const me_gnss_signal_t mesid,
                    u64 ref_timing_count,
                    float doppler_freq_hz,
                    double code_phase,
                    u32 chips_to_correlate);
void nap_track_update(u8 channel,
                      double doppler_freq_hz,
                      double code_phase_rate,
                      u32 chips_to_correlate,
                      u8 corr_spacing);

void nap_track_read_results(swiftnap_tracking_t *res, u8 nap_channel);

void nap_track_parse_results(const swiftnap_tracking_t *nap_results,
                             u8 channel,
                             u32 *count_snapshot,
                             corr_t corrs[],
                             double *code_phase_prompt,
                             double *carrier_phase);

bool nap_track_supports(u8 channel, const me_gnss_signal_t mesid);

void nap_track_enable(u8 channel);
void nap_track_disable(u8 channel);
void nap_scan_channels(void);

#endif /* SWIFTNAV_TRACK_CHANNEL_H */
