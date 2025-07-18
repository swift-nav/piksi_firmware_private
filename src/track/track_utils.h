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

#ifndef SWIFTNAV_TRACK_PARAMS_H
#define SWIFTNAV_TRACK_PARAMS_H

#include <swiftnav/ch_meas.h>

#include "platform_signal.h"
#include "tracker.h"

/**
 * Input entry for cross-correlation processing
 *
 * \sa tracker_cc_data_t
 */
typedef struct {
  u8 id;                  /**< Tracking channel id */
  u32 flags;              /**< Tracker flags TRACKER_FLAG_... */
  me_gnss_signal_t mesid; /**< Tracked GNSS ME signal identifier */
  float freq_hz;          /**< Doppler frequency for cross-correlation [hz] */
  float cn0;              /**< C/N0 level [dB/Hz] */
} tracker_cc_entry_t;

/**
 * Data container for cross-correlation processing
 */
typedef struct {
  /** Entries with data for cross-correlation  */
  tracker_cc_entry_t entries[ME_CHANNELS];
} tracker_cc_data_t;

/* Tracker utility interface. */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void tracker_measurement_get(u64 ref_tc,
                             const tracker_info_t *info,
                             const tracker_freq_info_t *freq_info,
                             channel_measurement_t *meas);

bool tracker_calc_pseudorange(u64 ref_tc,
                              const channel_measurement_t *meas,
                              double *raw_pseudorange);

double tracker_get_lock_time(const tracker_freq_info_t *freq_info);
u16 tracker_load_cc_data(tracker_cc_data_t *cc_data);

void tracker_set_carrier_phase_offset(const tracker_info_t *info,
                                      s32 carrier_phase_offset);
void tracker_adjust_all_phase_offsets(double offset_s);

tracker_t *tracker_get_by_mesid(me_gnss_signal_t mesid);
void tracker_drop_unhealthy(me_gnss_signal_t mesid);

bool handover_valid(double code_phase_chips, double max_chips);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_PARAMS_H */
