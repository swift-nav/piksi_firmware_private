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
  float freq;             /**< Doppler frequency for cross-correlation [hz] */
  float cn0;              /**< C/N0 level [dB/Hz] */
} tracker_cc_entry_t;

/**
 * Data container for cross-correlation processing
 */
typedef struct {
  /** Entries with data for cross-correlation  */
  tracker_cc_entry_t entries[NUM_TRACKER_CHANNELS];
} tracker_cc_data_t;

/* Tracker utility interface. */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void tracker_measurement_get(u64 ref_tc,
                             const tracker_info_t *info,
                             const tracker_freq_info_t *freq_info,
                             const tracker_time_info_t *time_info,
                             const tracker_misc_info_t *misc_info,
                             channel_measurement_t *meas);

bool tracker_calc_pseudorange(u64 ref_tc,
                              const channel_measurement_t *meas,
                              double *raw_pseudorange);

void tracker_get_state(u8 id,
                       tracker_info_t *info,
                       tracker_time_info_t *time_info,
                       tracker_freq_info_t *freq_info,
                       tracker_misc_info_t *misc_params);
double tracker_get_lock_time(const tracker_time_info_t *time_info,
                             const tracker_misc_info_t *misc_info);
u16 tracker_load_cc_data(tracker_cc_data_t *cc_data);

void tracker_set_carrier_phase_offset(const tracker_info_t *info,
                                      s64 carrier_phase_offset);

tracker_t *tracker_get_by_mesid(const me_gnss_signal_t mesid);
void tracker_drop_unhealthy(const me_gnss_signal_t mesid);

bool handover_valid(double code_phase_chips, double max_chips);

double propagate_code_phase(const me_gnss_signal_t mesid,
                            double code_phase,
                            double carrier_freq,
                            u32 n_samples);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_PARAMS_H */
