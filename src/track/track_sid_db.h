/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_SID_DB_H_
#define SWIFTNAV_TRACK_SID_DB_H_

#include <libswiftnav/signal.h>

#include "ephemeris/ephemeris.h"
#include "tracker.h"

/** Minimum value for C/N0 to update ToW cache [dB/Hz] */
#define CN0_TOW_CACHE_THRESHOLD (30.f)

/** Maximum SV azimuth/elevation age in [s]: 1 minute is about 0.5 degrees */
#define MAX_AZ_EL_AGE_SEC 60

/**
 * ToW cache entry.
 */
typedef struct {
  s32 TOW_ms;          /**< ToW value [ms] */
  s32 TOW_residual_ns; /**< Residual to TOW_ms [ns] */
  u64 sample_time_tk;  /**< ToW value time [ticks] */
} tp_tow_entry_t;

/**
 * SV elevation cache entry.
 */
typedef struct {
  double azimuth_d;   /**< SV azimuth [degrees] */
  double elevation_d; /**< SV elevation [degrees] */
  u64 timestamp_tk;   /**< Azimuth and elevation evaluation time [ticks] */
} tp_azel_entry_t;

void track_sid_db_init(void);
void track_sid_db_clear_glo_tow(void);
s32 tp_tow_compute(s32 old_TOW_ms, u64 delta_tk, u8 ms_align, double *error_ms);
bool tp_tow_is_sane(s32 tow_ms);
void track_sid_db_load_tow(const gnss_signal_t sid, tp_tow_entry_t *tow_entry);
void track_sid_db_update_tow(const gnss_signal_t sid,
                             const tp_tow_entry_t *tow_entry);
void track_sid_db_azel_degrees_set(const gnss_signal_t sid,
                                   double azimuth,
                                   double elevation,
                                   u64 timestamp);
bool track_sid_db_azimuth_degrees_get(const gnss_signal_t sid, double *azimuth);
bool track_sid_db_elevation_degrees_get(const gnss_signal_t sid,
                                        double *elevation);

void update_tow_in_sid_db(tracker_t *tracker);
void propagate_tow_from_sid_db(tracker_t *tracker);
void clear_tow_in_sid_db(const gnss_signal_t sid);

#endif /* SWIFTNAV_TRACK_SID_DB_H_ */
