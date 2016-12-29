/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swift-nav.com>
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
#include <ephemeris.h>

/** Minimum value for C/N0 to update ToW cache [dB/Hz] */
#define CN0_TOW_CACHE_THRESHOLD (30.f)

/**
 * ToW cache entry.
 */
typedef struct {
  s32 TOW_ms;          /**< ToW value [ms] */
  u64 sample_time_tk;  /**< ToW value time [ticks] */
} tp_tow_entry_t;

/**
 * SV elevation cache entry.
 */
typedef struct {
  s8  elevation_d;   /**< SV elevation [degrees] */
  u64 timestamp_tk;  /**< Elevation value time [ticks] */
} tp_elevation_entry_t;

void track_sid_db_init(void);
s32 tp_tow_compute(s32 old_ToW_ms, u64 delta_tk, u8 ms_align, double *error_ms);
bool tp_tow_is_sane(s32 tow_ms);
bool track_sid_db_load_tow(gnss_signal_t sid, tp_tow_entry_t *tow_entry);
bool track_sid_db_update_tow(gnss_signal_t sid, const tp_tow_entry_t *tow_entry);
bool track_sid_db_load_elevation(gnss_signal_t sid,
                                 tp_elevation_entry_t *elevation_entry);
bool track_sid_db_update_elevation(gnss_signal_t sid,
                                   const tp_elevation_entry_t *elevation_entry);
bool track_sid_db_load_positions(gnss_signal_t sid,
                                 xcorr_positions_t *position_entry);
bool track_sid_db_update_positions(gnss_signal_t sid,
                                   const xcorr_positions_t *position_entry);

#endif /* SWIFTNAV_TRACK_SID_DB_H_ */
