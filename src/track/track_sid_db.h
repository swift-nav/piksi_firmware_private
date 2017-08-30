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

#include <ephemeris.h>
#include <libswiftnav/signal.h>

#include "piksi_systime.h"

/** Minimum value for C/N0 to update ToW cache [dB/Hz] */
#define CN0_TOW_CACHE_THRESHOLD (30.f)

/**
 * ToW cache entry.
 */
typedef struct {
  s32 TOW_ms;          /**< ToW value [ms] */
  s32 TOW_residual_ns; /**< Residual to TOW_ms [ns] */
  piksi_systime_t sample_time; /**< ToW value time */
} tp_tow_entry_t;

/**
 * SV elevation cache entry.
 */
typedef struct {
  u16 azimuth_d;    /**< SV azimuth [degrees] */
  s8 elevation_d;   /**< SV elevation [degrees] */
  u64 timestamp_tk; /**< Azimuth and elevation evaluation time [ticks] */
} tp_azel_entry_t;

void track_sid_db_init(void);
void track_sid_db_clear_glo_tow(void);
s32 tp_tow_compute(s32 old_ToW_ms, u64 delta_us, u8 ms_align, double *error_ms);
bool tp_tow_is_sane(s32 tow_ms);
void track_sid_db_load_tow(const gnss_signal_t sid, tp_tow_entry_t *tow_entry);
void track_sid_db_update_tow(const gnss_signal_t sid,
                             const tp_tow_entry_t *tow_entry);
bool track_sid_db_load_elevation(const gnss_signal_t sid,
                                 tp_azel_entry_t *azel_entry);
bool track_sid_db_update_azel(const gnss_signal_t sid,
                              const tp_azel_entry_t *azel_entry);
bool track_sid_db_load_positions(const gnss_signal_t sid,
                                 xcorr_positions_t *position_entry);
bool track_sid_db_update_positions(const gnss_signal_t sid,
                                   const xcorr_positions_t *position_entry);

#endif /* SWIFTNAV_TRACK_SID_DB_H_ */
