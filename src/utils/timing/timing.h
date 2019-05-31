/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TIME_H
#define SWIFTNAV_TIME_H

#include <swiftnav/common.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>
#include <swiftnav/single_epoch_solver.h>

#include "nap/nap_constants.h"
#include "signal_db/signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define RX_DT_NOMINAL (1.0 / NAP_TIMING_COUNT_RATE_Hz)
#define SEC2TICK(x) ((x)*NAP_TIMING_COUNT_RATE_Hz)

void timing_setup(void);
bool time_updated_within(gps_time_t* current_time, double timeout);
gps_time_t get_current_time(void);
void set_time(u64 tc, const gps_time_t* t, double accuracy);
void update_time(u64 tc, const gnss_solution* sol);
time_quality_t get_time_quality(void);
gps_time_t napcount2gpstime(double tc);
u64 gpstime2napcount(const gps_time_t* t);
u64 timing_getms(void);
gps_time_t glo2gps_with_utc_params(const glo_time_t* glo_time,
                                   const gps_time_t* ref_time);
gps_time_t gps_time_round_to_epoch(const gps_time_t* time, double soln_freq);
double get_clock_drift(void);
double sub_2ms_cpo_correction(u64 tc);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
