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

#include <libswiftnav/common.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/single_epoch_solver.h>

#include "nap/nap_constants.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** \addtogroup timing Timing
 * \{ */

typedef enum {
  TIME_UNKNOWN = 0, /**< GPS time is completely unknown, estimate invalid. */
  TIME_COARSE,      /**< GPS time is known roughly, within 10 ms. */
  TIME_PROPAGATED, /**< GPS time was known but is now propagated, accurate to at
                      least a microsecond. */
  TIME_FINE,       /**< GPS time is known precisely with reference to the local
                      SwiftNAP timer, accurate within 100 ns. */
  TIME_FINEST      /**< GPS time is known precisely with reference to the local
                      SwiftNAP timer, accurate within 10 ns. */
} time_quality_t;

/** \} */

#define RX_DT_NOMINAL (1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz)
#define SEC2TICK(x) ((x)*NAP_FRONTEND_SAMPLE_RATE_Hz)

void timing_setup(void);
gps_time_t get_current_time(void);
void set_time(u64 tc, const gps_time_t* t, double accuracy);
void update_time(u64 tc, const gnss_solution* sol);
time_quality_t get_time_quality(void);
void adjust_rcvtime_offset(const double dt);
gps_time_t napcount2gpstime(const double tc);
gps_time_t napcount2rcvtime(const double tc);
u64 gpstime2napcount(const gps_time_t* t);
u64 rcvtime2napcount(const gps_time_t* t);
u64 timing_getms(void);
gps_time_t glo2gps_with_utc_params(me_gnss_signal_t mesid,
                                   const glo_time_t* glo_t);
gps_time_t gps_time_round_to_epoch(const gps_time_t* time, double soln_freq);
double get_clock_drift(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
