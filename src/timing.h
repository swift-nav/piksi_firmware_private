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
#include <libswiftnav/time.h>

#include "nap/nap_constants.h"

/** \addtogroup timing Timing
 * \{ */

typedef enum {
  TIME_UNKNOWN = 0, /**< GPS time is completely unknown, estimate invalid. */
  TIME_GUESS,       /**< GPS time is just a guess, it could be off by weeks or
                         just totally incorrect. */
  TIME_COARSE,      /**< GPS time is known roughly, within 1 second. */
  TIME_FINE         /**< GPS time is known precisely with reference to the
                         local SwiftNAP timer. */
} time_quality_t;

typedef struct {
  gps_time_t t0_gps;   /**< Clock offset estimate. GPS time when local timer
                            value equals zero. */
  double clock_period; /**< Clock period estimate. */
  double clock_offset; /**< offset to improve precision of GPS time */
  double P[2][2];      /**< State covariance matrix. */
} clock_est_state_t;

/** \} */

extern volatile time_quality_t time_quality;

#define RX_DT_NOMINAL (1.0 / NAP_FRONTEND_SAMPLE_RATE_Hz)

void timing_setup(void);
gps_time_t get_current_time(void);
gps_time_t get_current_gps_time(void);
void set_time(time_quality_t quality, gps_time_t t);
void set_time_fine(u64 tc, gps_time_t t);
void set_gps_time_offset(u64 tc, gps_time_t t);
void adjust_time_fine(double dt);
gps_time_t rx2gpstime(double tc);
gps_time_t rx2rcvtime(double tc);
double gps2rxtime(const gps_time_t* t);
double rcv2rxtime(const gps_time_t* t);
u64 timing_getms(void);

#endif
