/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
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

#ifndef SWIFTNAV_NAP_COMMON_H
#define SWIFTNAV_NAP_COMMON_H

#include <ch.h>
#include <stdbool.h>
#include <swiftnav/common.h>

#include "ext_events/ext_events.h"
#include "nap/nap_constants.h"
#include "nap/nap_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup nap
 * \{ */

/** A complex IQ correlation. */
typedef struct {
  s32 I; /**< In-phase correlation. */
  s32 Q; /**< Quadrature correlation. */
} corr_t;

/** Accumulation of IQ correlations.
 * Used to receive accumulation of correlations from Acq channel in the form
 * acc.I = sum(tap[N].corr_i^2) for N = 0 to NAP_ACQ_N_TAPS-1
 * acc.Q = sum(tap[N].corr_q^2) for N = 0 to NAP_ACQ_N_TAPS-1
 */
typedef struct {
  u64 I; /**< In-phase correlation accumulation. */
  u64 Q; /**< Quadrature correlation accumulation. */
} acc_t;

extern binary_semaphore_t timing_strobe_sem;
extern u8 nap_dna[NAP_DNA_LENGTH];

/** The maximum expected correlation length for sanity checks [ms] */
#define NAP_CORR_LENGTH_MAX_MS 30
/** The minimum expected correlation length for sanity checks [ms] */
#define NAP_CORR_LENGTH_MIN_MS 0.5

/** Convert milliseconds to NAP samples */
#define NAP_MS_2_SAMPLES(ms) (((double)(ms) / 1000.) * NAP_TRACK_SAMPLE_RATE_Hz)

/** Convert NAP samples to milliseconds */
#define NAP_SAMPLES_2_MS(samples) (((samples)*1000.) / NAP_TRACK_SAMPLE_RATE_Hz)

/** \} */

void nap_setup(void);

void nap_rd_dna(u8 dna[]);

void nap_dna_callback_register(void);

u64 nap_timing_count(void);
u32 nap_timing_count_latched(void);
void nap_timing_strobe(u32 falling_edge_count);
bool nap_timing_strobe_wait(u32 timeout);
u64 nap_sample_time_to_count(u32 sample_count);
double nap_count_to_ms(u64 delta_time);
double nap_count_to_ns(u64 delta_time);

u32 nap_get_ext_event(u8 pin, ext_event_trigger_t *trig);
void nap_set_ext_event(u8 pin, ext_event_trigger_t trig, u32 timeout);

void nap_pps(u32 count);
void nap_pps_config(u32 microseconds, u8 active);
bool nap_pps_armed(void);

void nap_set_can_termination(void);
void nap_unset_can_termination(void);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_NAP_COMMON_H */
