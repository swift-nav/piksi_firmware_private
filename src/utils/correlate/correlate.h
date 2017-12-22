/*
 * Copyright (C) 2013,2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_CORRELATE_H
#define SWIFTNAV_CORRELATE_H

#include <libswiftnav/common.h>

/** \defgroup corr Correlation
 * Correlators used for tracking.
 * \{ */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void l1_ca_track_correlate(const s8* samples,
                           size_t samples_len,
                           const s8* code,
                           u32 chips_to_correlate,
                           double* init_code_phase,
                           double code_step,
                           double* init_carr_phase,
                           double carr_step,
                           double* I_E,
                           double* Q_E,
                           double* I_P,
                           double* Q_P,
                           double* I_L,
                           double* Q_L,
                           u32* num_samples);

void l2c_cm_track_correlate(const s8* samples,
                            size_t samples_len,
                            const s8* code,
                            u32 chips_to_correlate,
                            double* init_code_phase,
                            double code_step,
                            double* init_carr_phase,
                            double carr_step,
                            double* I_E,
                            double* Q_E,
                            double* I_P,
                            double* Q_P,
                            double* I_L,
                            double* Q_L,
                            u32* num_samples);

void glo_ca_track_correlate(const s8* samples,
                            size_t samples_len,
                            const s8* code,
                            u32 chips_to_correlate,
                            double* init_code_phase,
                            double code_step,
                            double* init_carr_phase,
                            double carr_step,
                            double* I_E,
                            double* Q_E,
                            double* I_P,
                            double* Q_P,
                            double* I_L,
                            double* Q_L,
                            u32* num_samples);

/** \} */

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_CORRELATE_H */
