/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PVT_DRIVER_C_H
#define STARLING_PVT_DRIVER_C_H

#include <pvt_common/observations.h>
#include <pvt_driver/debug.h>
#include <pvt_driver/external_functions.h>
#include <pvt_driver/solutions.h>
#include <pvt_driver/types.h>

#include <libpal/error.h>
#include <pvt_engine/process_noise_motion.h>
#include <pvt_engine/solution_frequency.h>
#include <stdbool.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/sbas_raw_data.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pvt_driver_s;
typedef struct pvt_driver_s *pvt_driver_t;

pvt_driver_t pvt_driver_new(void);
void pvt_driver_delete(pvt_driver_t ctx);

/* Initializes from only ME type and frequency bands,
 * use other init() function to customize other config parameters */
enum pal_error pvt_driver_simple_init(
    pvt_driver_t ctx, pvt_driver_config_me_t me_config,
    pvt_driver_insights_config_t insights_config);

/* Initializes from a complete config structure.
 * Requires ME type and frequency bands to be set,
 * set size of array parameters to zero to use default values for them. */
enum pal_error pvt_driver_init(pvt_driver_t ctx, pvt_driver_config_t config,
                               pvt_driver_insights_config_t insights_config);

bool pvt_driver_start_engine(pvt_driver_t ctx);
void pvt_driver_stop_engine(pvt_driver_t ctx);

obs_array_t *pvt_driver_alloc_rover_obs(pvt_driver_t ctx);
obs_array_t *pvt_driver_alloc_base_obs(pvt_driver_t ctx);
void pvt_driver_free_rover_obs(pvt_driver_t ctx, obs_array_t *obs_array);
void pvt_driver_free_base_obs(pvt_driver_t ctx, obs_array_t *obs_array);
bool pvt_driver_send_rover_obs(pvt_driver_t ctx, obs_array_t *obs_array);
bool pvt_driver_send_base_obs(pvt_driver_t ctx, obs_array_t *obs_array);
bool pvt_driver_send_ephemerides(pvt_driver_t ctx,
                                 const ephemeris_t *ephemerides, size_t n);
bool pvt_driver_send_sbas_data(pvt_driver_t ctx,
                               const sbas_raw_data_t *sbas_data);
bool pvt_driver_send_prior_position(
    pvt_driver_t ctx, const pvt_driver_prior_position_t *prior_position);

int pvt_driver_add_solution_handler(pvt_driver_t ctx,
                                    pvt_driver_solution_callback_t callbacks);
void pvt_driver_remove_solution_handler(pvt_driver_t ctx, int id);

/* Reset all filters. */
void pvt_driver_reset_all_filters(pvt_driver_t ctx);
/* Reset Time Matched filter. */
void pvt_driver_reset_time_matched_filter(pvt_driver_t ctx);
/* Reset Low Latency filter. */
void pvt_driver_reset_low_latency_filter(pvt_driver_t ctx);

// Settings
void pvt_driver_set_input_bridge_mode(pvt_driver_t ctx,
                                      pvt_driver_bridge_mode_t mode);
void pvt_driver_set_is_glonass_enabled(pvt_driver_t ctx, bool is_enabled);
void pvt_driver_set_is_galileo_enabled(pvt_driver_t ctx, bool is_enabled);
void pvt_driver_set_is_beidou_enabled(pvt_driver_t ctx, bool is_enabled);
void pvt_driver_set_is_rtk_fixed_enabled(pvt_driver_t ctx, bool is_enabled);
void pvt_driver_set_is_time_matched_klobuchar_enabled(pvt_driver_t ctx,
                                                      bool is_enabled);
void pvt_driver_set_max_correction_age(pvt_driver_t ctx, unsigned int max_age);
void pvt_driver_set_process_noise_motion(
    pvt_driver_t ctx, PROCESS_NOISE_MOTION_TYPE process_noise_settings);
void pvt_driver_set_downweight_factor(pvt_driver_t ctx, code_t code,
                                      float factor);
void pvt_driver_set_elevation_mask(pvt_driver_t ctx, float elevation_mask);
void pvt_driver_set_min_modelled_baseline_len_km(pvt_driver_t ctx,
                                                 double value);

bool pvt_driver_get_max_sats_from_requested_freq(pvt_driver_t ctx,
                                                 double requested_frequency_hz,
                                                 s32 *max_sats);
bool pvt_driver_set_max_sats(pvt_driver_t ctx, s32 max_sats);
void pvt_driver_set_known_ref_pos(pvt_driver_t ctx, const double base_pos[3]);
void pvt_driver_set_known_glonass_biases(pvt_driver_t ctx,
                                         const glo_biases_t *biases);
pvt_driver_solution_mode_t pvt_driver_get_solution_mode(pvt_driver_t ctx);
void pvt_driver_set_solution_mode(pvt_driver_t ctx,
                                  pvt_driver_solution_mode_t new_mode);
void pvt_driver_set_external_functions(
    pvt_driver_t ctx, pvt_driver_external_functions_t functions);
void pvt_driver_set_debug_functions(
    pvt_driver_t ctx, pvt_driver_debug_functions_t debug_functions);
void pvt_driver_set_thread_synchronization(pvt_driver_t ctx, bool enabled);
void pvt_driver_set_antenna_offset(pvt_driver_t ctx,
                                   pvt_driver_antenna_t antenna);
void pvt_driver_set_is_pl_enabled(pvt_driver_t ctx, bool is_enabled);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // STARLING_PVT_DRIVER_C_H
