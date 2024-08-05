/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_FIRMWARE_BINDING_H
#define LIBSWIFTNAV_PVT_ENGINE_FIRMWARE_BINDING_H

#include <swiftnav/constants.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/single_epoch_solver.h>

#include <pvt_engine/configurations/motion_process_noise_parameters.h>
#include <starling/config.h>
#include <starling/observation.h>
#include <starling/starling.h>

#ifdef __cplusplus
#include <pvt_engine/ambiguity_manager.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/filter_manager_rtk.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/sbas/sbas.h>
#else   // __cplusplus
typedef struct FilterManager FilterManager;
typedef struct FilterManagerRTK FilterManagerRTK;
typedef struct FilterConfiguration FilterConfiguration;
typedef struct AmbiguityManager AmbiguityManager;
typedef struct FilterObservationIdSet FilterObservationIdSet;
typedef struct FilterObservationHandler FilterObservationHandler;
typedef int FILTER_MANAGER_TYPE;
typedef int PROCESSING_MODE;
#endif  // __cplusplus

#ifdef __cplusplus
namespace pvt_engine {
extern "C" {
#endif  // __cplusplus

typedef enum {
  PVT_ENGINE_SUCCESS = 0,
  PVT_ENGINE_NOT_INITIALIZED,
  PVT_ENGINE_FAILURE,
  PVT_ENGINE_NO_OBS,
  PVT_ENGINE_INSUFFICIENT_OBS
} PVT_ENGINE_INTERFACE_RC;

typedef int PVT_ENGINE_CONFIGURATION_TYPE;

/* Create a filter manager object with no event log. */
FilterManagerRTK *create_filter_manager_rtk(
    PVT_ENGINE_CONFIGURATION_TYPE pvt_config,
    const PROCESSING_MODE processing_mode);

FilterManager *create_filter_manager_spp(
    PVT_ENGINE_CONFIGURATION_TYPE pvt_config);

AmbiguityManager *create_amb_manager(PVT_ENGINE_CONFIGURATION_TYPE pvt_config,
                                     FILTER_MANAGER_TYPE upstream_filter_type);

void filter_manager_init(FilterManager *filter_manager);

void copy_filter_manager_rtk(FilterManagerRTK *destination,
                             const FilterManagerRTK *source);

PVT_ENGINE_INTERFACE_RC filter_manager_update(
    FilterManager *filter_manager, FilterObservationIdSet *obs_to_drop,
    bool *reset_downstream_filter);

void filter_manager_overwrite_ephemerides(FilterManager *filter_manager,
                                          s16 num_ephs,
                                          const ephemeris_t *stored_ephs[]);

PVT_ENGINE_INTERFACE_RC filter_manager_get_result(
    const FilterManager *filter_manager, const bool time_matched,
    pvt_engine_result_t *result, StarlingProtectionLevels *pl_result);

bool filter_manager_is_initialized(const FilterManager *filter_manager);

dops_t filter_manager_get_dop_values(const FilterManager *filter_manager);

void filter_manager_update_iono_parameters(FilterManager *filter_manager,
                                           const ionosphere_t *new_iono_params,
                                           const bool disable_klobuchar);

void filter_manager_process_sbas_message(FilterManager *filter_manager,
                                         const sbas_raw_data_t *sbas_data);

void filter_manager_reinitialize_sbas(FilterManager *filter_manager);

PVT_ENGINE_INTERFACE_RC filter_manager_update_rov_obs(
    FilterManager *filter_manager, obss_t *obs_sol);

void filter_manager_set_known_ref_pos(FilterManagerRTK *filter_manager,
                                      const double base_pos[3]);

void filter_manager_set_known_glonass_biases(FilterManagerRTK *filter_manager,
                                             const glo_biases_t biases);

PVT_ENGINE_INTERFACE_RC filter_manager_update_ref_obs(
    FilterManagerRTK *filter_manager, const gps_time_t *obs_time,
    const u8 num_obs, const navigation_measurement_t nav_meas[],
    const measurement_std_t meas_std[], const double ref_spp_position_ecef[3]);

void pvt_engine_covariance_to_accuracy(const double covariance_ecef_in[9],
                                       const double ref_ecef_in[3],
                                       double *accuracy, double *h_accuracy,
                                       double *v_accuracy, double ecef_cov[6],
                                       double ned_cov[6]);

// void pvt_engine_enable_low_latency_mode(FilterManager *filter_manager,
//                                        const bool ebnable_low_latency);

void set_pvt_engine_update_max_sats(FilterManager *filter_manager,
                                    s32 max_sats);
void set_pvt_engine_update_max_sats_amb_man(AmbiguityManager *amb_manager,
                                            s32 max_sats);

void set_pvt_engine_process_noise_motion(
    FilterManager *filter_manager,
    const PROCESS_NOISE_MOTION_TYPE process_noise_motion_type);
void set_pvt_engine_process_noise_motion_amb_man(
    AmbiguityManager *amb_manager,
    const PROCESS_NOISE_MOTION_TYPE process_noise_motion_type);

void set_pvt_engine_elevation_mask(FilterManager *filter_manager,
                                   const double elevation_mask);
void set_pvt_engine_elevation_mask_amb_man(AmbiguityManager *amb_manager,
                                           const double elevation_mask);

void set_pvt_engine_enable_constellation(FilterManager *filter_manager,
                                         const constellation_t constellation,
                                         const bool enable_constellation);
void set_pvt_engine_enable_constellation_amb_man(
    AmbiguityManager *amb_manager, const constellation_t constellation,
    const bool enable_constellation);

void set_pvt_engine_obs_downweight_factor(FilterManager *filter_manager,
                                          const double downweight_factor,
                                          const code_t code_type);
void set_pvt_engine_obs_downweight_factor_amb_man(
    AmbiguityManager *amb_manager, const double downweight_factor,
    const code_t code_type);

void set_pvt_engine_enable_fix_mode(FilterManager *filter_manager,
                                    const bool enable_fix);
void set_pvt_engine_enable_fix_mode_amb_man(AmbiguityManager *amb_manager,
                                            const bool enable_fix);

void set_max_correction_age(FilterManager *filter_manager,
                            const double max_correction_age);
void set_max_correction_age_amb_man(AmbiguityManager *amb_manager,
                                    const double max_correction_age);

void filter_manager_set_apriori_position(FilterManager *filter_manager,
                                         const double apriori_position[3]);

bool filter_manager_has_sbas_corrections(FilterManager *filter_manager,
                                         const gnss_signal_t *sid,
                                         const gps_time_t *epoch_time, u8 IODE);

void set_pvt_engine_receiver_antenna_offset(FilterManager *filter_manager,
                                            const char *antenna_type);
void set_pvt_engine_receiver_antenna_offset_amb_man(
    AmbiguityManager *amb_manager, const char *antenna_type);

#ifdef __cplusplus
}  // extern "C"
}  // namespace pvt_engine
#endif

#endif  // LIBSWIFTNAV_PVT_ENGINE_FIRMWARE_BINDING_H
