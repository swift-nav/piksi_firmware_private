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
#include <swiftnav/ephemeris.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/pvt_result.h>
#include <swiftnav/sbas_raw_data.h>
#include <swiftnav/single_epoch_solver.h>

#include <pvt_engine/configurations/motion_process_noise_parameters.h>
#include <pvt_engine/obss.h>

#include <starling/build/config.h>

#ifdef __cplusplus
#include <pvt_engine/ambiguity_manager.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/filter_manager_interface.h>
#include <pvt_engine/filter_manager_rtk.h>
#include <pvt_engine/filter_manager_spp.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/observation_handler.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sbas/sbas.h>

#else   // __cplusplus
typedef struct FilterManagerInterface FilterManagerInterface;
typedef struct FilterManager FilterManager;
typedef struct FilterManagerPPP FilterManagerPPP;
typedef struct FilterManagerRTK FilterManagerRTK;
typedef struct FilterManagerSPP FilterManagerSPP;
typedef struct FilterConfiguration FilterConfiguration;
typedef struct AmbiguityManager AmbiguityManager;
typedef struct SBASCorrectionsManager SBASCorrectionsManager;
typedef struct FilterObservationIdSet FilterObservationIdSet;
typedef struct FilterObservationHandler FilterObservationHandler;
typedef int FILTER_MANAGER_TYPE;
typedef int PROCESSING_MODE;
typedef struct CommonAprioriModels CommonAprioriModels;
typedef struct CommonAprioriModelConfiguration CommonAprioriModelConfiguration;
typedef struct EphemerisHandler EphemerisHandler;
typedef struct EphemerisHandlerInterface EphemerisHandlerInterface;
typedef struct GnssInputConfiguration GnssInputConfiguration;
typedef void (*get_eph_lock_cb_t)(void);
typedef void (*release_eph_lock_cb_t)(void);
typedef struct protection_level_results_t protection_level_results_t;
typedef int PVT_ENGINE_INTERFACE_RC;
typedef int FILTER_MANAGER_TYPE;
typedef struct RobustEstimatorConfiguration RobustEstimatorConfiguration;
#endif  // __cplusplus

#ifdef __cplusplus
namespace pvt_engine {

using PVT_ENGINE_INTERFACE_RC = pvt_engine::PRC;

extern "C" {
#endif  // __cplusplus

typedef int PVT_ENGINE_CONFIGURATION_TYPE;

EphemerisHandlerInterface *create_ephemeris_handler(
    get_eph_lock_cb_t get_lock_cb, release_eph_lock_cb_t release_lock_cb,
    void *lock_cb_ctx);

/* Create a filter manager object with no event log. */
FilterManagerRTK *create_filter_manager_rtk(
    const GnssInputConfiguration *config, const PROCESSING_MODE processing_mode,
    const CommonAprioriModelConfiguration *common_parameters_,
    EphemerisHandlerInterface *eph_handler,
    CommonAprioriModels *common_apriori_models);

FilterManagerSPP *create_filter_manager_spp(
    const GnssInputConfiguration *config,
    const CommonAprioriModelConfiguration *common_parameters_,
    EphemerisHandlerInterface *eph_handler,
    CommonAprioriModels *common_apriori_models);

AmbiguityManager *create_amb_manager(
    const GnssInputConfiguration *config,
    FILTER_MANAGER_TYPE upstream_filter_type,
    const CommonAprioriModelConfiguration *common_parameters_,
    EphemerisHandlerInterface *eph_handler,
    CommonAprioriModels *common_apriori_models);

void destroy_filter_manager_rtk(FilterManagerRTK *fm);
void destroy_filter_manager_spp(FilterManagerSPP *fm);
void destroy_amb_manager(AmbiguityManager *am);
void destroy_ephemeris_handler(EphemerisHandlerInterface *eh);

void filter_manager_init(FilterManager *filter_manager);

void copy_filter_manager_rtk(FilterManagerRTK *destination,
                             const FilterManagerRTK *source);

PVT_ENGINE_INTERFACE_RC filter_manager_update(
    FilterManagerPPP *filter_manager_ppp, FilterObservationIdSet *obs_to_drop,
    bool *reset_downstream_filter);

PVT_ENGINE_INTERFACE_RC spp_filter_manager_update(
    obss_t *obs_sol, FilterManagerSPP *filter_manager_spp,
    FilterObservationIdSet *obs_to_drop, bool *reset_downstream_filter,
    pvt_engine_result_t *result, protection_level_results_t *pl_result);

PVT_ENGINE_INTERFACE_RC filter_manager_get_result(
    const FilterManagerPPP *filter_manager_ppp, pvt_engine_result_t *result,
    protection_level_results_t *pl_result);

bool filter_manager_is_initialized(const FilterManager *filter_manager);

dops_t filter_manager_get_dop_values(const FilterManager *filter_manager);

void filter_manager_update_iono_parameters(FilterManager *filter_manager,
                                           const ionosphere_t *new_iono_params,
                                           const bool disable_klobuchar);

void filter_manager_overwrite_sbas_manager(
    FilterManager *filter_manager,
    const SBASCorrectionsManager *sbas_corrections_manager);

PVT_ENGINE_INTERFACE_RC filter_manager_update_rov_obs(
    FilterManagerPPP *filter_manager_ppp, obss_t *obs_sol);

void filter_manager_set_known_ref_pos(FilterManagerRTK *filter_manager,
                                      const double base_pos[3]);

void filter_manager_set_known_glonass_biases(FilterManagerRTK *filter_manager,
                                             const glo_biases_t biases);

PVT_ENGINE_INTERFACE_RC filter_manager_update_ref_obs(
    FilterManagerRTK *filter_manager, const gps_time_t *obs_time,
    const u8 num_obs, const navigation_measurement_t nav_meas[],
    const measurement_std_t meas_std[], const double ref_spp_position_ecef[3]);

void pvt_engine_covariance_to_ecef_accuracy(const double covariance_ecef_in[9],
                                            double *accuracy,
                                            double ecef_cov[6]);

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

void set_pvt_engine_enable_constellation(
    FilterManager *filter_manager, FILTER_MANAGER_TYPE filter_manager_type,
    const GnssInputConfiguration *config, const constellation_t constellation,
    const bool enable_constellation);
void set_pvt_engine_enable_constellation_amb_man(
    AmbiguityManager *amb_manager, const GnssInputConfiguration *config,
    const constellation_t constellation, const bool enable_constellation);

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

void filter_manager_set_apriori_position(FilterManager *filter_manager,
                                         const double apriori_position[3]);

bool filter_manager_has_sbas_corrections(FilterManager *filter_manager,
                                         const gnss_signal_t *sid,
                                         const gps_time_t *epoch_time,
                                         u16 IODE);

u32 pvt_engine_make_ephemeris_key(const ephemeris_t *eph);

void set_pvt_engine_enable_pl(FilterManager *filter_manager,
                              const bool enable_pl);

void set_pvt_engine_enable_output_gating(FilterManager *filter_manager,
                                         const bool enable_output_gating);

void set_pvt_engine_robust_estimator(
    FilterManager *filter_manager,
    const RobustEstimatorConfiguration *robust_estimator);

#ifdef __cplusplus
}  // extern "C"
}  // namespace pvt_engine
#endif

#endif  // LIBSWIFTNAV_PVT_ENGINE_FIRMWARE_BINDING_H
