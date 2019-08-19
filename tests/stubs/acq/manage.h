/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STUBS_MANAGE_H
#define STUBS_MANAGE_H

#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include "ch.h"
#include "ndb/ndb_common.h"
#include "position/position.h"
#include "search_manager_api.h"
#include "soft_macq_main.h"
#include "track_cfg.h"

#define ACQ_FULL_CF_STEP ((99.375e6 / 25) / (16 * 1024))

#define ACQ_THRESHOLD 38.0 /* dBHz */

/** How many SBAS SV can be tracked */
#define SBAS_SV_NUM_LIMIT 3

/** Unit test input data type */
typedef struct {
  u32 now_ms;
  u32 vis_mask, known_mask, track_mask;
  u32 lgf_stamp_ms;
  u32 deep_mask,     /**< Expected results */
      fallback_mask; /**< Expected results */
} test_case_t;

typedef struct {
  me_gnss_signal_t mesid; /**< ME signal identifier. */
  u16 glo_slot_id;        /**< GLO orbital slot. */
  u64 sample_count;       /**< Reference NAP sample count. */
  float doppler_hz;       /**< Doppler frequency (Hz). */
  double code_phase;      /**< Code phase (chips). */
  u32 chips_to_correlate; /**< Chips to integrate over. */
  float cn0_init;         /**< C/N0 estimate (dBHz). */
} tracking_startup_params_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

float get_solution_elevation_mask(void);
bool mesid_is_tracked(const me_gnss_signal_t mesid);
bool tracking_startup_ready(const me_gnss_signal_t mesid);
bool is_glo_enabled(void);
bool is_sbas_enabled(void);
bool is_bds2_enabled(void);
bool is_qzss_enabled(void);
bool is_galileo_enabled(void);
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known);
void sm_calc_all_glo_visibility_flags(void);
bool sm_lgf_stamp(u64 *lgf_stamp);
gps_time_t get_current_time(void);
void dum_get_doppler_wndw(const gnss_signal_t *sid,
                          const gps_time_t *t,
                          const last_good_fix_t *lgf,
                          float speed,
                          float *doppler_min_hz,
                          float *doppler_max_hz);
u16 get_orbit_slot(const u16 fcn);
void tracker_set_sbas_provider_change_flag(void);
u8 tracking_startup_request(const tracking_startup_params_t *startup_params);
void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found);
void dum_report_reacq_result(const gnss_signal_t *sid, bool res);
void acq_result_send(const me_gnss_signal_t mesid,
                     float cn0,
                     float cp,
                     float df_hz);
void sch_initialize_cost(acq_job_t *init_job,
                         const acq_jobs_context_t *all_jobs_data);
bool soft_multi_acq_search(const me_gnss_signal_t mesid,
                           float doppler_min_hz,
                           float doppler_max_hz,
                           acq_result_t *p_acqres);
reacq_sched_ret_t sch_select_job(acq_jobs_context_t *jobs_data,
                                 acq_job_t **job_to_run);
reacq_sched_ret_t sch_run(acq_jobs_context_t *jobs_data);
bool soft_multi_acq_search(const me_gnss_signal_t mesid,
                           float doppler_min_hz,
                           float doppler_max_hz,
                           acq_result_t *p_acqres);
ndb_op_code_t ndb_lgf_read(last_good_fix_t *lgf);
u16 sm_constellation_to_start_index(constellation_t gnss);
u8 code_track_count(code_t code);
u8 constellation_track_count(constellation_t gnss);
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif
