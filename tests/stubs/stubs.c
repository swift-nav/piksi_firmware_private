/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "manage.h"
#include "sbp/sbp.h"

#define MAX_ACQUIRED_SV_INDEX 15

/* Piksi V3 TCXO nominal temperature frequency stability [ppm] */
#define TCXO_FREQ_STAB_PPM 0.28f
/* Maximum TCXO offset. Includes TCXO nominal temperature frequency stability,
   aging and soldering [ppm] */
#define TCXO_FREQ_OFFSET_MAX_PPM (TCXO_FREQ_STAB_PPM + 2.42)

#define TCXO_FREQ_HZ 10e6 /* TCXO nominal frequency [Hz] */

/* TCXO offset to Hz conversion factor.
   With TCXO frequency set to 10MHz the GLO L1 IF is computed like this:
   GLO_L1_HZ - 10e6 * 159 = 1602.0e6 - 10e6 * 159 = 12.0e6 [Hz] */
#define GLO_L1_TCXO_PPM_TO_HZ (TCXO_FREQ_HZ * 1e-6 * 159.)

#define TRACKING_ELEVATION_UNKNOWN 100 /* Default to above elev. mask */

extern test_case_t *test_case;
extern test_case_t test_cases;

extern bool hw_has_run;   /** Set if acq_search is called */
extern u32 hw_code_index; /** Set to code index for which hw was run */

extern u32 stubs_now_ms;

/*** SEARCH MANAGER UNIT TESTS STUBS ***/
/** Check if SV is tracked
 *
 * \param sid SV identifier
 *
 * \return true is SV is tracked, false otherwise
 */
bool mesid_is_tracked(const me_gnss_signal_t mesid) {
  if (0 != (test_case->track_mask & (1 << mesid_to_code_index(mesid)))) {
    return true;
  }
  return false;
}

bool tracking_startup_ready(const me_gnss_signal_t mesid) {
  (void)mesid;
  return true;
}

bool is_glo_enabled(void) { return true; }

bool is_sbas_enabled(void) { return true; }

bool is_bds2_enabled(void) { return true; }

bool is_qzss_enabled(void) { return true; }

bool is_galileo_enabled(void) { return true; }

void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known) {
  (void)sat;
  *visible = true;
  *known = true;
}

/** Get SV visibility flags
 *
 * \param[in] sid GNSS signal identifier
 * \param[out] visible set if SV is visible
 * \param[out] known set if SV is known visible or known invisible
 */
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known) {
  if (0 != (test_case->vis_mask & (1 << sid_to_code_index(sid)))) {
    *visible = true;
  } else {
    *visible = false;
  }
  if (0 != (test_case->known_mask & (1 << sid_to_code_index(sid)))) {
    *known = true;
  } else {
    *known = false;
  }
}

void sm_calc_all_glo_visibility_flags(void){};

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
u64 timing_getms(void) { return (u64)stubs_now_ms; }

/** Get HW time of the last good fix (LGF)
 *
 * \param[out] lgf_stamp time of LGF (ms)
 * \return true lgf_stamp is valid, false otherwise
 */
bool sm_lgf_stamp(u64 *lgf_stamp) {
  *lgf_stamp = (u64)test_case->lgf_stamp_ms;
  return true;
}

/*** TASK GENERATOR UNIT TESTS STUBS ***/
/* Stub functions */
gps_time_t get_current_time(void) {
  gps_time_t t;
  t.wn = 0;
  t.tow = TOW_UNKNOWN;
  return t;
}

void dum_get_doppler_wndw(const gnss_signal_t *sid,
                          const gps_time_t *t,
                          const last_good_fix_t *lgf,
                          float speed,
                          float *doppler_min,
                          float *doppler_max) {
  (void)sid;
  (void)t;
  (void)lgf;
  (void)speed;
  *doppler_min = 100;
  *doppler_max = 200;
}

float code_to_tcxo_doppler_min(code_t code) {
  (void)code;
  return -TCXO_FREQ_OFFSET_MAX_PPM * GLO_L1_TCXO_PPM_TO_HZ;
}

/** Return the maximum Doppler value for a code induced by TCXO error.
 *
 * \param code The code to use.
 * \return Maximum Doppler value [Hz]
 */
float code_to_tcxo_doppler_max(code_t code) {
  (void)code;
  return TCXO_FREQ_OFFSET_MAX_PPM * GLO_L1_TCXO_PPM_TO_HZ;
}

/*** SCHEDULER UNIT TESTS STUBS ***/
u16 get_orbit_slot(const u16 fcn) {
  (void)fcn;
  return GLO_ORBIT_SLOT_UNKNOWN;
}

void tracker_set_sbas_provider_change_flag(void) {}

u8 tracking_startup_request(const tracking_startup_params_t *startup_params) {
  /* Remove from acquisition */
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  u16 idx = sm_constellation_to_start_index(data->constellation);
  data->jobs[0][mesid_to_code_index(startup_params->mesid) + idx].needs_to_run =
      false;
  data->jobs[1][mesid_to_code_index(startup_params->mesid) + idx].needs_to_run =
      false;
  return 0;
}

void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found) {
  (void)job;
  (void)acq_result;
  (void)peak_found;
}

void dum_report_reacq_result(const gnss_signal_t *sid, bool res) {
  (void)sid;
  (void)res;
}

void acq_result_send(const me_gnss_signal_t mesid,
                     float cn0,
                     float cp,
                     float cf) {
  (void)mesid;
  (void)cn0;
  (void)cp;
  (void)cf;
}

bool soft_multi_acq_search(const me_gnss_signal_t mesid,
                           float _fCarrFreqMin,
                           float _fCarrFreqMax,
                           acq_result_t *p_acqres) {
  (void)_fCarrFreqMin;
  (void)_fCarrFreqMax;
  (void)p_acqres;
  u32 i = mesid_to_code_index(mesid);
  hw_has_run = true;
  hw_code_index = i;
  stubs_now_ms++;
  if (i <= MAX_ACQUIRED_SV_INDEX) {
    return false;
  }
  p_acqres->cf = i * 100;
  p_acqres->cn0 = 30.0f + (float)i;
  p_acqres->cp = i * 10;
  return true;
}

ndb_op_code_t ndb_lgf_read(last_good_fix_t *lgf) {
  (void)lgf;
  return NDB_ERR_UNSUPPORTED;
}

ndb_op_code_t ndb_utc_params_read(utc_params_t *utc_params_p, bool *is_nv) {
  (void)utc_params_p;
  (void)is_nv;
  return NDB_ERR_UNSUPPORTED;
}

ndb_op_code_t ndb_almanac_read(gnss_signal_t sid, almanac_t *a) {
  (void)sid;
  (void)a;
  return NDB_ERR_MISSING_IE;
}

u8 code_track_count(code_t code) {
  (void)code;
  return 0;
}

u8 constellation_track_count(constellation_t gnss) {
  (void)gnss;
  return 0;
}

/** SBP stubs */
s8 sbp_send_msg(u16 msg_type, u8 len, u8 buff[]) {
  (void)msg_type;
  (void)len;
  (void)buff;
  return SBP_OK;
}

s8 sbp_send_msg_(u16 msg_type, u8 len, u8 buff[], u16 sender_id) {
  (void)msg_type;
  (void)len;
  (void)buff;
  (void)sender_id;
  return SBP_OK;
}

void sbp_register_cbk(u16 msg_type,
                      sbp_msg_callback_t cb,
                      sbp_msg_callbacks_node_t *node) {
  (void)msg_type;
  (void)cb;
  (void)node;
}

bool track_sid_db_elevation_degrees_get(const gnss_signal_t sid,
                                        double *result) {
  (void)sid;
  (void)result;
  return false;
}
