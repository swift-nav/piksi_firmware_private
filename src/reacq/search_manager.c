/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Perttu Salmela <psalmela@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <string.h>
#include <swiftnav/glo_map.h>
#include "manage.h"
#include "ndb/ndb_lgf.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "search_manager_api.h"
#include "timing/timing.h"

/** Re-acq normal priority masks. */
static const u32 reacq_normal_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b000000000100000000010000000001, /* SBAS */
    0b111111111111111111111111111111, /* GLO */
    0b100010001000100010001000100010, /* BDS2 */
    0b010101010101010101010101010101, /* QZSS */
    0b101110111011101110111011101110  /* GAL */
};

/** Re-acq gps high priority masks. */
static const u32 reacq_gps_high_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b000000000000000000000000000001, /* SBAS */
    0b000010000100001000010000100001, /* GLO */
    0b000100000000000000000001000000, /* BDS2 */
    0b001000010000100001000010000100, /* QZSS */
    0b010000101001010010100100001010  /* GAL */
};

/** Re-acq sbas high priority masks. */
static const u32 reacq_sbas_high_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b101010101010101010101010101010, /* SBAS */
    0b000010000100001000010000100001, /* GLO */
    0b000100000000000000000001000000, /* BDS2 */
    0b001000010000100001000010000100, /* QZSS */
    0b010000101001010010100100001010  /* GAL */
};

/* Search manager functions which call other modules */

bool sm_lgf_stamp(u64 *lgf_stamp);
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known);
void sm_calc_all_glo_visibility_flags(void);
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known);
bool is_constellation_enabled(constellation_t con);

/**
 * The function returns start job index according to gnss
 * \param[in] gnss Constellation
 * \return Start index of GNSS in job array
 */
u16 sm_constellation_to_start_index(constellation_t gnss) {
  switch ((s8)gnss) {
    case CONSTELLATION_GPS:
      return 0;
    case CONSTELLATION_GLO:
      return NUM_SATS_GPS;
    case CONSTELLATION_SBAS:
      return NUM_SATS_GPS + NUM_SATS_GLO;
    case CONSTELLATION_BDS:
      return NUM_SATS_GPS + NUM_SATS_GLO + NUM_SATS_SBAS;
    case CONSTELLATION_QZS:
      return NUM_SATS_GPS + NUM_SATS_GLO + NUM_SATS_SBAS + NUM_SATS_BDS;
    case CONSTELLATION_GAL:
      return NUM_SATS_GPS + NUM_SATS_GLO + NUM_SATS_SBAS + NUM_SATS_BDS +
             NUM_SATS_QZS;
    default:
      assert(!"Incorrect constellation");
  }
}

/**
 * Helper function. Return SBAS mask depending on user position
 * \return mask for SBAS SV.
 */
static u32 sbas_limit_mask(void) {
  /* read LGF */
  last_good_fix_t lgf;
  if (NDB_ERR_NONE != ndb_lgf_read(&lgf)) {
    /* cannot read LGF for some reason, so set mask for all possible SBAS SV*/
    return sbas_select_prn_mask(SBAS_WAAS) | sbas_select_prn_mask(SBAS_EGNOS) |
           sbas_select_prn_mask(SBAS_GAGAN) | sbas_select_prn_mask(SBAS_MSAS);
  } else {
    static sbas_system_t sbas_provider = SBAS_NONE;
    sbas_system_t new_provider = sbas_select_provider(&lgf);
    if ((sbas_provider != new_provider) && (SBAS_NONE != sbas_provider)) {
      tracker_set_sbas_provider_change_flag();
    }
    sbas_provider = new_provider;
    return sbas_select_prn_mask(sbas_provider);
  }
}

/** Global search job data */
acq_jobs_state_t acq_all_jobs_state_data;

/* Search manager API functions */

/** Initialize search manager module
 *
 * \param data pointer to global job data
 *
 * \return none
 */
void sm_init(acq_jobs_state_t *data) {
  memset(data, 0, sizeof(acq_jobs_state_t));

  acq_job_types_e type;

  struct init_struct {
    constellation_t gnss;
    u16 first_prn;
  } reacq_gnss[] = {{CONSTELLATION_GPS, GPS_FIRST_PRN},
                    {CONSTELLATION_GLO, GLO_FIRST_PRN},
                    {CONSTELLATION_SBAS, SBAS_FIRST_PRN},
                    {CONSTELLATION_BDS, BDS_FIRST_PRN},
                    {CONSTELLATION_QZS, QZS_FIRST_PRN},
                    {CONSTELLATION_GAL, GAL_FIRST_PRN}};

  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    u32 i, k;
    for (k = 0; k < ARRAY_SIZE(reacq_gnss); k++) {
      constellation_t gnss = reacq_gnss[k].gnss;
      u16 idx = sm_constellation_to_start_index(gnss);
      u16 num_sv = constellation_to_sat_count(gnss);
      code_t code = constellation_to_l1_code(gnss);
      if ((CODE_GAL_E1C == code) || (CODE_GAL_E1X == code)) {
        code = CODE_GAL_E1B;
      }
      acq_job_t *job = &data->jobs[type][idx];
      u16 first_prn = reacq_gnss[k].first_prn;
      for (i = 0; i < num_sv; i++) {
        if (CONSTELLATION_GLO == gnss) {
          /* NOTE: GLO MESID is initialized evenly with all FCNs, so that
           * blind searches are immediately done with whole range of FCNs */
          job[i].mesid = construct_mesid(code, first_prn + (i % GLO_MAX_FCN));
        } else {
          job[i].mesid = construct_mesid(code, first_prn + i);
        }
        job[i].sid = construct_sid(code, first_prn + i);
        job[i].job_type = type;
      }
    }
  }
  /* When constellation is initialized with CONSTELLATION_INVALID,
   * sm_constellation_select() will choose GPS as the first constellation. */
  data->constellation = CONSTELLATION_INVALID;
}

/** Checks if deep searches need to run for SV
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run(acq_jobs_state_t *jobs_data) {
  assert(jobs_data != NULL);

  constellation_t con = jobs_data->constellation;

  if (!is_constellation_enabled(con)) {
    return;
  }

  u32 i;
  u16 idx = sm_constellation_to_start_index(con);
  u16 num_sv = constellation_to_sat_count(con);

  u32 sbas_mask = 0;
  if (CONSTELLATION_SBAS == con) {
    sbas_mask = sbas_limit_mask();
    if ((0 == sbas_mask) ||
        (constellation_track_count(CONSTELLATION_SBAS) >= SBAS_SV_NUM_LIMIT)) {
      /* mark all SBAS SV as not needed to run*/
      for (i = 0; i < num_sv; i++) {
        jobs_data->jobs[ACQ_JOB_DEEP_SEARCH][idx + i].needs_to_run = false;
      }
      /* exit to prevent unnecessary reacq */
      return;
    }
  }

  for (i = 0; i < num_sv; i++) {
    acq_job_t *deep_job = &jobs_data->jobs[ACQ_JOB_DEEP_SEARCH][idx + i];
    deep_job->needs_to_run = false;

    if ((CONSTELLATION_SBAS == con) && !((sbas_mask >> i) & 1)) {
      /* don't set job for those SBAS SV which are not in our SBAS range */
      continue;
    }

    me_gnss_signal_t *mesid = &deep_job->mesid;
    gnss_signal_t sid = deep_job->sid;

    bool visible = false;
    bool known = false;

    if (CONSTELLATION_GLO == con) {
      u16 glo_fcn = GLO_FCN_UNKNOWN;
      if (glo_map_valid(sid)) {
        glo_fcn = glo_map_get_fcn(sid);
        *mesid = construct_mesid(CODE_GLO_L1OF, glo_fcn);
        sm_get_visibility_flags(sid, &visible, &known);
      }
    }

    assert(mesid_valid(*mesid));
    assert(sid_valid(sid));
    assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

    if (mesid_is_tracked(*mesid)) {
      continue;
    }
    if (!tracking_startup_ready(*mesid)) {
      continue;
    }

    if (con != CONSTELLATION_GLO) {
      sm_get_visibility_flags(sid, &visible, &known);
    }
    visible = visible && known;

    if (CONSTELLATION_SBAS == con) {
      /* sbas_mask SVs are visible as they were selected by location. */
      visible = true;
    }

    if (visible) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }

    if (CONSTELLATION_GLO == con && !glo_map_valid(sid)) {
      deep_job->cost_hint = ACQ_COST_AVG;
    }
  } /* loop SVs */
}

/** Checks if fallback searches need to run for SV
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run(acq_jobs_state_t *jobs_data,
                                   u64 now_ms,
                                   u64 lgf_age_ms) {
  assert(jobs_data != NULL);

  constellation_t con = jobs_data->constellation;

  if (!is_constellation_enabled(con)) {
    return;
  }

  u32 i;
  u16 idx = sm_constellation_to_start_index(con);
  u16 num_sv = constellation_to_sat_count(con);

  u32 sbas_mask = 0;
  if (CONSTELLATION_SBAS == con) {
    sbas_mask = sbas_limit_mask();
    if ((0 == sbas_mask) ||
        (constellation_track_count(CONSTELLATION_SBAS) >= SBAS_SV_NUM_LIMIT)) {
      /* mark all SBAS SV as not needed to run */
      for (i = 0; i < num_sv; i++) {
        jobs_data->jobs[ACQ_JOB_FALLBACK_SEARCH][idx + i].needs_to_run = false;
      }
      /* exit to prevent unnecessary reacq */
      return;
    }
  }

  for (i = 0; i < num_sv; i++) {
    acq_job_t *fallback_job;
    fallback_job = &jobs_data->jobs[ACQ_JOB_FALLBACK_SEARCH][idx + i];
    fallback_job->needs_to_run = false;

    if ((CONSTELLATION_SBAS == con) && !((sbas_mask >> i) & 1)) {
      /* don't set job for those SBAS SV which are not in our SBAS range */
      continue;
    }

    me_gnss_signal_t *mesid = &fallback_job->mesid;
    gnss_signal_t sid = fallback_job->sid;

    bool visible = false;
    bool known = false;
    bool invisible = false;

    if (CONSTELLATION_GLO == con) {
      u16 glo_fcn = GLO_FCN_UNKNOWN;
      if (glo_map_valid(sid)) {
        glo_fcn = glo_map_get_fcn(sid);
        *mesid = construct_mesid(CODE_GLO_L1OF, glo_fcn);
        sm_get_visibility_flags(sid, &visible, &known);
      }
    }

    assert(mesid_valid(*mesid));
    assert(sid_valid(sid));
    assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);

    if (mesid_is_tracked(*mesid)) {
      continue;
    }
    if (!tracking_startup_ready(*mesid)) {
      continue;
    }

    if (con != CONSTELLATION_GLO) {
      sm_get_visibility_flags(sid, &visible, &known);
    }
    visible = visible && known;
    invisible = !visible && known;

    if (CONSTELLATION_SBAS == con) {
      /* sbas_mask SVs are visible as they were selected by location. */
      visible = true;
    }

    if (visible && lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
        now_ms - fallback_job->stop_time >
            ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_AVG;
      fallback_job->cost_delta = ACQ_COST_DELTA_VISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    } else if ((!known && lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
                now_ms - fallback_job->stop_time >
                    ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) ||
               (CONSTELLATION_GLO == con && !glo_map_valid(sid))) {
      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->cost_delta = ACQ_COST_DELTA_UNKNOWN_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    } else if (invisible && lgf_age_ms >= ACQ_LGF_TIMEOUT_INVIS_MS &&
               now_ms - fallback_job->stop_time >
                   ACQ_FALLBACK_SEARCH_TIMEOUT_INVIS_MS) {
      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->cost_delta = ACQ_COST_DELTA_INVISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    }
  } /* loop SVs */
}

/**
 * Check if constellation is scheduled for reacq, based on priority mask bit.
 *
 * The method extracts the scheduling bit from the priority mask.
 *
 * \return true  if constellation is scheduled (mask bit is 1).
 *         false otherwise
 */
bool check_priority_mask(reacq_prio_level_t prio_level,
                         acq_jobs_state_t *jobs_data) {
  u32 priority_mask = 0;
  switch (prio_level) {
    case REACQ_NORMAL_PRIO:
      assert((u8)jobs_data->constellation < ARRAY_SIZE(reacq_normal_prio));
      priority_mask = reacq_normal_prio[jobs_data->constellation];
      break;

    case REACQ_GPS_HIGH_PRIO:
      assert((u8)jobs_data->constellation < ARRAY_SIZE(reacq_gps_high_prio));
      priority_mask = reacq_gps_high_prio[jobs_data->constellation];
      break;

    case REACQ_SBAS_HIGH_PRIO:
      assert((u8)jobs_data->constellation < ARRAY_SIZE(reacq_sbas_high_prio));
      priority_mask = reacq_sbas_high_prio[jobs_data->constellation];
      break;

    case REACQ_PRIO_COUNT:
    default:
      assert(!"Unsupported re-acq priority mask");
  }

  priority_mask >>= jobs_data->priority_counter;
  priority_mask &= 0x1;
  return priority_mask;
}

/**
 * Check if given constellation is supported.
 *
 * \return true  if constellation is supported.
 *         false otherwise
 */
bool is_constellation_enabled(constellation_t con) {
  switch (con) {
    case CONSTELLATION_GPS:
      return true;
      break;

    case CONSTELLATION_SBAS:
      return is_sbas_enabled();
      break;

    case CONSTELLATION_GLO:
      return is_glo_enabled();
      break;

    case CONSTELLATION_BDS:
      return is_bds2_enabled();
      break;

    case CONSTELLATION_QZS:
      return is_qzss_enabled();
      break;

    case CONSTELLATION_GAL:
      return is_galileo_enabled();
      break;

    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      assert(!"Unsupported reacq constellation!");
      break;
  }
  return false;
}

static reacq_prio_level_t get_dynamic_prio(void) {
  if (code_track_count(CODE_GPS_L1CA) < LOW_GPS_L1CA_SV_LIMIT) {
    return REACQ_GPS_HIGH_PRIO;
  }
  if (code_track_count(CODE_SBAS_L1CA) < LOW_SBAS_L1CA_SV_LIMIT) {
    return REACQ_SBAS_HIGH_PRIO;
  }
  return REACQ_NORMAL_PRIO;
}

/**
 * Check if current constellation is supported and scheduled for reacqusition.
 *
 * \return true  if constellation is supported and scheduled.
 *         false otherwise
 */
bool reacq_scheduled(acq_jobs_state_t *jobs_data) {
  reacq_prio_level_t prio_level = get_dynamic_prio();

  return (is_constellation_enabled(jobs_data->constellation) &&
          check_priority_mask(prio_level, jobs_data));
}

/**
 * Select constellation for reacqusition.
 *
 * The method selects next scheduled constellation for reacquisition.
 *
 * \return None
 */
void sm_constellation_select(acq_jobs_state_t *jobs_data) {
  while (true) {
    if (CONSTELLATION_COUNT == ++jobs_data->constellation) {
      jobs_data->constellation = CONSTELLATION_GPS;
      if (REACQ_PRIORITY_CYCLE == ++jobs_data->priority_counter) {
        jobs_data->priority_counter = 0;
      }
    }

    if (reacq_scheduled(jobs_data)) {
      return;
    }
  }
}

/** Run search manager
 *
 *  Decides when and which jobs need to be run
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
void sm_run(acq_jobs_state_t *jobs_data) {
  u64 now_ms = timing_getms();
  u64 lgf_ms, lgf_age_ms;
  if (sm_lgf_stamp(&lgf_ms) && (now_ms >= lgf_ms)) {
    lgf_age_ms = now_ms - lgf_ms;
  } else {
    lgf_age_ms =
        MAX(ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS, ACQ_LGF_TIMEOUT_INVIS_MS);
  }

  if (CONSTELLATION_GLO == jobs_data->constellation) {
    sm_calc_all_glo_visibility_flags();
  }
  sm_deep_search_run(jobs_data);
  sm_fallback_search_run(jobs_data, now_ms, lgf_age_ms);
}
