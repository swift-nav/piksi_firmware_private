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
#include <libswiftnav/glo_map.h>
#include <string.h>
#include "ndb/ndb_lgf.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "search_manager_api.h"
#include "timing/timing.h"

#include "manage.h"
/** How many SBAS SV can be tracked */
#define SBAS_SV_NUM_LIMIT 3

/** Re-acq normal priority masks. */
static const u32 reacq_normal_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b000000000100000000010000000001, /* SBAS */
    0b111111111111111111111111111111, /* GLO */
    0b101010101010101010101010101010, /* BDS2 */
    0b010101010101010101010101010101, /* QZSS */
    0b101010101010101010101010101010  /* GAL */
};

/** Re-acq low priority masks. */
static const u32 reacq_low_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b000000000000000000000000000001, /* SBAS */
    0b000010000100001000010000100001, /* GLO */
    0b000100001000010000100001000010, /* BDS2 */
    0b001000010000100001000010000100, /* QZSS */
    0b010000100001000010000100001000  /* GAL */
};

/* Search manager functions which call other modules */

bool sm_lgf_stamp(u64 *lgf_stamp);
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known);
void sm_calc_all_glo_visibility_flags(void);
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known);

/**
 * The function calculates how many SV of defined GNSS are in track
 * \param[in] jobs_data Pointer to all jobs
 * \param[in] gnss GNSS constellation type
 */
static u8 sv_track_count(acq_jobs_state_t *jobs_data, constellation_t gnss) {
  assert(jobs_data != NULL);
  u8 num_sats = 0;
  u8 sv_tracked = 0;
  acq_job_t *job_ptr;
  switch ((s8)gnss) {
    case CONSTELLATION_GPS:
      num_sats = NUM_SATS_GPS;
      job_ptr = &jobs_data->jobs_gps[0][0];
      break;
    case CONSTELLATION_GLO:
      num_sats = NUM_SATS_GLO;
      job_ptr = &jobs_data->jobs_glo[0][0];
      break;
    case CONSTELLATION_SBAS:
      num_sats = NUM_SATS_SBAS;
      job_ptr = &jobs_data->jobs_sbas[0][0];
      break;
    default:
      assert(!"Incorrect constellation");
  }
  for (u8 i = 0; i < num_sats; i++) {
    if (mesid_is_tracked(job_ptr[i].mesid)) {
      sv_tracked++;
    }
  }
  return sv_tracked;
}

/**
 * Helper function. Return SBAS mask depending on user position and limit SBAS
 * SV that needs to be acquired
 * \param[in] jobs pointer to jobs list
 * \param[in] job_type Job type
 * \return mask for SBAS SV. 0 mask means no SBAS SV need to be acquired,
 * because we already reach the limit.
 */
static u32 sbas_limit_mask(acq_jobs_state_t *jobs_data,
                           acq_job_types_e job_type) {
  u32 ret = 0;
  if (sv_track_count(jobs_data, CONSTELLATION_SBAS) >= SBAS_SV_NUM_LIMIT) {
    u8 i;
    for (i = 0; i < NUM_SATS_SBAS; i++) {
      /* mark all jobs as not needed to run */
      jobs_data->jobs_sbas[job_type][i].needs_to_run = false;
    }
    return ret;
  }
  /* read LGF */
  last_good_fix_t lgf;
  if (NDB_ERR_NONE != ndb_lgf_read(&lgf)) {
    /* cannot read LGF for some reason, so set mask for all possible SBAS SV*/
    ret = sbas_select_prn_mask(SBAS_WAAS) | sbas_select_prn_mask(SBAS_EGNOS) |
          sbas_select_prn_mask(SBAS_GAGAN) | sbas_select_prn_mask(SBAS_MSAS);
  } else {
    ret = sbas_select_prn_mask(sbas_select_provider(&lgf));
  }
  return ret;
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

  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    u32 i;
    for (i = 0; i < NUM_SATS_GPS; i++) {
      data->jobs_gps[type][i].mesid =
          construct_mesid(CODE_GPS_L1CA, GPS_FIRST_PRN + i);
      /* for GPS SID it's just copy of MESID */
      data->jobs_gps[type][i].sid =
          construct_sid(CODE_GPS_L1CA, GPS_FIRST_PRN + i);
      data->jobs_gps[type][i].job_type = type;
    }
    for (i = 0; i < NUM_SATS_GLO; i++) {
      data->jobs_glo[type][i].sid =
          construct_sid(CODE_GLO_L1OF, GLO_FIRST_PRN + i);
      /* NOTE: GLO MESID is initialized evenly with all FCNs, so that
       * blind searches are immediately done with whole range of FCNs */
      data->jobs_glo[type][i].mesid =
          construct_mesid(CODE_GLO_L1OF, GLO_FIRST_PRN + (i % GLO_MAX_FCN));

      data->jobs_glo[type][i].job_type = type;
    }
    for (i = 0; i < NUM_SATS_SBAS; i++) {
      data->jobs_sbas[type][i].sid =
          construct_sid(CODE_SBAS_L1CA, SBAS_FIRST_PRN + i);
      /* for SBAS SID it's just copy of MESID */
      data->jobs_sbas[type][i].mesid =
          construct_mesid(CODE_SBAS_L1CA, SBAS_FIRST_PRN + i);
      data->jobs_sbas[type][i].job_type = type;
    }
  }
  /* When constellation is initialized with CONSTELLATION_INVALID,
   * sm_constellation_select() will choose GPS as the first constellation. */
  data->constellation = CONSTELLATION_INVALID;
}

/** Checks if deep searches need to run for GPS SV
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run_gps(acq_jobs_state_t *jobs_data) {
  u32 i;
  for (i = 0; i < NUM_SATS_GPS; i++) {
    acq_job_t *deep_job = &jobs_data->jobs_gps[ACQ_JOB_DEEP_SEARCH][i];
    me_gnss_signal_t mesid = deep_job->mesid;
    gnss_signal_t sid = deep_job->sid;

    bool visible, known;

    assert(mesid_valid(mesid));
    assert(sid_valid(sid));
    assert(IS_GPS(mesid));

    assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);
    visible = visible && known;

    if (visible) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }
  } /* loop SVs */
}

/** Checks if deep searches need to run for GLO SV
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run_glo(acq_jobs_state_t *jobs_data) {
  if (!is_glo_enabled()) {
    return;
  }

  u32 i;
  for (i = 0; i < NUM_SATS_GLO; i++) {
    acq_job_t *deep_job = &jobs_data->jobs_glo[ACQ_JOB_DEEP_SEARCH][i];
    me_gnss_signal_t *mesid = &deep_job->mesid;
    gnss_signal_t sid = deep_job->sid;
    u16 glo_fcn = GLO_FCN_UNKNOWN;
    bool visible = false;
    bool known = false;

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;

    if (glo_map_valid(sid)) {
      glo_fcn = glo_map_get_fcn(sid);

      *mesid = construct_mesid(CODE_GLO_L1OF, glo_fcn);

      assert(mesid_valid(*mesid));
      assert(sid_valid(sid));
      assert(IS_GLO(*mesid));
      assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

      /* Check if jobs need to run */
      if (mesid_is_tracked(*mesid)) {
        continue;
      }

      sm_get_glo_visibility_flags(sid.sat, &visible, &known);
      visible = visible && known;
    }

    if (visible) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    } else if (!glo_map_valid(sid)) {
      deep_job->cost_hint = ACQ_COST_AVG;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
    }
  } /* loop SVs */
}

/** Checks if deep searches need to run for SBAS SV
 *
 * \param jobs_data pointer to job data
 *
 * \return none
 */
static void sm_deep_search_run_sbas(acq_jobs_state_t *jobs_data) {
  u8 sbas_limit = SBAS_SV_NUM_LIMIT;
  u32 sbas_mask = sbas_limit_mask(jobs_data, ACQ_JOB_DEEP_SEARCH);
  if (0 == sbas_mask) {
    return;
  }
  u32 i;
  for (i = 0; i < NUM_SATS_SBAS; i++) {
    if (!((sbas_mask >> i) & 1)) {
      /* don't set job for those SBAS SV which are not in our SBAS range */
      continue;
    }
    acq_job_t *deep_job = &jobs_data->jobs_sbas[ACQ_JOB_DEEP_SEARCH][i];
    me_gnss_signal_t mesid = deep_job->mesid;
    gnss_signal_t sid = deep_job->sid;

    bool visible, known;

    assert(mesid_valid(mesid));
    assert(sid_valid(sid));
    assert(IS_SBAS(mesid));

    assert(deep_job->job_type < ACQ_NUM_JOB_TYPES);

    /* Initialize jobs to not run */
    deep_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);

    visible = visible && known;

    if (visible) {
      deep_job->cost_hint = ACQ_COST_MIN;
      deep_job->cost_delta = 0;
      deep_job->needs_to_run = true;
      deep_job->oneshot = false;
      sbas_limit--;
    }
    if (0 == sbas_limit) {
      return;
    }
  } /* loop SVs */
}

/** Checks if fallback searches need to run for GPS SV
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run_gps(acq_jobs_state_t *jobs_data,
                                       u64 now_ms,
                                       u64 lgf_age_ms) {
  u32 i;
  for (i = 0; i < NUM_SATS_GPS; i++) {
    acq_job_t *fallback_job = &jobs_data->jobs_gps[ACQ_JOB_FALLBACK_SEARCH][i];
    me_gnss_signal_t mesid = fallback_job->mesid;
    gnss_signal_t sid = fallback_job->sid;

    bool visible, invisible, known;

    assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);

    assert(mesid_valid(mesid));
    assert(IS_GPS(mesid));

    /* Initialize jobs to not run */
    fallback_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);
    visible = visible && known;
    invisible = !visible && known;

    if (visible && lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
        now_ms - fallback_job->stop_time >
            ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_AVG;
      fallback_job->cost_delta = ACQ_COST_DELTA_VISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
    } else if (!known && lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
               now_ms - fallback_job->stop_time >
                   ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
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

/** Checks if fallback searches need to run for GLO SV
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run_glo(acq_jobs_state_t *jobs_data,
                                       u64 now_ms,
                                       u64 lgf_age_ms) {
  if (!is_glo_enabled()) {
    return;
  }

  u32 i;
  for (i = 0; i < NUM_SATS_GLO; i++) {
    acq_job_t *fallback_job = &jobs_data->jobs_glo[ACQ_JOB_FALLBACK_SEARCH][i];
    me_gnss_signal_t *mesid = &fallback_job->mesid;
    gnss_signal_t sid = fallback_job->sid;
    u16 glo_fcn = GLO_FCN_UNKNOWN;
    bool visible = false;
    bool invisible = false;
    bool known = false;

    /* Initialize jobs to not run */
    fallback_job->needs_to_run = false;

    if (glo_map_valid(sid)) {
      glo_fcn = glo_map_get_fcn(sid);

      *mesid = construct_mesid(CODE_GLO_L1OF, glo_fcn);
      assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);
      assert(mesid_valid(*mesid));
      assert(sid_valid(sid));
      assert(IS_GLO(*mesid));

      /* Check if jobs need to run */
      if (mesid_is_tracked(*mesid)) {
        continue;
      }

      sm_get_glo_visibility_flags(sid.sat, &visible, &known);
      visible = visible && known;
      invisible = !visible && known;
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
               !glo_map_valid(sid)) {
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

/** Checks if fallback searches need to run for SBAS SV
 *
 * \param jobs_data pointer to job data
 * \param now_ms current time (ms)
 * \param lgf_age_ms age of the last good fix (ms)
 *
 * \return none
 */
static void sm_fallback_search_run_sbas(acq_jobs_state_t *jobs_data,
                                        u64 now_ms,
                                        u64 lgf_age_ms) {
  u8 sbas_limit = SBAS_SV_NUM_LIMIT;
  u32 sbas_mask = sbas_limit_mask(jobs_data, ACQ_JOB_FALLBACK_SEARCH);
  if (0 == sbas_mask) {
    return;
  }
  u32 i;
  for (i = 0; i < NUM_SATS_SBAS; i++) {
    if (!((sbas_mask >> i) & 1)) {
      /* don't set job for those SBAS SV which are not in our SBAS range */
      continue;
    }
    acq_job_t *fallback_job = &jobs_data->jobs_sbas[ACQ_JOB_FALLBACK_SEARCH][i];
    me_gnss_signal_t mesid = fallback_job->mesid;
    gnss_signal_t sid = fallback_job->sid;

    bool visible, invisible, known;

    assert(fallback_job->job_type < ACQ_NUM_JOB_TYPES);

    assert(mesid_valid(mesid));
    assert(IS_SBAS(mesid));

    /* Initialize jobs to not run */
    fallback_job->needs_to_run = false;

    /* Check if jobs need to run */
    if (mesid_is_tracked(mesid)) {
      continue;
    }

    sm_get_visibility_flags(sid, &visible, &known);
    visible = visible && known;
    invisible = !visible && known;

    if (visible && lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
        now_ms - fallback_job->stop_time >
            ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_AVG;
      fallback_job->cost_delta = ACQ_COST_DELTA_VISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
      sbas_limit--;
    } else if (!known && lgf_age_ms >= ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS &&
               now_ms - fallback_job->stop_time >
                   ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS) {
      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->cost_delta = ACQ_COST_DELTA_UNKNOWN_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
      sbas_limit--;
    } else if (invisible && lgf_age_ms >= ACQ_LGF_TIMEOUT_INVIS_MS &&
               now_ms - fallback_job->stop_time >
                   ACQ_FALLBACK_SEARCH_TIMEOUT_INVIS_MS) {
      fallback_job->cost_hint = ACQ_COST_MAX_PLUS;
      fallback_job->cost_delta = ACQ_COST_DELTA_INVISIBLE_MS;
      fallback_job->needs_to_run = true;
      fallback_job->oneshot = true;
      sbas_limit--;
    }
    if (0 == sbas_limit) {
      return;
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

    case REACQ_LOW_PRIO:
      assert((u8)jobs_data->constellation < ARRAY_SIZE(reacq_low_prio));
      priority_mask = reacq_low_prio[jobs_data->constellation];
      break;

    case REACQ_PRIO_COUNT:
    default:
      assert(!"Unsupported re-acq priority mask");
  }

  priority_mask >>= (REACQ_PRIORITY_CYCLE - jobs_data->priority_counter);
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

    case CONSTELLATION_BDS2:
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

/**
 * Check if current constellation is supported and scheduled for reacqusition.
 *
 * \return true  if constellation is supported and scheduled.
 *         false otherwise
 */
bool reacq_scheduled(acq_jobs_state_t *jobs_data) {
  /* TODO: Add logic to select priority level based on tracked GPS count. */
  reacq_prio_level_t prio_level = REACQ_NORMAL_PRIO;

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

  if (CONSTELLATION_GPS == jobs_data->constellation) {
    sm_deep_search_run_gps(jobs_data);
    sm_fallback_search_run_gps(jobs_data, now_ms, lgf_age_ms);
  } else if (CONSTELLATION_GLO == jobs_data->constellation) {
    sm_calc_all_glo_visibility_flags();
    sm_deep_search_run_glo(jobs_data);
    sm_fallback_search_run_glo(jobs_data, now_ms, lgf_age_ms);
  } else if (CONSTELLATION_SBAS == jobs_data->constellation) {
    sm_deep_search_run_sbas(jobs_data);
    sm_fallback_search_run_sbas(jobs_data, now_ms, lgf_age_ms);
  }
}
