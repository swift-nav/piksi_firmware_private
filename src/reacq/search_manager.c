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
#include <inttypes.h>
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
    0b101010101010101010101010101010, /* BDS2 */
    0b010101010101010101010101010101, /* QZSS */
    0b101010101010101010101010101010  /* GAL */
};

/** Re-acq gps high priority masks. */
static const u32 reacq_gps_high_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b000000000000000000000000000001, /* SBAS */
    0b000010000100001000010000100001, /* GLO */
    0b000100001000010000100001000010, /* BDS2 */
    0b001000010000100001000010000100, /* QZSS */
    0b010000100001000010000100001000  /* GAL */
};

/** Re-acq sbas high priority masks. */
static const u32 reacq_sbas_high_prio[] = {
    0b111111111111111111111111111111, /* GPS */
    0b101010101010101010101010101010, /* SBAS */
    0b000010000100001000010000100001, /* GLO */
    0b000100001000010000100001000010, /* BDS2 */
    0b001000010000100001000010000100, /* QZSS */
    0b010000100001000010000100001000  /* GAL */
};

/* Search manager functions which call other modules */

static bool is_constellation_enabled(constellation_t con);

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

  struct init_struct {
    constellation_t gnss;
    u16 first_prn;
  } reacq_gnss[] = {{CONSTELLATION_GPS, GPS_FIRST_PRN},
                    {CONSTELLATION_GLO, GLO_FIRST_PRN},
                    {CONSTELLATION_SBAS, SBAS_FIRST_PRN},
                    {CONSTELLATION_BDS, BDS_FIRST_PRN},
                    {CONSTELLATION_QZS, QZS_FIRST_PRN},
                    {CONSTELLATION_GAL, GAL_FIRST_PRN}};

  for (u32 k = 0; k < ARRAY_SIZE(reacq_gnss); k++) {
    constellation_t gnss = reacq_gnss[k].gnss;
    u16 idx = sm_constellation_to_start_index(gnss);
    u16 num_sv = constellation_to_sat_count(gnss);
    code_t code = constellation_to_l1_code(gnss);

    acq_job_t *job = &data->jobs[idx];
    u16 first_prn = reacq_gnss[k].first_prn;
    for (u32 i = 0; i < num_sv; i++) {
      if (CONSTELLATION_GLO == gnss) {
        /* NOTE: GLO MESID is initialized evenly with all FCNs, so that
         * blind searches are immediately done with whole range of FCNs */
        job[i].mesid = construct_mesid(code, first_prn + (i % GLO_MAX_FCN));
      } else {
        job[i].mesid = construct_mesid(code, first_prn + i);
      }
      job[i].sid = construct_sid(code, first_prn + i);
    }
  }

  /* When constellation is initialized with CONSTELLATION_INVALID,
   * sm_constellation_select() will choose GPS as the first constellation. */
  data->constellation = CONSTELLATION_INVALID;
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
                                   u64 now_ms) {
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
        jobs_data->jobs[idx + i].state = ACQ_STATE_IDLE;
      }
      /* exit to prevent unnecessary reacq */
      return;
    }
  }

  /* check if in this constellation there are visible satellites ready to be acquired */
  for (i = 0; i < num_sv; i++) {
    acq_job_t *waiting_job;
    waiting_job = &jobs_data->jobs[idx + i];
    if ((VISIBLE == waiting_job->sky_status) && (ACQ_STATE_WAIT == waiting_job->state)) {
      /* don't change anything, the scheduler will consume all visible satellites */
      return;
    }
  }

  /* done searching the visible satellites, check if there are unknown satellites ready to be acquired */
  bool any_unknown = false;
  for (i = 0; i < num_sv; i++) {
    acq_job_t *waiting_job;
    waiting_job = &jobs_data->jobs[idx + i];
    if ((UNKNOWN == waiting_job->sky_status) && (ACQ_STATE_WAIT == waiting_job->state)) {
      /* just note that there are more unknown satellites to search */
      any_unknown = true;
      break;
    }
  }

  bool any_invisible = false;
  for (i = 0; i < num_sv; i++) {
    acq_job_t *waiting_job;
    waiting_job = &jobs_data->jobs[idx + i];
    if ((INVISIBLE == waiting_job->sky_status) && (ACQ_STATE_WAIT == waiting_job->state)) {
      /* just note that there are more invisible satellites to search */
      any_invisible = true;
      break;
    }
  }

  for (i = 0; i < num_sv; i++) {
    acq_job_t *fallback_job;
    fallback_job = &jobs_data->jobs[idx + i];

    fallback_job->state = ACQ_STATE_IDLE;

    if ((CONSTELLATION_SBAS == con) && !((sbas_mask >> i) & 1)) {
      /* don't set job for those SBAS SV which are not in our SBAS range */
      continue;
    }

    gnss_signal_t sid = fallback_job->sid;
    assert(sid_valid(sid));

    me_gnss_signal_t *mesid = &fallback_job->mesid;
    if (CONSTELLATION_GLO == con) {
      u16 glo_fcn = GLO_FCN_UNKNOWN;
      if (glo_map_valid(sid)) {
        glo_fcn = glo_map_get_fcn(sid);
        *mesid = construct_mesid(CODE_GLO_L1OF, glo_fcn);
      }
    }
    assert(mesid_valid(*mesid));

    if (mesid_is_tracked(*mesid)) {
      continue;
    }
    if (!tracking_startup_ready(*mesid)) {
      continue;
    }

    bool visible = false;
    bool known = false;
    bool invisible = false;

    sm_get_visibility_flags(sid, &visible, &known);
    visible = visible && known;

    if (CONSTELLATION_SBAS == con) {
      /* sbas_mask SVs are visible as they were selected by location. */
      visible = true;
    }

    fallback_job->sky_status = known ? (visible ? VISIBLE : INVISIBLE) : UNKNOWN;
    if (visible && ((now_ms - fallback_job->stop_time) > REACQ_MIN_SEARCH_INTERVAL_VISIBLE_MS)) {
      /* alwayes reset the visible satellites when the scheduler has searched them */
      fallback_job->state = ACQ_STATE_WAIT;
    } else if ((!known || (CONSTELLATION_GLO == con && !glo_map_valid(sid))) &&
        ((now_ms - fallback_job->stop_time) > REACQ_MIN_SEARCH_INTERVAL_UNKNOWN_MS)) {
      /* reset the state of unknown satellites if they were all searched for */
      if (!any_unknown) fallback_job->state = ACQ_STATE_WAIT;
    } else if (invisible && ((now_ms - fallback_job->stop_time) > REACQ_MIN_SEARCH_INTERVAL_INVISIBLE_MS)) {
      /* reset the state of invisible satellites if they and the unknowns also were all searched for */
      if (!any_unknown && !any_invisible) fallback_job->state = ACQ_STATE_WAIT;
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
  if (CONSTELLATION_GLO == jobs_data->constellation) {
    sm_calc_all_glo_visibility_flags();
  }
  sm_fallback_search_run(jobs_data, now_ms);
}
