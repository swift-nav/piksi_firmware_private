/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#define DEBUG true

#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include <swiftnav/glo_map.h>

#include "acq/manage.h"
#include "ndb/ndb_lgf.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "search_manager_api.h"
#include "swiftnap.h"
#include "timing/timing.h"

/**
 * Check if given constellation is supported.
 *
 * \return true  if constellation is supported.
 *         false otherwise
 */
static bool is_constellation_enabled(constellation_t con) {
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

/**
 * The function returns start job index according to gnss
 * \param[in] gnss Constellation
 * \return Start index of GNSS in job array
 */
u16 sm_constellation_to_start_index(constellation_t gnss) {
  switch ((s8)gnss) {
    case CONSTELLATION_GPS:
      return 0;
    case CONSTELLATION_GAL:
      return NUM_SATS_GPS;
    case CONSTELLATION_SBAS:
      return NUM_SATS_GPS + NUM_SATS_GAL;
    case CONSTELLATION_QZS:
      return NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS;
    case CONSTELLATION_BDS:
      return NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS + NUM_SATS_QZS;
    case CONSTELLATION_GLO:
      return NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_SBAS + NUM_SATS_QZS +
      NUM_SATS_BDS;
    default:
      assert(!"Incorrect constellation");
      return 0;
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
  }
  static sbas_system_t sbas_provider = SBAS_NONE;
  sbas_system_t new_provider = sbas_select_provider(&lgf);
  if ((sbas_provider != new_provider) && (SBAS_NONE != sbas_provider)) {
    tracker_set_sbas_provider_change_flag();
  }
  sbas_provider = new_provider;
  return sbas_select_prn_mask(sbas_provider);
}

/** Global search job data */
acq_jobs_context_t acq_all_jobs_state_data;

/* Search manager API functions */

/** Initialize search manager module
 *
 * \param data pointer to global job data
 *
 * \return none
 */
void sm_init(acq_jobs_context_t *data) {
  memset(data, 0, sizeof(acq_jobs_context_t));

  const struct {
    constellation_t gnss;
    u16 first_sv;
    u16 num_sv;
  } reacq_gnss[] = {{CONSTELLATION_GPS, GPS_FIRST_PRN, NUM_SATS_GPS},
                    {CONSTELLATION_GAL, GAL_FIRST_PRN, NUM_SATS_GAL},
                    {CONSTELLATION_SBAS, SBAS_FIRST_PRN, NUM_SATS_SBAS},
                    {CONSTELLATION_QZS, QZS_FIRST_PRN, NUM_SATS_QZS},
                    {CONSTELLATION_GLO, GLO_MIN_FCN, GLO_MAX_FCN},
                    {CONSTELLATION_BDS, BDS_FIRST_PRN, NUM_SATS_BDS}};

  for (u16 k = 0; k < ARRAY_SIZE(reacq_gnss); k++) {
    u16 num_sv = reacq_gnss[k].num_sv;
    constellation_t gnss = reacq_gnss[k].gnss;
    u16 idx = sm_constellation_to_start_index(gnss);

    if (idx + num_sv > REACQ_NUM_SAT) {
      log_error("idx + num_sv < REACQ_NUM_SAT: %d + %d < %d",
                idx,
                num_sv,
                REACQ_NUM_SAT);
      assert(0);
    }

    acq_job_t *job = &data->jobs[idx];
    u16 first_sv = reacq_gnss[k].first_sv;
    code_t code = constellation_to_l1_code(gnss);
    for (u16 i = 0; i < num_sv; i++) {
      job[i].mesid = construct_mesid(code, first_sv + i);
    }
  }
}

/** Categorizes satellites in "visible", "unknown" and "invisible" groups,
 *  and resets their status to "ready for reacq" following the simple concept of
 *  - do all visible until one unknown or invisible is done (note the loop in
 * `sch_run()`)
 *  - reset all visible to ready
 *  - if there are no more unknowns reset all unknowns to ready
 *  - if there are no more invisible reset all invisible to ready
 *
 * \param jobs_data pointer to job data
 * \param last_job_type last job type
 *
 * \return none
 */
void sm_restore_jobs(acq_jobs_context_t *jobs_data,
                     reacq_sched_ret_t last_job_type) {
  assert(jobs_data != NULL);

  /* if the last job was a visible satellite, don't reset any of the reacq
   * states as the scheduler will continue to consume visibles next time */
  if (REACQ_DONE_VISIBLE == last_job_type) {
    return;
  }

  u64 now_ms = timing_getms();

  /* count the number of GPS L1CA signals tracked */
  const u16 num_gps_l1 = code_track_count(CODE_GPS_L1CA);
  /* count the number of SBAS satellites tracked */
  const u16 num_sbas = code_track_count(CODE_SBAS_L1CA);
  /* count the number of GLO satellites tracked */
  const u16 num_glo = code_track_count(CODE_GLO_L1OF);

  u32 sbas_mask = sbas_limit_mask();
  u32 sbas_start_idx = sm_constellation_to_start_index(CONSTELLATION_SBAS);

  for (u16 i = 0; i < REACQ_NUM_SAT; i++) {
    acq_job_t *job = &jobs_data->jobs[i];
    const me_gnss_signal_t mesid = job->mesid;
    assert(mesid_valid(mesid));
    const constellation_t con = code_to_constellation(mesid.code);

    /* constellation disabled */
    if (!is_constellation_enabled(con)) {
      job->state = ACQ_STATE_IDLE;
      continue;
    }

    if (CONSTELLATION_SBAS == con) {
      assert(sbas_start_idx <= i);
      u32 sbas_idx = i - sbas_start_idx;
      /* don't set job for those SBAS SV which are not in our SBAS range,
       * or if we already more than the limit */
      if ((num_sbas > NAP_NUM_SBAS_L1_CHANNELS) ||
          (0 == ((sbas_mask >> sbas_idx) & 1))) {
        job->state = ACQ_STATE_IDLE;
        continue;
      }
    }

    if (CONSTELLATION_GLO == con) {
      if (num_glo >= NAP_NUM_GLO_G1_CHANNELS) {
        job->state = ACQ_STATE_IDLE;
        continue;
      }
    }

    /* if this mesid is in track, no need for its job */
    if (mesid_is_tracked(mesid)) {
      job->state = ACQ_STATE_IDLE;
      continue;
    }
    /* if this mesid can't be tracked, no need for its job */
    if (!tracking_startup_ready(mesid)) {
      job->state = ACQ_STATE_IDLE;
      continue;
    }

    bool known = false;
    bool visible = false;

    sm_get_visibility_flags(mesid, &visible, &known);
    
    /* save the job category into `sky_status` */
    job->sky_status = known ? (visible ? VISIBLE : INVISIBLE) : UNKNOWN;

    if (VISIBLE == job->sky_status) {
      /* should only arrive here once all visibile satellite have been tried */
      if ((num_gps_l1 < LOW_GPS_L1CA_SV_LIMIT) && (CONSTELLATION_GPS == con)) {
        /* if there are less than 6 GPS L1CA don't delay their reacq */
        job->state = ACQ_STATE_WAIT;
      } else if ((now_ms - job->stop_time_ms) >
                 REACQ_MIN_SEARCH_INTERVAL_VISIBLE_MS) {
        /* schedule a job as ready if the min delay elapsed */
        job->state = ACQ_STATE_WAIT;
      }
    } else if ((UNKNOWN == job->sky_status) &&
               ((now_ms - job->stop_time_ms) >
                REACQ_MIN_SEARCH_INTERVAL_UNKNOWN_MS)) {
      /* if the scheduler has done nothing or tried an invisible satellite it
       * means that there were no more unknown satellites to search, so reset
       * their status */
      if (REACQ_DONE_UNKNOWN < last_job_type) {
        job->state = ACQ_STATE_WAIT;
      }
    } else if ((INVISIBLE == job->sky_status) && 
              ((now_ms - job->stop_time_ms) >
               REACQ_MIN_SEARCH_INTERVAL_INVISIBLE_MS)) {
      /* if the scheduler last time has done nothing then there were no ready
       * satellites so it's time to put the invisible sats in the search queue
       * again */
      if (REACQ_DONE_INVISIBLE < last_job_type) {
        job->state = ACQ_STATE_WAIT;
      }
    }
  } /* loop jobs */
}
