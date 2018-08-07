/*
 * Copyright (C) 2014-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/correct_iono_tropo.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/glonass_phase_biases.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/signal.h>
#include <starling/starling.h>
#include <starling/starling_platform.h>

#include "calc_base_obs.h"
#include "calc_pvt_me.h"
#include "calc_starling_obs_array.h"
#include "manage.h"
#include "nav_msg/cnav_msg_storage.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "position/position.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "timing/timing.h"

/** Most recent observations from the base station. */
static obss_t base_obss;

static u8 old_base_sender_id = 0;

/**
 * Uncollapsed observation input type.
 * Remote observations may contain multiple useful signals for satellites.
 * This observation type can fit more observations than the obss_t.
 * Eventually signals in uncollapsed_obss_t are collapsed, and copied to obss_t.
 */
typedef struct {
  /** GPS system time of the observation. */
  gps_time_t tor;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** Observation Solution */
  pvt_engine_result_t soln;

  /** Number of observations in the set. */
  u8 n;
  u8 sender_id;
  /** Set of observations. */
  navigation_measurement_t nm[STARLING_MAX_OBS_COUNT];
} uncollapsed_obss_t;

/* Helper function used for sorting starling observations based on their
 * SID field. */
static int compare_starling_obs_by_sid(const void *a, const void *b) {
  return sid_compare(((starling_obs_t *)a)->sid, ((starling_obs_t *)b)->sid);
}

/* Converter for moving into the intermediary uncollapsed observation type.
 * This performs some preliminary filtering of the observations in addition to
 * populating the navigation measurement fields. */
static void convert_starling_obs_array_to_uncollapsed_obss(
    obs_array_t *obs_array, uncollapsed_obss_t *obss) {
  assert(obs_array);
  assert(obss);
  assert(obs_array->n <= STARLING_MAX_OBS_COUNT);

  obss->sender_id = obs_array->sender;
  obss->tor = obs_array->t;
  obss->has_pos = 0;
  obss->soln.valid = 0;

  /* Selectively populate the navigation measurement array. */
  obss->n = 0;
  for (size_t i = 0; i < obs_array->n; ++i) {
    navigation_measurement_t *nm = &obss->nm[obss->n];
    convert_starling_obs_to_navigation_measurement(&obs_array->observations[i],
                                                   nm);

    /* Filter out any observation without a valid pseudorange observation. */
    if (!pseudorange_valid(*nm)) {
      continue;
    }

    /* Filter out any observation not marked healthy by the SHM. */
    if (shm_navigation_unusable(nm->sid)) {
      continue;
    }

    /* Calculate satellite parameters using the ephemeris. */
    ephemeris_t ephe;
    ndb_op_code_t res = ndb_ephemeris_read(nm->sid, &ephe);
    s8 cscc_ret, css_ret;
    const ephemeris_t *ephe_p = &ephe;

    /* TTFF shortcut: accept also unconfirmed ephemeris candidate when there
     * is no confirmed candidate */
    bool eph_valid = (NDB_ERR_NONE == res || NDB_ERR_UNCONFIRMED_DATA == res) &&
                     ephemeris_valid(&ephe, &nm->tot);

    if (eph_valid) {
      /* Apply corrections to the pseudorange, carrier phase and Doppler. */
      cscc_ret = calc_sat_clock_corrections(1, &nm, &ephe_p);

      /* After correcting the time of transmission for the satellite clock
         error,
         recalculate the satellite position. */
      css_ret = calc_sat_state(&ephe,
                               &nm->tot,
                               nm->sat_pos,
                               nm->sat_vel,
                               nm->sat_acc,
                               &nm->sat_clock_err,
                               &nm->sat_clock_err_rate,
                               &nm->IODC,
                               &nm->IODE);
    }

    if (!eph_valid || (cscc_ret != 0) || (css_ret != 0)) {
      continue;
    }

    /* Only update the output index if we actually passed all the tests. */
    obss->n++;
  }
}

/** Update the #base_obss state given a new set of obss.
 * First sorts by PRN and computes the TDCP Doppler for the observation set. If
 * #base_pos_known is false then a single point position solution is also
 * calculated. Next the `has_pos`, `pos_ecef` and `sat_dists` fields are filled
 * in. Finally the #base_obs_received semaphore is flagged to indicate that new
 * observations are available.
 *
 * \note This function is stateful as it must store the previous observation
 *       set for the TDCP Doppler.
 */
void update_obss(obs_array_t *obs_array) {
  /* Ensure raw observations are sorted by PRN. */
  qsort(obs_array->observations,
        obs_array->n,
        sizeof(obs_array->observations[0]),
        compare_starling_obs_by_sid);

  /* First we need to convert the obs array into this type. */
  uncollapsed_obss_t uncollapsed_obss;
  uncollapsed_obss_t *new_uncollapsed_obss = &uncollapsed_obss;
  convert_starling_obs_array_to_uncollapsed_obss(obs_array,
                                                 new_uncollapsed_obss);

  /* Warn on receiving observations which are very old. This may be indicative
   * of a connectivity problem. Obviously, if we don't have a good local time
   * estimate, then we can't perform this check. */
  gps_time_t now = get_current_time();
  if (get_time_quality() > TIME_UNKNOWN &&
      gpsdifftime(&now, &new_uncollapsed_obss->tor) > BASE_LATENCY_TIMEOUT) {
    log_info("Communication latency exceeds 15 seconds");
  }

  /** Precheck any base station observations and filter if needed. This is not a
   *  permanent solution for actually correcting GPS L2 base station
   *  observations that have mixed tracking modes in a signal epoch. For more
   *  details, see:
   *  https://github.com/swift-nav/estimation_team_planning/issues/215.
   */
  filter_base_meas(&new_uncollapsed_obss->n, new_uncollapsed_obss->nm);

  if (new_uncollapsed_obss->n == 0) {
    log_info("All base obs filtered");
    return;
  }

  /* After collapsing the measurements to Piksi supported signals,
   * only less or equal than MAX_CHANNELS measurements should remain. */
  if (new_uncollapsed_obss->n > MAX_CHANNELS) {
    log_warn("Obs collapsing failure");
  }

  /* Copy contents of new_uncollapsed_obss into new_obss. */
  obss_t obss;
  obss_t *new_obss = &obss;
  new_obss->tor = new_uncollapsed_obss->tor;
  new_obss->has_pos = new_uncollapsed_obss->has_pos;
  new_obss->soln = new_uncollapsed_obss->soln;
  new_obss->n = new_uncollapsed_obss->n;
  new_obss->sender_id = new_uncollapsed_obss->sender_id;
  MEMCPY_S(new_obss->pos_ecef,
           sizeof(new_obss->pos_ecef),
           new_uncollapsed_obss->pos_ecef,
           sizeof(new_uncollapsed_obss->pos_ecef));
  MEMCPY_S(new_obss->nm,
           sizeof(new_obss->nm),
           new_uncollapsed_obss->nm,
           MAX_CHANNELS * sizeof(navigation_measurement_t));

  /* Count distinct satellites */
  gnss_sid_set_t codes;
  sid_set_init(&codes);
  for (u8 i = 0; i < new_obss->n; i++) {
    if (sid_to_constellation(new_obss->nm[i].sid) == CONSTELLATION_GPS) {
      sid_set_add(&codes, new_obss->nm[i].sid);
    }
  }

  /* Require at least 5 distinct satellites */
  if (sid_set_get_sat_count(&codes) >= MIN_SATS_FOR_PVT) {
    bool base_changed = (old_base_sender_id != 0) &&
                        (old_base_sender_id != new_obss->sender_id);
    /* check if we have fix, if yes, calculate iono and tropo correction */
    if (!base_changed && base_obss.has_pos) {
      log_debug("Base: IONO/TROPO correction");
      ionosphere_t i_params;
      /* get iono parameters if available, otherwise use default ones */
      if (ndb_iono_corr_read(&i_params) != NDB_ERR_NONE) {
        i_params = DEFAULT_IONO_PARAMS;
      }
      /* Use the previous ECEF position to get the iono/tropo for the new
       * measurements */
      correct_tropo(base_obss.pos_ecef, new_obss->n, new_obss->nm);
      correct_iono(base_obss.pos_ecef, &i_params, new_obss->n, new_obss->nm);
    }

    gnss_solution soln;
    dops_t dops;

    /* Calculate a position solution. */
    /* disable_raim controlled by external setting (see solution.c). */
    /* Skip velocity solving for the base incase we have bad doppler values
     * due to a cycle slip. */
    s32 ret = calc_PVT(new_obss->n,
                       new_obss->nm,
                       &new_obss->tor,
                       disable_raim,
                       true,
                       GPS_ONLY,
                       &soln,
                       &dops,
                       NULL);

    if (ret >= 0 && soln.valid) {
      /* Copy over the time. */
      base_obss.tor = new_obss->tor;

      base_obss.n = new_obss->n;
      MEMCPY_S(base_obss.nm,
               sizeof(base_obss.nm),
               new_obss->nm,
               new_obss->n * sizeof(navigation_measurement_t));

      MEMCPY_S(base_obss.pos_ecef,
               sizeof(base_obss.pos_ecef),
               soln.pos_ecef,
               sizeof(soln.pos_ecef));

      /* Copy over sender ID. */
      base_obss.sender_id = new_obss->sender_id;

      /* Check if the base sender ID has changed and reset the RTK filter if
       * it has.
       */
      if (base_changed) {
        log_warn(
            "Base station sender ID changed from %u to %u. Resetting RTK"
            " filter.",
            old_base_sender_id,
            base_obss.sender_id);
        starling_reset_rtk_filter();
      }
      old_base_sender_id = base_obss.sender_id;

      base_obss.has_pos = 1;

      obss_t *new_base_obs = platform_mailbox_item_alloc(MB_ID_BASE_OBS);
      if (new_base_obs == NULL) {
        log_warn(
            "Base obs pool full, discarding base obs at: wn: %d, tow: %.2f",
            base_obss.tor.wn,
            base_obss.tor.tow);
        return;
      }

      *new_base_obs = base_obss;

      const errno_t post_ret =
          platform_mailbox_post(MB_ID_BASE_OBS, new_base_obs, MB_NONBLOCKING);
      if (post_ret != 0) {
        log_error("Base obs mailbox should have space!");
        platform_mailbox_item_free(MB_ID_BASE_OBS, new_base_obs);
      }
    } else {
      base_obss.has_pos = 0;
      /* TODO(dsk) check for repair failure */
      /* There was an error calculating the position solution. */
      log_warn("Error calculating base station position: (%s).",
               pvt_err_msg[-ret - 1]);
    }
  }
}
