/*
 * Copyright (C) 2014,2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <string.h>

#include <pvt_engine/propagate.h>
#include <starling/observation.h>
#include <stdbool.h>
#include <swiftnav/constants.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/set.h>
#include <swiftnav/signal.h>

/** \defgroup single_diff Single Difference Observations
 * Functions for storing and manipulating single difference observations.
 * \{ */

/** Comparison function for `sdiff_t` by PRN.
 * See `cmp_fn`. */
int cmp_sdiff(const void *a_, const void *b_) {
  const sdiff_t *a = (const sdiff_t *)a_;
  const sdiff_t *b = (const sdiff_t *)b_;

  return sid_compare(a->sid, b->sid);
}

int cmp_sdiff_sid(const void *a_, const void *b_) {
  const sdiff_t *a = (const sdiff_t *)a_;
  const gnss_signal_t *b = (const gnss_signal_t *)b_;
  return sid_compare(a->sid, *b);
}

/** Create a single difference from two observations.
 * Used by single_diff() to map two `navigation_measurement_t`s
 * into an `sdiff_t`.
 *
 * C/N0 in the output is the lesser of the C/N0 of inputs a and b.
 *
 * `sat_pos` and `sat_vel` are taken from input b.
 *
 * Called once for each pair of observations with matching PRNs.
 */
static void single_diff_(void *context, const u32 n, const void *a,
                         const void *b) {
  const navigation_measurement_t *m_a = (const navigation_measurement_t *)a;
  const navigation_measurement_t *m_b = (const navigation_measurement_t *)b;
  sdiff_t *sds = (sdiff_t *)context;

  sds[n].sid = m_a->sid;
  sds[n].pseudorange = m_a->raw_pseudorange - m_b->raw_pseudorange;
  sds[n].carrier_phase = m_a->raw_carrier_phase - m_b->raw_carrier_phase;
  sds[n].measured_doppler =
      m_a->raw_measured_doppler - m_b->raw_measured_doppler;
  sds[n].computed_doppler =
      m_a->raw_computed_doppler - m_b->raw_computed_doppler;
  sds[n].rover_lock_time = m_a->lock_time;
  sds[n].base_lock_time = m_b->lock_time;
  sds[n].flags = m_a->flags & m_b->flags;

  /* Need to check C/N0 valid flags. */
  if ((0 != (m_a->flags & NAV_MEAS_FLAG_CN0_VALID)) &&
      (0 != (m_b->flags & NAV_MEAS_FLAG_CN0_VALID))) {
    /* A and B are valid, return MIN() */
    sds[n].cn0 = MIN(m_a->cn0, m_b->cn0);
    sds[n].flags |= NAV_MEAS_FLAG_CN0_VALID;
  } else if (0 != (m_a->flags & NAV_MEAS_FLAG_CN0_VALID)) {
    /* A only valid, return A */
    sds[n].cn0 = m_a->cn0;
    sds[n].flags |= NAV_MEAS_FLAG_CN0_VALID;
  } else if (0 != (m_b->flags & NAV_MEAS_FLAG_CN0_VALID)) {
    /* B only valid, return B */
    sds[n].cn0 = m_b->cn0;
    sds[n].flags |= NAV_MEAS_FLAG_CN0_VALID;
  } else {
    /* Neither valid */
    sds[n].cn0 = 0.0;
    sds[n].flags &= ~NAV_MEAS_FLAG_CN0_VALID;
  }

  /* NOTE: We use the position and velocity from B (this is required by
   * make_propagated_sdiffs(). */
  memcpy(&(sds[n].sat_pos), &(m_b->sat_pos), 3 * sizeof(double));
  memcpy(&(sds[n].sat_vel), &(m_b->sat_vel), 3 * sizeof(double));
}

/** Calculate single differences from two sets of observations.
 * Undifferenced input observations are assumed to be both taken at the
 * same time, `t`.
 *
 * C/N0 in the output is the lesser of the C/N0s of inputs a and b.
 *
 * `sat_pos` and `sat_vel` are taken from input b.
 *
 * \param n_a Number of measurements in set `m_a`
 * \param m_a Array of undifferenced observations, as a set sorted by PRN
 * \param n_b Number of measurements in set `m_b`
 * \param m_b Array of undifferenced observations, as a set sorted by PRN
 * \param sds Single difference observations
 *
 * \return The number of observations written to `sds` on success,
 *         -1 if `m_a` is not a valid set,
 *         -2 if `m_b` is not a valid set
 */
u8 single_diff(const u8 n_a, const navigation_measurement_t *m_a, const u8 n_b,
               const navigation_measurement_t *m_b, sdiff_t *sds) {
  return intersection_map(n_a, sizeof(navigation_measurement_t), m_a, n_b,
                          sizeof(navigation_measurement_t), m_b, nav_meas_cmp,
                          sds, single_diff_);
}

typedef struct {
  sdiff_t *sds;
  double *remote_pos_ecef;
} make_propagated_sdiff_ctxt;

int cmp_sid_sdiff(const void *a, const void *b) {
  return sid_compare(*(gnss_signal_t *)a, ((sdiff_t *)b)->sid);
}

/** Propagates remote measurements to a local time and makes sdiffs.
 * When we get two sets of observations that aren't time matched to each
 * other (but are internally time matched within each set), we need to
 * adjust one set of measurements to be our best guess of what it would have
 * been had we measured it at the other set's time. This function does that
 * and differences those measurements from sats present in both sets.
 *
 * It returns the number of sats common in both.
 *
 * \remark This is actually using the sat positions at the time the receiver
 *         got the signal. You can backtrack to the sat position at the time
 *         the signal was sent. At the very least, if you backtrack assuming
 *         sat constant velocity, the difference is miniscule.
 *
 * \todo  Integrate this with single_diff via a higher order function.
 *
 * \param n_local           The number of measurements taken locally.
 * \param m_local           The measurements taken locally (sorted by prn).
 * \param n_remote          The number of measurements taken remotely.
 * \param m_remote          THe measurements taken remotely (sorted by prn).
 * \param remote_pos_ecef   The position of the remote receiver (presumed
 *                           constant in ecef).
 * \param sds               The single differenced propagated measurements.
 * \return The number of sats common in both local and remote sdiffs.
 */
u8 make_propagated_sdiffs(const u8 n_local,
                          const navigation_measurement_t *m_local,
                          const u8 n_remote,
                          const navigation_measurement_t *m_remote,
                          const double remote_pos_ecef[3], sdiff_t *sds) {
  u8 i, j, n = 0;

  /* Loop over m_a and m_b and check if a PRN is present in both. */
  for (i = 0, j = 0; i < n_local && j < n_remote; i++, j++) {
    if (sid_compare(m_local[i].sid, m_remote[j].sid) < 0) {
      j--;
    } else if (sid_compare(m_local[i].sid, m_remote[j].sid) > 0) {
      i--;
    } else {
      sds[n].sid = m_local[i].sid;
      /* The expected "dist" at the new, propagated time contains the distance
         from the base position to the satellite position as well as the
         satellite clock error. */
      double new_dist = nominal_pseudorange(m_local[i].sat_pos, remote_pos_ecef,
                                            m_local[i].sat_clock_err);
      double new_dist_dot =
          nominal_doppler(m_local[i].sat_vel, m_local[i].sat_pos,
                          remote_pos_ecef, m_local[i].sat_clock_err_rate);

      double remote_dist = nominal_pseudorange(
          m_remote[j].sat_pos, remote_pos_ecef, m_remote[j].sat_clock_err);
      double remote_dist_dot =
          nominal_doppler(m_remote[j].sat_vel, m_remote[j].sat_pos,
                          remote_pos_ecef, m_remote[j].sat_clock_err_rate);

      double dist_diff = new_dist - remote_dist;
      double dist_diff_dot = new_dist_dot - remote_dist_dot;
      /* Explanation:
       * pseudorange = dist + c
       * To update a pseudorange in time:
       *  new_pseudorange = new_dist + c
       *                  = old_dist + c + (new_dist - old_dist)
       *                  = old_pseudorange + (new_dist - old_dist)
       *
       * So to get the single differenced pseudorange:
       *  local_pseudorange - new_remote_pseudorange
       *    = local_pseudorange - (old_remote_pseudorange + new_dist - old_dist)
       */
      sds[n].pseudorange = m_local[i].raw_pseudorange -
                           (m_remote[j].raw_pseudorange + dist_diff);

      sds[n].carrier_phase = m_local[i].raw_carrier_phase -
                             (m_remote[j].raw_carrier_phase +
                              dist_diff / sid_to_lambda(m_local[i].sid));

      sds[n].computed_doppler = m_local[i].raw_computed_doppler -
                                (m_remote[j].raw_computed_doppler +
                                 dist_diff_dot / sid_to_lambda(m_local[i].sid));

      sds[n].measured_doppler = m_local[i].raw_measured_doppler -
                                (m_remote[j].raw_measured_doppler +
                                 dist_diff_dot / sid_to_lambda(m_local[i].sid));

      memcpy(sds[n].sat_pos, m_local[i].sat_pos, sizeof(m_local[i].sat_pos));
      memcpy(sds[n].sat_vel, m_local[i].sat_vel, sizeof(m_local[i].sat_vel));
      sds[n].rover_lock_time = m_local[i].lock_time;
      sds[n].base_lock_time = m_remote[j].lock_time;
      sds[n].flags = m_local[i].flags & m_remote[j].flags;

      /* Need to check C/N0 valid flags. */
      if ((0 != (m_local[i].flags & NAV_MEAS_FLAG_CN0_VALID)) &&
          (0 != (m_remote[j].flags & NAV_MEAS_FLAG_CN0_VALID))) {
        /* Local and remote are valid, return MIN() */
        sds[n].cn0 = MIN(m_local[i].cn0, m_remote[j].cn0);
        sds[n].flags |= NAV_MEAS_FLAG_CN0_VALID;
      } else if (0 != (m_local[i].flags & NAV_MEAS_FLAG_CN0_VALID)) {
        /* Local only valid, return local */
        sds[n].cn0 = m_local[i].cn0;
        sds[n].flags |= NAV_MEAS_FLAG_CN0_VALID;
      } else if (0 != (m_remote[j].flags & NAV_MEAS_FLAG_CN0_VALID)) {
        /* Remote only valid, return remote */
        sds[n].cn0 = m_remote[j].cn0;
        sds[n].flags |= NAV_MEAS_FLAG_CN0_VALID;
      } else {
        /* Neither valid */
        sds[n].cn0 = 0.0;
        sds[n].flags &= ~NAV_MEAS_FLAG_CN0_VALID;
      }

      n++;
    }
  }

  return n;
}

/** Prints an sdiff_t
 * \param sd    the sdiff_t to print.
 */
void debug_sdiff(sdiff_t sd) {
  log_debug(
      "sdiff_t:"
      "\tprn = %u\n"
      "\tcn0 = %f\n"
      "\trover_lock_time = %f\n"
      "\tbase_lock_time = %f\n"
      "\tpseudorange      = %f\n"
      "\tcarrier_phase    = %f\n"
      "\tmeasured_doppler = %f\n"
      "\tcomputed_doppler = %f\n"
      "\tsat_pos = [%f, %f, %f]\n"
      "\tsat_vel = [%f, %f, %f]\n",
      sd.sid.sat, sd.cn0, sd.rover_lock_time, sd.base_lock_time, sd.pseudorange,
      sd.carrier_phase, sd.measured_doppler, sd.computed_doppler, sd.sat_pos[0],
      sd.sat_pos[1], sd.sat_pos[2], sd.sat_vel[0], sd.sat_vel[1],
      sd.sat_vel[2]);
}

/** Prints an array of sdiffs
 * \param n     The number of sdiffs to print.
 * \param sds   A pointer to the head of the array of sdiffs to print.
 */
void debug_sdiffs(u8 n, sdiff_t *sds) {
  log_debug("[");
  for (u8 i = 0; i < n; i++) {
    debug_sdiff(sds[i]);
  }
  log_debug("]");
}

static inline bool supported_sid(navigation_measurement_t a) {
  switch ((s8)a.sid.code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L2CM:
    case CODE_GLO_L1OF:
    case CODE_GLO_L2OF:
    case CODE_GPS_L2P:
    case CODE_BDS2_B1:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E7I:
    case CODE_GPS_L5I:
    case CODE_GAL_E5I:
      return true;

    default:
      return false;
  }
}

static inline bool starling_supported_sid(starling_obs_t *a) {
  s8 code = a->sid.code;
  switch (code) {
    case CODE_GPS_L1CA:
    case CODE_GPS_L1P:
    case CODE_GPS_L2CM:
    case CODE_GPS_L2P:
    case CODE_GPS_L5I:
    case CODE_GLO_L1OF:
    case CODE_GLO_L1P:
    case CODE_GLO_L2OF:
    case CODE_GLO_L2P:
    case CODE_BDS2_B1:
    case CODE_BDS2_B2:
    case CODE_GAL_E1B:
    case CODE_GAL_E7I:
    case CODE_GAL_E5I:
      return true;

    default:
      return false;
  }
}

/** Check if an array of navigation measurements has mixed GPS L2 tracking
 * codes.
 *
 * \remark has_mixed_l2_obs is related to handling of L2P and L2CM observations
 *         here:
 *         https://github.com/swift-nav/estimation_team_planning/issues/215. In
 *         the future, we'll want to correct/canonicalize L2P carrier phases
 *         instead of check and filter them.
 *
 * \param  n          The number of navigation measurements.
 * \param  nav_meas   Pointer to array of navigation measurements.
 * \return true if both CODE_GPS_L2CM and CODE_GPS_L2P are found, false
 *         otherwise.
 */
bool has_mixed_l2_obs(u8 n, navigation_measurement_t *nav_meas) {
  assert(nav_meas != NULL);
  bool has_l2c = false;
  bool has_l2p = false;
  for (u8 i = 0; i < n; ++i) {
    switch ((s8)nav_meas[i].sid.code) {
      case CODE_GPS_L2CM:
        has_l2c = true;
        break;
      case CODE_GPS_L2P:
        has_l2p = true;
        break;
      case CODE_INVALID:
      case CODE_COUNT:
        assert(!"Invalid code.");
        break;
      default:
        break;
    }
    if (has_l2c && has_l2p) {
      return true;
    }
  }
  return false;
}

/** Contains the mapping of codes into Piksi supported codes.
 *
 * Primary codes are the ones supported by Piksi.
 * Primary codes are mapped to themselves.
 *
 * Secondary codes are the codes that can be found from remote observations.
 * Secondary codes have a mapping to the corresponding primary code.
 *
 */
/* clang-format off */
static const code_t code_replacement_table[CODE_COUNT] = {
  [CODE_GPS_L1CA] = CODE_GPS_L1CA,
  [CODE_GPS_L2CM] = CODE_GPS_L2CM,
  [CODE_SBAS_L1CA] = CODE_SBAS_L1CA,
  [CODE_GLO_L1OF] = CODE_GLO_L1OF,
  [CODE_GLO_L2OF] = CODE_GLO_L2OF,
  [CODE_GPS_L1P] = CODE_GPS_L1P,
  [CODE_GPS_L2P] = CODE_GPS_L2P,
  [CODE_GPS_L2CL] = CODE_GPS_L2CM,
  [CODE_GPS_L2CX] = CODE_GPS_L2CM,
  [CODE_GPS_L5I] = CODE_GPS_L5I,
  [CODE_GPS_L5Q] = CODE_GPS_L5I,
  [CODE_GPS_L5X] = CODE_GPS_L5I,
  [CODE_BDS2_B1] = CODE_BDS2_B1,
  [CODE_BDS2_B2] = CODE_BDS2_B2,
  [CODE_GAL_E1B] = CODE_GAL_E1B,
  [CODE_GAL_E1C] = CODE_GAL_E1B,
  [CODE_GAL_E1X] = CODE_GAL_E1B,
  [CODE_GAL_E6B] = CODE_GAL_E6B,
  [CODE_GAL_E6C] = CODE_GAL_E6B,
  [CODE_GAL_E6X] = CODE_GAL_E6B,
  [CODE_GAL_E7I] = CODE_GAL_E7I,
  [CODE_GAL_E7Q] = CODE_GAL_E7I,
  [CODE_GAL_E7X] = CODE_GAL_E7I,
  [CODE_GAL_E8I] = CODE_GAL_E8I,
  [CODE_GAL_E8Q] = CODE_GAL_E8I,
  [CODE_GAL_E8X] = CODE_GAL_E8I,
  [CODE_GAL_E5I] = CODE_GAL_E5I,
  [CODE_GAL_E5Q] = CODE_GAL_E5I,
  [CODE_GAL_E5X] = CODE_GAL_E5I,
  [CODE_GLO_L1P] = CODE_GLO_L1P,
  [CODE_GLO_L2P] = CODE_GLO_L2P,
  [CODE_QZS_L1CA] = CODE_QZS_L1CA,
  [CODE_QZS_L1CI] = CODE_QZS_L1CI,
  [CODE_QZS_L1CQ] = CODE_QZS_L1CI,
  [CODE_QZS_L1CX] = CODE_QZS_L1CI,
  [CODE_QZS_L2CM] = CODE_QZS_L2CM,
  [CODE_QZS_L2CL] = CODE_QZS_L2CM,
  [CODE_QZS_L2CX] = CODE_QZS_L2CM,
  [CODE_QZS_L5I] = CODE_QZS_L5I,
  [CODE_QZS_L5Q] = CODE_QZS_L5I,
  [CODE_QZS_L5X] = CODE_QZS_L5I,
  [CODE_SBAS_L5I] = CODE_SBAS_L5I,
  [CODE_SBAS_L5Q] = CODE_SBAS_L5I,
  [CODE_SBAS_L5X] = CODE_SBAS_L5I,
  [CODE_BDS3_B1CI] = CODE_BDS3_B1CI,
  [CODE_BDS3_B1CQ] = CODE_BDS3_B1CI,
  [CODE_BDS3_B1CX] = CODE_BDS3_B1CI,
  [CODE_BDS3_B5I] = CODE_BDS3_B5I,
  [CODE_BDS3_B5Q] = CODE_BDS3_B5I,
  [CODE_BDS3_B5X] = CODE_BDS3_B5I,
  [CODE_BDS3_B7I] = CODE_BDS3_B7I,
  [CODE_BDS3_B7Q] = CODE_BDS3_B7I,
  [CODE_BDS3_B7X] = CODE_BDS3_B7I,
  [CODE_BDS3_B3I] = CODE_BDS3_B3I,
  [CODE_BDS3_B3Q] = CODE_BDS3_B3I,
  [CODE_BDS3_B3X] = CODE_BDS3_B3I,
  [CODE_GPS_L1CI] = CODE_GPS_L1CI,
  [CODE_GPS_L1CQ] = CODE_GPS_L1CI,
  [CODE_GPS_L1CX] = CODE_GPS_L1CI,
  [CODE_AUX_GPS] = CODE_AUX_GPS,
  [CODE_AUX_SBAS] = CODE_AUX_SBAS,
  [CODE_AUX_GAL] = CODE_AUX_GAL,
  [CODE_AUX_QZS] = CODE_AUX_QZS,
  [CODE_AUX_BDS] = CODE_AUX_BDS,
};
/* clang-format on */

code_t to_supported_code_t(const code_t code) {
  return code_replacement_table[code];
}

/** Contains bit masks for secondary codes.
 * See the note above for `code_replacement_table`.
 *
 * Primary codes are never masked.
 * If primary code is found in the remote observations,
 * all corresponding secondary codes are masked.
 *
 * If a secondary code is found in the remote observations,
 * a mask is applied so that only one secondary code remains in the end.
 *
 */
static const u64 code_mask_table[CODE_COUNT] = {
    [CODE_GPS_L1CA] = (1ull << CODE_GPS_L1P),
    [CODE_GPS_L2CM] = (1ull << CODE_GPS_L2CL | 1ull << CODE_GPS_L2CX),
    [CODE_GPS_L2P] = (1ull << CODE_GPS_L2CL | 1ull << CODE_GPS_L2CX),
    [CODE_GPS_L2CL] = (1ull << CODE_GPS_L2CX),
    [CODE_GPS_L5I] = (1ull << CODE_GPS_L5Q | 1ull << CODE_GPS_L5X),
    [CODE_GPS_L5Q] = (1ull << CODE_GPS_L5X),
    [CODE_GAL_E1B] = (1ull << CODE_GAL_E1C | 1ull << CODE_GAL_E1X),
    [CODE_GAL_E1C] = (1ull << CODE_GAL_E1X),
    [CODE_GAL_E6B] = (1ull << CODE_GAL_E6C | 1ull << CODE_GAL_E6X),
    [CODE_GAL_E6C] = (1ull << CODE_GAL_E6X),
    [CODE_GAL_E7I] = (1ull << CODE_GAL_E7Q | 1ull << CODE_GAL_E7X),
    [CODE_GAL_E7Q] = (1ull << CODE_GAL_E7X),
    [CODE_GAL_E5I] = (1ull << CODE_GAL_E5Q | 1ull << CODE_GAL_E5X),
    [CODE_GAL_E5Q] = (1ull << CODE_GAL_E5X),
    [CODE_GLO_L1OF] = (1ull << CODE_GLO_L1P),
    [CODE_GLO_L2OF] = (1ull << CODE_GLO_L2P),
    [CODE_QZS_L2CM] = (1ull << CODE_QZS_L2CL | 1ull << CODE_QZS_L2CX),
    [CODE_QZS_L2CL] = (1ull << CODE_QZS_L2CX),
    [CODE_QZS_L5I] = (1ull << CODE_QZS_L5Q | 1ull << CODE_QZS_L5X),
    [CODE_QZS_L5Q] = (1ull << CODE_QZS_L5X),
};

bool is_code_in_mask(const u64 masked_codes, const code_t code) {
  return masked_codes & (1ull << code);
}

/** Replace secondary codes in remote observations with primary codes
 *
 * \param n_channels Number of tracking channel measurements
 * \param nav_meas Pointer to array of navigation measurements.
 * \param masked_codes Bit mask indicating codes which should be replaced.
 */
static void replace_navmeas_sid(u8 n, navigation_measurement_t *nav_meas,
                                u64 masked_codes) {
  for (u8 i = 0; i < n; ++i) {
    code_t code = nav_meas[i].sid.code;
    if (is_code_in_mask(masked_codes, code)) {
      nav_meas[i].sid.code = to_supported_code_t(code);
    }
  }
}

/** Apply code mask for secondary codes in remote observations.
 *
 * \param found_codes Bit mask containing all available codes in remote
 * obsevations.
 * \return Bit mask where secondary codes have been marked for code replacement.
 */
u64 mask_secondary_codes(u64 found_codes) {
  for (u8 i = 0; i < CODE_COUNT; ++i) {
    if (found_codes & (1ull << i)) {
      found_codes &= ~(code_mask_table[i]);
    }
  }
  return found_codes;
}

u64 mark_found_code(const u64 found_codes, const code_t code) {
  return (found_codes | (1ull << code));
}

/** Check if an array of navigation measurements has measurements
 *  which need to be mapped to Piksi supported codes.
 *
 * \param  n          The number of navigation measurements.
 * \param  nav_meas   Pointer to array of navigation measurements.
 */
void collapse_navmeas(u8 n, navigation_measurement_t *nav_meas) {
  assert(nav_meas != NULL);
  /* Loop through the measurements and mark all contained codes. */
  u64 found_codes = 0;
  for (u8 i = 0; i < n; ++i) {
    found_codes = mark_found_code(found_codes, nav_meas[i].sid.code);
  }

  /* Mask secondary codes.
   * Only one code should end up getting mapped to a primary code. */
  u64 masked_codes = mask_secondary_codes(found_codes);

  /* Make the mapping to primary codes supported by Piksi. */
  replace_navmeas_sid(n, nav_meas, masked_codes);
}

/* This guy takes two arguments; the first is the measurement to test
 * and the second is a unary predicate for selection of elements.
 * This is used in `filter_nav_meas()` so that we can use the same
 * implementation of the filtering algorithm whether or not the user
 * wants to pass in some extra data to the selection predicate.  We
 * simply call `filter_nav_meas_extra()` with this function as the
 * selection predicate and pass in the user function as the
 * `extra_data`.
 */
static bool no_argument_wrapper(navigation_measurement_t meas,
                                void *extra_data) {
  navigation_measurement_predicate_f predicate =
      (navigation_measurement_predicate_f)extra_data;
  return predicate(meas);
}

/* This guy takes two arguments; the first is the measurement to test
 * and the second is a unary predicate for selection of elements.
 * This is used in `filter_nav_meas()` so that we can use the same
 * implementation of the filtering algorithm whether or not the user
 * wants to pass in some extra data to the selection predicate.  We
 * simply call `filter_nav_meas_extra()` with this function as the
 * selection predicate and pass in the user function as the
 * `extra_data`.
 */
static bool starling_no_argument_wrapper(starling_obs_t *meas,
                                         void *extra_data) {
  starling_obs_predicate_f predicate = (starling_obs_predicate_f)extra_data;
  return predicate(meas);
}

bool not_gps_l2cm_sid(navigation_measurement_t a) {
  return !(a.sid.code == CODE_GPS_L2CM);
}

bool only_gps_l2cm_sid(navigation_measurement_t a) {
  return a.sid.code == CODE_GPS_L2CM;
}

static bool not_gps_l2p_sid(navigation_measurement_t a) {
  return a.sid.code != CODE_GPS_L2P;
}

/**/

static bool pred_extra_not_code(starling_obs_t *a, void *arg) {
  code_t *code = (code_t *)arg;
  return a->sid.code != (*code);
}

static bool pred_has_pseudorange(starling_obs_t *a) {
  return (0 != ((a->flags) & NAV_MEAS_FLAG_CODE_VALID));
}

void filter_navmeas(u8 *n, navigation_measurement_t nav_meas[],
                    bool prefer_l2c) {
  /* Map codes to Piksi supported codes. */
  collapse_navmeas(*n, nav_meas);

  /* These functions assume that collapse_navmeas() mapped
   * CODE_GPS_L2CL and CODE_GPS_L2CX to CODE_GPS_L2CM */
  if (*n > 0 && has_mixed_l2_obs(*n, nav_meas)) {
    if (prefer_l2c) {
      log_info(
          "Base observations have mixed GPS L2 tracking types. Discarding "
          "L2P!");
      *n = filter_nav_meas(*n, nav_meas, not_gps_l2p_sid);
    } else {
      log_debug(
          "Base observations have mixed GPS L2 tracking types. Discarding "
          "L2C!");
      *n = filter_nav_meas(*n, nav_meas, not_gps_l2cm_sid);
    }
  }

  /* Drop any codes not supported by Starling */
  *n = filter_nav_meas(*n, nav_meas, supported_sid);
  /* Filter out any observation without a valid pseudorange observation. */
  if (*n > 0) {
    *n = filter_nav_meas(*n, nav_meas, pseudorange_valid);
  }
}

/* Loop through the measurements and mark all contained codes. */
static void map_codes_into_known_ones(obs_array_t *obsa) {
  for (u8 i = 0; i < obsa->n; i++) {
    code_t code = obsa->observations[i].sid.code;
    if (code < CODE_COUNT && CODE_INVALID != code) {
      obsa->observations[i].sid.code = code_replacement_table[code];
    } else {
      obsa->observations[i].sid.code = CODE_INVALID;
    }
  }
}

/** Check how many observations appear with a given code */
static u8 num_codes_in_epoch(obs_array_t *obsa, code_t code) {
  u8 num = 0;
  for (u8 i = 0; i < obsa->n; i++) {
    code_t c = obsa->observations[i].sid.code;
    if (c == code) num++;
  }
  return num;
}

static void filter_obs_c_p_helper(obs_array_t *obsa, code_t c, code_t p) {
  u8 num_c = num_codes_in_epoch(obsa, c);
  u8 num_p = num_codes_in_epoch(obsa, p);
  if (num_c && num_p) {
    DO_EVERY(64, log_info("Epoch has %d %s and %d %s, promoting the majority",
                          num_c, code_to_string(c), num_p, code_to_string(p)));
    if (num_c >= num_p) {
      (obsa->n) = filter_starling_obs_extra(obsa, pred_extra_not_code, &p);
    } else {
      (obsa->n) = filter_starling_obs_extra(obsa, pred_extra_not_code, &c);
    }
  }
}

/* Apply filtering and collapse the input observation array */
void filter_obs_array(obs_array_t *obsa) {
  assert(obsa);
  if (0 == (obsa->n)) return;

  /* This function does not discard any measurements,
   * just groups equivalent codes together */
  map_codes_into_known_ones(obsa);

  /* Now discard either C or P measurements */
  filter_obs_c_p_helper(obsa, CODE_GPS_L1CA, CODE_GPS_L1P);
  filter_obs_c_p_helper(obsa, CODE_GPS_L2CM, CODE_GPS_L2P);
  filter_obs_c_p_helper(obsa, CODE_GLO_L1OF, CODE_GLO_L1P);
  filter_obs_c_p_helper(obsa, CODE_GLO_L2OF, CODE_GLO_L2P);

  /* Also discard obs with no pseudorange */
  obsa->n = filter_starling_obs(obsa, pred_has_pseudorange);

  /* Now drop any codes not supported by Starling */
  obsa->n = filter_starling_obs(obsa, starling_supported_sid);
}

u8 filter_nav_meas(u8 n, navigation_measurement_t nav_meas[],
                   navigation_measurement_predicate_f predicate) {
  /* See the note above `no_argument_wrapper()`. */
  return filter_nav_meas_extra(n, nav_meas, no_argument_wrapper, predicate);
}

u8 filter_nav_meas_extra(u8 n, navigation_measurement_t nav_meas[],
                         navigation_measurement_predicate_extra_f predicate,
                         void *extra_data) {
  /* Identify the indices to drop. */
  u16 idxs[256 + 1];
  u16 idx_count = 0;
  for (u8 i = 0; i < n; i++) {
    /* If the selection function returns false, away with it! */
    if (!predicate(nav_meas[i], extra_data)) {
      idxs[idx_count++] = i;
    }
  }

  /* Add an end-marker so our use of `i + 1` can do the right thing. */
  idxs[idx_count] = n;

  /* Drop all the marked indices. */
  for (u16 n_dropped = 0; n_dropped < idx_count; n_dropped++) {
    const u16 to_drop = idxs[n_dropped];
    const u16 next_drop = idxs[n_dropped + 1];
    /* Move some elements: */
    memmove(/* _To_ the current index to drop minus the number of
             * elements already dropped.  This should just be the
             * index when we drop the first element. */
            &nav_meas[to_drop - n_dropped],
            /* _From_ the next index to keep.  If the next index to
             * drop is the next element in the array, then this source
             * doesn't matter, because the size passed to `memmove()`
             * will be 0 and we won't move anything. */
            &nav_meas[to_drop + 1],
            /* _Move_ the number of elements in between this index and
             * the next index.  This should be 0 if our indices to
             * drop are consecutive.  `idxs` gets `n` appended to it
             * above, so `next_drop` will be `n` when `to_drop` is `n
             * - 1` (in the case where we drop the final element). */
            (next_drop - to_drop - 1) * sizeof(navigation_measurement_t));
  }

  /* Zero the empty space at the end of the array. */
  memset(nav_meas + n - idx_count, 0,
         idx_count * sizeof(navigation_measurement_t));
  return n - idx_count;
}

u8 filter_starling_obs(obs_array_t *obsa, starling_obs_predicate_f predicate) {
  /* See the note above `no_argument_wrapper()`. */
  return filter_starling_obs_extra(obsa, starling_no_argument_wrapper,
                                   predicate);
}

u8 filter_starling_obs_extra(obs_array_t *obsa,
                             starling_obs_predicate_extra_f predicate,
                             void *extra_data) {
  u8 n = obsa->n;
  if (0 == n) return 0;

  /* Identify the indices to drop. */
  u16 idxs[256 + 1];
  u16 idx_count = 0;
  for (u8 i = 0; i < n; i++) {
    /* If the selection function returns false, away with it! */
    if (!predicate(&(obsa->observations[i]), extra_data)) {
      idxs[idx_count++] = i;
    }
  }

  /* Add an end-marker so our use of `i + 1` can do the right thing. */
  idxs[idx_count] = n;

  /* Drop all the marked indices. */
  for (u16 n_dropped = 0; n_dropped < idx_count; n_dropped++) {
    const u16 to_drop = idxs[n_dropped];
    const u16 next_drop = idxs[n_dropped + 1];
    /* Move some elements: */
    memmove(/* _To_ the current index to drop minus the number of
             * elements already dropped.  This should just be the
             * index when we drop the first element. */
            &obsa->observations[to_drop - n_dropped],
            /* _From_ the next index to keep.  If the next index to
             * drop is the next element in the array, then this source
             * doesn't matter, because the size passed to `memmove()`
             * will be 0 and we won't move anything. */
            &obsa->observations[to_drop + 1],
            /* _Move_ the number of elements in between this index and
             * the next index.  This should be 0 if our indices to
             * drop are consecutive.  `idxs` gets `n` appended to it
             * above, so `next_drop` will be `n` when `to_drop` is `n
             * - 1` (in the case where we drop the final element). */
            (next_drop - to_drop - 1) * sizeof(starling_obs_t));
  }

  /* Zero the empty space at the end of the array. */
  memset(&obsa->observations[n - idx_count], 0,
         idx_count * sizeof(starling_obs_t));
  return n - idx_count;
}

/** Correct observations with satellite clock parameters
 *
 * \param n_channels Number of tracking channel measurements
 * \param nav_meas Array of pointers of where to store the output observations,
 *                 length `n_channels`
 */
void apply_sat_clock_corrections(u8 n_channels,
                                 navigation_measurement_t *nav_meas[]) {
  for (u8 i = 0; i < n_channels; i++) {
    /* Correct the measurements with the satellite clock(rate) errors */
    double carrier_freq = sid_to_carr_freq(nav_meas[i]->sid);
    nav_meas[i]->pseudorange += GPS_C * nav_meas[i]->sat_clock_err;
    nav_meas[i]->carrier_phase += nav_meas[i]->sat_clock_err * carrier_freq;
    nav_meas[i]->measured_doppler +=
        nav_meas[i]->sat_clock_err_rate * carrier_freq;
  }
}

/** \} */
