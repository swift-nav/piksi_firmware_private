/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <libswiftnav/constants.h>
#include <sbp.h>
#include <sbp_utils.h>
#include <stdint.h>

#include "board/nap/track_channel.h"
#include "board/v3/nap/nap_constants.h"
#include "board/v3/nap/nap_hw.h"
#include "ndb/ndb.h"
#include "shm/shm.h"
#include "timing/timing.h"
#include "track.h"
#include "track_flags.h"
#include "track_sbp.h"

/** Get synchronization status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_sync_status_t flags
 */
static u8 get_sync_flags(const tracking_channel_info_t *channel_info) {
  u8 flags = TRACK_SBP_SYNC_NONE;
  if (0 != (channel_info->flags & TRACKER_FLAG_BIT_SYNC)) {
    flags = TRACK_SBP_SYNC_BIT;
  }
  return flags;
}

/** Get TOW status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_tow_status_t flags
 */
static u8 get_tow_flags(const tracking_channel_info_t *channel_info) {
  u8 flags = TRACK_SBP_TOW_NONE;
  if (0 == (channel_info->flags & TRACKER_FLAG_TOW_VALID)) {
    return flags;
  }
  if (0 == (channel_info->flags & TRACKER_FLAG_TOW_DECODED)) {
    flags = TRACK_SBP_TOW_PROPAGATED;
  } else {
    flags = TRACK_SBP_TOW_DECODED;
  }

  return flags;
}

/** Get tracking loop status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_loop_status_t,
 *         #TRACK_SBP_LOOP_PLL and #TRACK_SBP_LOOP_FLL flags
 */
static u8 get_track_flags(const tracking_channel_info_t *channel_info) {
  u8 flags = TRACK_SBP_LOOP_NO_LOCK;
  if (0 != (channel_info->flags & TRACKER_FLAG_HAS_PLOCK)) {
    flags = TRACK_SBP_LOOP_PLL_PESSIMISTIC_LOCK;

  } else if (0 != (channel_info->flags & TRACKER_FLAG_HAS_FLOCK)) {
    flags = TRACK_SBP_LOOP_FLL_LOCK;
  }
  if (0 != (channel_info->flags & TRACKER_FLAG_PLL_USE)) {
    flags |= TRACK_SBP_LOOP_PLL;
  }
  if (0 != (channel_info->flags & TRACKER_FLAG_FLL_USE)) {
    flags |= TRACK_SBP_LOOP_FLL;
  }
  return flags;
}

/** Get navigation data status flags
 * \param[in] common_data tracking loop common data
 * \param[in] sid signal identifier
 * \return Bit field of #sv_health_status_t,
 *         #TRACK_SBP_NAV_STATE_EPHEMERIS and #TRACK_SBP_NAV_STATE_ALMANAC flags
 */
static u8 get_nav_data_status_flags(gnss_signal_t sid) {
  u8 flags = TRACK_SBP_HEALTH_UNKNOWN;

  if (shm_health_unknown(sid)) {
    flags = TRACK_SBP_HEALTH_UNKNOWN;
  } else if (shm_signal_healthy(sid)) {
    flags = TRACK_SBP_HEALTH_GOOD;
  } else if (shm_signal_unhealthy(sid)) {
    flags = TRACK_SBP_HEALTH_BAD;
  } else {
    assert(!"Unknown nav state");
  }

  if (get_time_quality() == TIME_UNKNOWN) {
    return flags;
  }

  gps_time_t t = get_current_time();
  union {
    ephemeris_t e;
    almanac_t a;
  } orbit;
  enum ndb_op_code ndb_op_code;

  ndb_op_code = ndb_ephemeris_read(sid, &orbit.e);
  if (NDB_ERR_NONE == ndb_op_code) {
    if (ephemeris_valid(&orbit.e, &t)) {
      flags |= TRACK_SBP_NAV_STATE_EPHEMERIS;
    }
  }

  ndb_op_code = ndb_almanac_read(sid, &orbit.a);
  if (NDB_ERR_NONE == ndb_op_code) {
    /* TODO: check almanac validity once it becomes
             possible. For now assume that we do not have almanac. */
  }
  return flags;
}

/** Get parameter sets flags
 * \param[in] ctrl_info Controller parameters for error sigma computations
 * \return Bit field of #track_sbp_param_set_t flags
 */
static u8 get_pset_flags(const tracking_channel_ctrl_info_t *ctrl_info) {
  u8 flags = 0;

  switch (ctrl_info->int_ms) {
    case 1:
      flags = TRACK_SBP_PARAM_SET_1MS;
      break;
    case 2:
      flags = TRACK_SBP_PARAM_SET_2MS;
      break;
    case 5:
      flags = TRACK_SBP_PARAM_SET_5MS;
      break;
    case 10:
      flags = TRACK_SBP_PARAM_SET_10MS;
      break;
    case 20:
      flags = TRACK_SBP_PARAM_SET_20MS;
      break;
    default:
      log_error("ctrl_info->pll_bw  %.1f fll_bw  %.1f dll_bw %.1f int_ms %d",
                ctrl_info->pll_bw,
                ctrl_info->fll_bw,
                ctrl_info->dll_bw,
                ctrl_info->int_ms);
      assert(!"Unsupported integration time.");
      break;
  }

  return flags;
}

/** Get miscellaneous flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_channel_status_t,
 *         #TRACK_SBP_ACCELERATION_VALID,
 *         #TRACK_SBP_HALF_CYCLE_AMBIGUITY_RESOLVED and
 *         #TRACK_SBP_PSEUDORANGE_VALID flags
 */
static u8 get_misc_flags(const tracking_channel_info_t *channel_info) {
  /* TODO: set status correctly when re-acq support is added */
  u8 flags = TRACK_SBP_STATUS_RUNNING; /* no re-acq state support */

  flags |= TRACK_SBP_ACCELERATION_VALID; /* acceleration is always valid */
  ;

  if (0 != (channel_info->flags & TRACKER_FLAG_BIT_POLARITY_KNOWN)) {
    flags |= TRACK_SBP_HALF_CYCLE_AMBIGUITY_RESOLVED;
  }

  return flags;
}

/** Limit the provided value by coercing it into the provided limits.
 * \param[in] value The value to coerce
 * \param[in] min Minimum value
 * \param[in] max Maximum value
 * \return The coerced value
 */
static double limit_value(double value, s64 min, s64 max) {
  if (value > max) {
    value = max;
  } else if (value < min) {
    value = min;
  }
  return value;
}

/** Get detailed tracker state
 * \param[out] state Detailed tracker state
 * \param[in] channel_info Channel information
 * \param[in] freq_info Frequency information
 * \param[in] ctrl_info Controller parameters for error sigma computations
 * \param[in] misc_info Miscellaneous information
 * \param[in] lgf Last good fix
 * \return None
 */
void track_sbp_get_detailed_state(msg_tracking_state_detailed_t *state,
                                  const tracking_channel_info_t *channel_info,
                                  const tracking_channel_freq_info_t *freq_info,
                                  const tracking_channel_time_info_t *time_info,
                                  const tracking_channel_ctrl_info_t *ctrl_info,
                                  const tracking_channel_misc_info_t *misc_info,
                                  const last_good_fix_t *lgf) {
  u64 recv_time_ticks = nap_sample_time_to_count(channel_info->sample_count);
  /* receiver clock time of the measurements [ns] */
  state->recv_time = nap_count_to_ns(recv_time_ticks);

  channel_measurement_t meas;

  tracking_channel_measurement_get(
      recv_time_ticks, channel_info, freq_info, time_info, misc_info, &meas);

  s32 tow_ms = channel_info->tow_ms;

  double raw_pseudorange = 0;
  if ((0 != (channel_info->flags & TRACKER_FLAG_TOW_VALID)) &&
      (0 != (channel_info->flags & TRACKER_FLAG_ACTIVE)) &&
      (0 == (channel_info->flags & TRACKER_FLAG_ERROR)) &&
      (get_time_quality() >= TIME_FINE)) {
    u64 ref_tc = nap_sample_time_to_count(channel_info->sample_count);
    tracking_channel_calc_pseudorange(ref_tc, &meas, &raw_pseudorange);
  }

  /* TOW status flags */
  state->tow_flags = get_tow_flags(channel_info);

  if (TOW_UNKNOWN == tow_ms) {
    u8 tow_status = state->tow_flags & TRACK_SBP_TOW_STATUS_MASK;
    bool tow_available = (tow_status != TRACK_SBP_TOW_NONE);

    state->tot.tow = 0;
    assert(!tow_available);
  } else {
    state->tot.tow = tow_ms;
  }

  state->tot.wn = 0;
  if (get_time_quality() >= TIME_COARSE) {
    gps_time_t rec_time = napcount2gpstime(recv_time_ticks);

    if (WN_UNKNOWN != rec_time.wn) {
      state->tot.wn = rec_time.wn;
      state->tow_flags |= TRACK_SBP_WN_VALID;
    }
  }

  double pseudorange = raw_pseudorange * MSG_OBS_P_MULTIPLIER;
  state->P = (u32)limit_value(pseudorange, 0, UINT32_MAX);

  /* pseudorange standard deviation is not computed
     as it was deemed to be unusable. Mark as invalid for now.
     TODO: rework the detailed tracking status message be removing
     this field */
  state->P_std = UINT16_MAX;

  /* carrier phase coming from NAP (cycles) */
  double L = freq_info->carrier_phase;
  L = limit_value(L, INT32_MIN, INT32_MAX);
  double Li = floor(-L);
  double Lf = -L - Li;
  state->L.i = (s32)Li;
  state->L.f = (u8)(Lf * MSG_OBS_LF_MULTIPLIER);

  /* TODO GLO: Handle GLO orbit slot properly. */
  u16 glo_orbit_slot = channel_info->glo_orbit_slot;
  if (GLO_ORBIT_SLOT_UNKNOWN == glo_orbit_slot) {
    glo_orbit_slot = channel_info->mesid.sat;
  }
  gnss_signal_t sid = mesid2sid(channel_info->mesid, glo_orbit_slot);

  float cn0 = channel_info->cn0 * MSG_OBS_CN0_MULTIPLIER;
  state->cn0 = (u8)limit_value(cn0, 0, UINT8_MAX);

  double lock_time = tracking_channel_get_lock_time(time_info, misc_info);
  state->lock = encode_lock_time(lock_time);

  state->sid = sid_to_sbp(sid);

  double carrier_freq =
      freq_info->carrier_freq * TRACK_SBP_DOPPLER_SCALING_FACTOR;
  state->doppler = (s32)limit_value(carrier_freq, INT32_MIN, INT32_MAX);

  /* Doppler standard deviation [Hz] is not computed
     as it was deemed to be unusable. Mark as invalid for now.
     TODO: rework the detailed tracking status message be removing
     this field */
  state->doppler_std = UINT16_MAX;

  /* number of seconds of continuous tracking */
  u64 now = timing_getms();
  state->uptime = (u32)((now - channel_info->init_timestamp_ms) / 1000);

  /* miscellaneous flags */
  state->misc_flags = get_misc_flags(channel_info);
  if (state->P != 0) {
    state->misc_flags |= TRACK_SBP_PSEUDORANGE_VALID;
  }

  if ((NULL != lgf) && (lgf->position_quality >= POSITION_GUESS)) {
    double clock_offset = lgf->position_solution.clock_offset *
                          TRACK_SBP_CLOCK_OFFSET_SCALING_FACTOR;
    double clock_drift = lgf->position_solution.clock_drift *
                         TRACK_SBP_CLOCK_DRIFT_SCALING_FACTOR;
    state->clock_offset = (s16)limit_value(clock_offset, INT16_MIN, INT16_MAX);
    state->clock_drift = (s16)limit_value(clock_drift, INT16_MIN, INT16_MAX);
    state->misc_flags |= TRACK_SBP_CLOCK_VALID;
  } else {
    state->clock_offset = 0;
    state->clock_drift = 0;
  }

  /* correlator spacing [ns] */
  state->corr_spacing =
      (u16)(NAP_EPL_SPACING_SAMPLES * TRACK_SBP_NAP_SPACING_SCALING_FACTOR);
  /* acceleration [g] */
  double acceleration =
      freq_info->acceleration * TRACK_SBP_ACCELERATION_SCALING_FACTOR;
  state->acceleration = (s8)(limit_value(acceleration, INT8_MIN, INT8_MAX));

  /* sync status flags */
  state->sync_flags = get_sync_flags(channel_info);

  /* track flags */
  state->track_flags = get_track_flags(channel_info);

  /* navigation data status flags */
  state->nav_flags = get_nav_data_status_flags(sid);

  /* parameters sets flags */
  state->pset_flags = get_pset_flags(ctrl_info);
}
