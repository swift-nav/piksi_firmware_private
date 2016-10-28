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
#include <math.h>
#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <timing.h>
#include "board/nap/track_channel.h"
#include "board/v3/nap/nap_hw.h"
#include "board/v3/nap/nap_constants.h"
#include "track.h"
#include "track_sbp.h"
#include "track_internal.h"
#include "position.h"
#include "ndb.h"
#include "shm.h"
#include <sbp.h>
#include <sbp_utils.h>

/** Get synchronization status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sync_status_t flags
 */
static u8 get_sync_flags(const tracker_channel_info_t *channel_info)
{
  u8 flags = TRACK_SYNC_NONE;
  bool bit_sync = tracker_has_bit_sync(channel_info->context);
  if (bit_sync) {
    flags |= TRACK_SYNC_BIT;
  }
  /* TODO: add subframe (L1C/A) and message sync states (L2C) */

  return flags;
}

/** Get TOW status flags
 * \param[in] common_data tracking loop common data
 * \return Bit field of #track_tow_status_t flags
 */
static u8 get_tow_flags(const tracker_common_data_t *common_data)
{
  u8 flags = TRACK_TOW_NONE;
  if (0 != (common_data->flags & TRACK_CMN_FLAG_TOW_DECODED)) {
    flags |= TRACK_TOW_DECODED;
  } else if (0 != (common_data->flags & TRACK_CMN_FLAG_TOW_PROPAGATED)) {
    flags |= TRACK_TOW_PROPAGATED;
  }

  return flags;
}

/** Get tracking loop status flags
 * \param[in] common_data tracking loop common data
 * \return Bit field of #track_loop_status_t flags
 */
static u8 get_track_flags(const tracker_common_data_t *common_data)
{
  u8 flags = 0;
  if (0 != (common_data->flags & TRACK_CMN_FLAG_PLL_USE)) {
    flags |= TRACK_LOOP_PLL;
  }
  if (0 != (common_data->flags & TRACK_CMN_FLAG_FLL_USE)) {
    flags |= TRACK_LOOP_FLL;
  }
  if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_PLOCK)) {
    flags |= TRACK_LOOP_PLL_PESSIMISTIC_LOCK;
  } else if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_OLOCK)) {
    flags |= TRACK_LOOP_PLL_OPTIMISTIC_LOCK;
  } else if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_FLOCK)) {
    flags |= TRACK_LOOP_FLL_LOCK;
  } else {
    flags |= TRACK_LOOP_NO_LOCK;
  }
  return flags;
}

/** Get navigation data status flags
 * \param[in] common_data tracking loop common data
 * \param[in] sid signal identifier
 * \return Bit field of #sv_health_status_t,
 *         #TRACK_NAV_STATE_EPHEMERIS and #TRACK_NAV_STATE_ALMANAC flags
 */
static u8 get_nav_data_status_flags(gnss_signal_t sid)
{
  u8 flags = 0;
  code_nav_state_t nav_state = shm_get_sat_state(sid);

  switch (nav_state) {
  case CODE_NAV_STATE_UNKNOWN:
    flags |= TRACK_HEALTH_UNKNOWN;
    break;
  case CODE_NAV_STATE_VALID:
    flags |= TRACK_HEALTH_GOOD;
    break;
  case CODE_NAV_STATE_INVALID:
    flags |= TRACK_HEALTH_BAD;
    break;
  default:
    assert(!"Unknown nav state");
    break;
  }

  if (time_quality <= TIME_GUESS) {
    return flags;
  }

  gps_time_t t = get_current_time();
  union { ephemeris_t e; almanac_t a; } orbit;
  enum ndb_op_code ndb_op_code;

  ndb_op_code = ndb_ephemeris_read(sid, &orbit.e);
  if (NDB_ERR_NONE == ndb_op_code) {
    if (ephemeris_valid(&orbit.e, &t)) {
      flags |= TRACK_NAV_STATE_EPHEMERIS;
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
 * \param[in] common_data tracking loop common data
 * \return Bit field of #track_param_set_t flags
 */
static u8 get_pset_flags(const tracker_common_data_t *common_data)
{
  u8 flags = 0;
  if (1 /*[ms]*/ == common_data->ctrl_params.int_ms) {
    flags |= TRACK_PARAM_SET_1MS;
  } else if (5 /*[ms]*/ == common_data->ctrl_params.int_ms) {
    flags |= TRACK_PARAM_SET_5MS;
  } else if (10 /*[ms]*/ == common_data->ctrl_params.int_ms) {
    flags |= TRACK_PARAM_SET_10MS;
  } else if (20 /*[ms]*/ == common_data->ctrl_params.int_ms) {
    flags |= TRACK_PARAM_SET_20MS;
  } else {
    assert(!"Unsupported integration time.");
  }
  return flags;
}

/** Get miscellaneous flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_param_set_t flags
 */
static u8 get_misc_flags(const tracker_channel_info_t *channel_info)
{
  u8 flags = TRACK_ACCELERATION_VALID; /* acceleration is always valid */
  tracker_internal_data_t *internal_data;

  flags |= TRACK_STATUS_RUNNING;       /* no re-acq state support */

  tracker_internal_context_resolve(channel_info->context, &channel_info, &internal_data);
  if (internal_data->bit_polarity != BIT_POLARITY_UNKNOWN) {
    flags |= TRACK_HALF_CYCLE_AMBIGUITY_RESOLVED;
  }
  return flags;
}

/** Send tracker state over SBP protocol
 * \param[in] channel_info Channel information
 * \param[out] common_data Tracker channel common data
 * \param[in] data Tracker channel data
 * \retval true channel state SBP was sent
 * \retval false channel state SBP was not sent
 */
bool track_sbp_send_state(const tracker_channel_info_t *channel_info,
                          tracker_common_data_t *common_data,
                          const tp_tracker_data_t *data)
{
  u16 now_ms;
  u16 update_period_ms;
  msg_tracking_state_detailed_t state;

  (void)data;

  /* check if is time to send an update */
  now_ms = (u16)(1e3 * nap_timing_count() * RX_DT_NOMINAL);
  update_period_ms = now_ms - common_data->sbp_update_time_ms;
  if (update_period_ms < TRACKER_STATE_UPDATE_PERIOD_MS) {
    return false;
  }

  common_data->sbp_update_time_ms = now_ms;

  u64 recv_time_ticks = common_data->sample_count;
  /* receiver clock time of the measurements [ns] */
  state.recv_time = recv_time_ticks * (1e9 / NAP_FRONTEND_SAMPLE_RATE_Hz);

  s32 tow_ms = common_data->TOW_ms;
  state.tot.tow = tow_ms > 0 ? tow_ms : 0;

  state.tot.wn = 0; /* TODO: add week number  */

  state.P = 0; /* TODO: add raw pseudorange  */
  state.P_std = 0; /* TODO: add raw pseudorange std deviation */

  /* carrier phase coming from NAP (cycles) */
  state.L.i = (u32)common_data->carrier_phase;
  state.L.f = (u8)((common_data->carrier_phase - state.L.i) *
                   TRACK_CARR_PHASE_SCALING_FACTOR);

  gnss_signal_t sid = channel_info->sid;

  state.cn0 = common_data->cn0 * TRACK_CN0_SCALING_FACTOR;
  state.lock = tracking_lock_counter_get(sid);

  state.sid = sid_to_sbp(sid);

  state.doppler = (s32)(common_data->carrier_freq *
                        TRACK_DOPPLER_SCALING_FACTOR);

  /* Doppler standard deviation [Hz] */
  double doppler_std = sqrt(common_data->carrier_freq_stat.variance);
  state.doppler_std = (u16)(doppler_std *
                            TRACK_DOPPLER_SCALING_FACTOR);

  /* number of seconds of continuous tracking */
  state.uptime = (u32)(common_data->update_count * 1e-3);

  last_good_fix_t lgf;
  if((NDB_ERR_NONE == ndb_lgf_read(&lgf)) &&
     (lgf.position_quality >= POSITION_GUESS)) {
    state.clock_offset = (u16)(lgf.position_solution.clock_offset * 1e6);
    state.clock_drift = (u16)(lgf.position_solution.clock_bias * 1e9);
  } else {
    state.clock_offset = 0;
    state.clock_drift = 0;
  }

  /* correlator spacing [ns] */
  state.corr_spacing = (u16)(1e9 * NAP_SPACING / TRACK_SAMPLE_FREQ);
  /* acceleration [g] */
  state.acceleration = (u8)(tp_profile_get_acceleration(&data->profile) *
                            TRACK_ACCELERATION_SCALING_FACTOR);

  /* sync status flags */
  state.sync_flags = get_sync_flags(channel_info);

  /* TOW status flags */
  state.tow_flags = get_tow_flags(common_data);

  /* track flags */
  state.track_flags = get_track_flags(common_data);

  /* navigation data status flags */
  state.nav_flags = get_nav_data_status_flags(sid);

  /* parameters sets flags */
  state.pset_flags = get_pset_flags(common_data);

  /* miscellaneous flags */
  state.misc_flags = get_misc_flags(channel_info);

  sbp_send_msg(SBP_MSG_TRACKING_STATE_DETAILED, sizeof(state), (u8*)&state);

  return true;
}
