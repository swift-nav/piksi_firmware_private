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
/* #include <math.h> */
#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <timing.h>
#include "board/nap/track_channel.h"
#include "board/v3/nap/nap_hw.h"
#include "board/v3/nap/nap_constants.h"
/* #include "track.h" */
#include "track_sbp.h"
#include "track_internal.h"
/* #include "position.h" */
#include "shm.h"
#include <sbp.h>
#include <sbp_utils.h>

/** Get synchronization status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_sync_status_t flags
 */
static u8 get_sync_flags(const tracking_channel_info_t *channel_info)
{
  u8 flags = TRACK_SBP_SYNC_NONE;
  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_SUBFRAME_SYNC)) {
    flags = TRACK_SBP_SYNC_SUBFRAME;
  } else if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_WORD_SYNC)) {
    flags = TRACK_SBP_SYNC_WORD;
  } else if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_BIT_SYNC)) {
    flags = TRACK_SBP_SYNC_BIT;
  }
  return flags;
}

/** Get TOW status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_tow_status_t flags
 */
static u8 get_tow_flags(const tracking_channel_info_t *channel_info)
{
  u8 flags = TRACK_SBP_TOW_NONE;
  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_TOW_DECODED)) {
    flags = TRACK_SBP_TOW_DECODED;
  } else if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_TOW_PROPAGATED)) {
    flags = TRACK_SBP_TOW_PROPAGATED;
  }

  return flags;
}

/** Get tracking loop status flags
 * \param[in] channel_info channel info
 * \return Bit field of #track_sbp_loop_status_t,
 *         #TRACK_SBP_LOOP_PLL and #TRACK_SBP_LOOP_FLL flags
 */
static u8 get_track_flags(const tracking_channel_info_t *channel_info)
{
  u8 flags = TRACK_SBP_LOOP_NO_LOCK;
  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_PLL_PLOCK)) {
    flags = TRACK_SBP_LOOP_PLL_PESSIMISTIC_LOCK;
  } else if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_PLL_OLOCK)) {
    flags = TRACK_SBP_LOOP_PLL_OPTIMISTIC_LOCK;
  } else if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_FLL_LOCK)) {
    flags = TRACK_SBP_LOOP_FLL_LOCK;
  }
  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_PLL_USE)) {
    flags |= TRACK_SBP_LOOP_PLL;
  }
  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_FLL_USE)) {
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
static u8 get_nav_data_status_flags(gnss_signal_t sid)
{
  u8 flags = TRACK_SBP_HEALTH_UNKNOWN;
  code_nav_state_t nav_state = shm_get_sat_state(sid);

  switch (nav_state) {
  case CODE_NAV_STATE_UNKNOWN:
    flags = TRACK_SBP_HEALTH_UNKNOWN;
    break;
  case CODE_NAV_STATE_VALID:
    flags = TRACK_SBP_HEALTH_GOOD;
    break;
  case CODE_NAV_STATE_INVALID:
    flags = TRACK_SBP_HEALTH_BAD;
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
static u8 get_pset_flags(const tracking_channel_ctrl_info_t *ctrl_info)
{
  u8 flags = 0;
  if (1 /*[ms]*/ == ctrl_info->int_ms) {
    flags = TRACK_SBP_PARAM_SET_1MS;
  } else if (5 /*[ms]*/ == ctrl_info->int_ms) {
    flags = TRACK_SBP_PARAM_SET_5MS;
  } else if (10 /*[ms]*/ == ctrl_info->int_ms) {
    flags = TRACK_SBP_PARAM_SET_10MS;
  } else if (20 /*[ms]*/ == ctrl_info->int_ms) {
    flags = TRACK_SBP_PARAM_SET_20MS;
  } else {
    assert(!"Unsupported integration time.");
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
static u8 get_misc_flags(const tracking_channel_info_t *channel_info)
{
  /* TODO: set status correctly when re-acq support is added */
  u8 flags = TRACK_SBP_STATUS_RUNNING;   /* no re-acq state support */

  flags |= TRACK_SBP_ACCELERATION_VALID; /* acceleration is always valid */;

  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_BIT_POLARITY)) {
    flags |= TRACK_SBP_HALF_CYCLE_AMBIGUITY_RESOLVED;
  }

  if (0 != (channel_info->flags & TRACKING_CHANNEL_FLAG_PSEUDORANGE)) {
    flags |= TRACK_SBP_PSEUDORANGE_VALID;
  }

  return flags;
}

/** Send tracker state over SBP protocol
 * \param[in] channel_info Channel information
 * \param[in] freq_info Frequency information
 * \param[in] ctrl_info Controller parameters for error sigma computations
 * \param[in] misc_info Miscellaneous information
 * \param[in] lgf Last good fix
 * \return None
 */
void track_sbp_send_state(const tracking_channel_info_t *channel_info,
                          const tracking_channel_freq_info_t *freq_info,
                          const tracking_channel_ctrl_info_t *ctrl_info,
                          const tracking_channel_misc_info_t *misc_info,
                          const last_good_fix_t *lgf)
{
  msg_tracking_state_detailed_t state;

  u64 recv_time_ticks = nap_sample_time_to_count(channel_info->sample_count);
  /* receiver clock time of the measurements [ns] */
  state.recv_time = nap_count_to_ns(recv_time_ticks);

  gps_time_t rec_time = rx2gpstime(recv_time_ticks);
  channel_measurement_t meas;
  bool wn_valid = false;
  bool clk_valid = false;

  tracking_channel_measurement_get(recv_time_ticks,
                                   channel_info,
                                   freq_info,
                                   &meas);

  s32 tow_ms = channel_info->tow_ms;
  state.tot.tow = tow_ms > 0 ? tow_ms : 0;

  if (TOW_UNKNOWN == tow_ms) {
    state.tot.tow = 0;
  } else {
    state.tot.tow = tow_ms;
  }

  if (WN_UNKNOWN == rec_time.wn) {
    state.tot.wn = 0;
  } else {
    state.tot.wn = rec_time.wn;
    wn_valid = true;
  }

  state.P = misc_info->pseudorange;
  state.P_std = misc_info->pseudorange_std;

  /* carrier phase coming from NAP (cycles) */
  state.L.i = (u32)freq_info->carrier_phase;
  state.L.f = (u8)((freq_info->carrier_phase - state.L.i) *
                   TRACK_SBP_CARR_PHASE_SCALING_FACTOR);

  gnss_signal_t sid = channel_info->sid;

  state.cn0 = channel_info->cn0 * TRACK_SBP_CN0_SCALING_FACTOR;
  state.lock = tracking_lock_counter_get(sid);

  state.sid = sid_to_sbp(sid);

  state.doppler = (s32)(freq_info->carrier_freq *
                        TRACK_SBP_DOPPLER_SCALING_FACTOR);

  /* Doppler standard deviation [Hz] */
  double doppler_std = freq_info->carrier_freq_std;
  state.doppler_std = (u16)(doppler_std *
                            TRACK_SBP_DOPPLER_SCALING_FACTOR);

  /* number of seconds of continuous tracking */
  state.uptime = (u32)(channel_info->uptime_ms * 1e-3);

  if ((NULL != lgf) && (lgf->position_quality >= POSITION_GUESS)) {
    state.clock_offset = (u16)(lgf->position_solution.clock_offset * 1e6);
    state.clock_drift = (u16)(lgf->position_solution.clock_bias * 1e9);
    clk_valid = true;
  } else {
    state.clock_offset = 0;
    state.clock_drift = 0;
  }

  /* correlator spacing [ns] */
  state.corr_spacing = (u16)(1e9 * NAP_SPACING / TRACK_SAMPLE_FREQ);
  /* acceleration [g] */
  state.acceleration = (u8)(freq_info->acceleration *
                            TRACK_SBP_ACCELERATION_SCALING_FACTOR);

  /* sync status flags */
  state.sync_flags = get_sync_flags(channel_info);

  /* TOW status flags */
  state.tow_flags = get_tow_flags(channel_info);
  if (wn_valid) {
    state.tow_flags |= TRACK_SBP_WN_VALID;
  }

  /* track flags */
  state.track_flags = get_track_flags(channel_info);

  /* navigation data status flags */
  state.nav_flags = get_nav_data_status_flags(sid);

  /* parameters sets flags */
  state.pset_flags = get_pset_flags(ctrl_info);

  /* miscellaneous flags */
  state.misc_flags = get_misc_flags(channel_info);
  if (clk_valid) {
    state.misc_flags |= TRACK_SBP_CLOCK_VALID;
  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE_DETAILED, sizeof(state), (u8*)&state);
}
