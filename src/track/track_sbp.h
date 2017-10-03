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

#ifndef _TRACK_SBP_H
#define _TRACK_SBP_H

#include "ndb.h"
#include "track.h"

/** Acceleration scaling factor. Original units [g]. */
#define TRACK_SBP_ACCELERATION_SCALING_FACTOR 8

/** Doppler frequency scaling factor. Original units [Hz]. */
#define TRACK_SBP_DOPPLER_SCALING_FACTOR 16

/** Clock drift scaling factor. Original units [s/s]. */
#define TRACK_SBP_CLOCK_DRIFT_SCALING_FACTOR (1ul << 31)

/** Clock offset scaling factor. Original units [s]. */
#define TRACK_SBP_CLOCK_OFFSET_SCALING_FACTOR (1ul << 20)

/** NAP correlator spacing scaling factor. Original units [cycles]. */
#define TRACK_SBP_NAP_SPACING_SCALING_FACTOR (1e9 / NAP_CODE_SAMPLE_RATE_Hz)

/** Synchronization status */
typedef enum {
  TRACK_SBP_SYNC_NONE = 0, /**< no synchronization */
  TRACK_SBP_SYNC_BIT = 1,  /**< bit synchronization */
} track_sbp_sync_status_t;

typedef enum {
  TRACK_SBP_TOW_NONE = 0,      /**< TOW is not available */
  TRACK_SBP_TOW_DECODED = 1,   /**< decoded TOW is available */
  TRACK_SBP_TOW_PROPAGATED = 2 /**< propagated TOW is available */
} track_sbp_tow_status_t;

/** TOW status mask */
#define TRACK_SBP_TOW_STATUS_MASK 0x3

/** Week number validity flag */
#define TRACK_SBP_WN_VALID (1 << 3)

/** Tracking loop status flags */
typedef enum {
  TRACK_SBP_LOOP_NO_LOCK = 0,              /**< no locks */
  TRACK_SBP_LOOP_FLL_LOCK = 1,             /**< FLL/DLL lock */
  TRACK_SBP_LOOP_PLL_OPTIMISTIC_LOCK = 2,  /**< PLL optimistic lock */
  TRACK_SBP_LOOP_PLL_PESSIMISTIC_LOCK = 3, /**< PLL pessimistic lock */
} track_sbp_loop_status_t;

/** PLL is active */
#define TRACK_SBP_LOOP_PLL (1 << 3)
/** FLL is active */
#define TRACK_SBP_LOOP_FLL (1 << 4)

/** SV health status */
typedef enum {
  TRACK_SBP_HEALTH_UNKNOWN = 0, /**< health status is unknown */
  TRACK_SBP_HEALTH_BAD = 1,     /**< SV is unhealthy */
  TRACK_SBP_HEALTH_GOOD = 2     /**< SV is healthy */
} track_sbp_sv_health_status_t;

/** Ephemeris data is available */
#define TRACK_SBP_NAV_STATE_EPHEMERIS (1 << 3)
/** Almanac data is available */
#define TRACK_SBP_NAV_STATE_ALMANAC (1 << 4)

/** Parameter sets */
typedef enum {
  TRACK_SBP_PARAM_SET_1MS = 0,  /**< 1ms integration time */
  TRACK_SBP_PARAM_SET_2MS = 1,  /**< 2ms integration time */
  TRACK_SBP_PARAM_SET_5MS = 2,  /**< 5ms integration time */
  TRACK_SBP_PARAM_SET_10MS = 3, /**< 10ms integration time */
  TRACK_SBP_PARAM_SET_20MS = 4  /**< 20ms integration time */
} track_sbp_param_set_t;

/** Tracking channel status */
typedef enum {
  TRACK_SBP_STATUS_RE_ACQ = 0, /**< re-acquisition state */
  TRACK_SBP_STATUS_RUNNING = 1 /**< tracking state */
} track_sbp_channel_status_t;

/** Carrier half cycle ambiguity is resolved */
#define TRACK_SBP_HALF_CYCLE_AMBIGUITY_RESOLVED (1 << 2)
/** Acceleration is valid */
#define TRACK_SBP_ACCELERATION_VALID (1 << 3)
/** Pseudorange is valid */
#define TRACK_SBP_PSEUDORANGE_VALID (1 << 4)
/** Clock offset and drift are valid */
#define TRACK_SBP_CLOCK_VALID (1 << 5)

void track_sbp_get_detailed_state(msg_tracking_state_detailed_t *state,
                                  const tracking_channel_info_t *channel_info,
                                  const tracking_channel_freq_info_t *freq_info,
                                  const tracking_channel_time_info_t *time_info,
                                  const tracking_channel_ctrl_info_t *ctrl_info,
                                  const tracking_channel_misc_info_t *misc_info,
                                  const last_good_fix_t *lgf);

#endif /* _TRACK_SBP_H */
