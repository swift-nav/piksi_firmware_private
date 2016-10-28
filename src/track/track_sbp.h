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

#include "track_profile_utils.h"

/** Acceleration scaling factor */
#define TRACK_ACCELERATION_SCALING_FACTOR 8

/** Carrier phase scaling factor */
#define TRACK_CARR_PHASE_SCALING_FACTOR 256

/** Doppler frequency scaling factor */
#define TRACK_DOPPLER_SCALING_FACTOR 16

/** Synchronization status */
typedef enum {
  TRACK_SYNC_NONE = 0,     /**< no synchronization */
  TRACK_SYNC_BIT = 1,      /**< bit synchronization */
  TRACK_SYNC_WORD = 2,     /**< word synchronization */
  TRACK_SYNC_SUBFRAME = 3, /**< subframe synchronization (relevant to L1C/A) */
  TRACK_SYNC_MESSAGE = TRACK_SYNC_SUBFRAME /**< message synchronization
                                                (relevant to L2C) */
} track_sync_status_t;

/** TOW status */
typedef enum {
  TRACK_TOW_NONE = 0,       /**< TOW is not available */
  TRACK_TOW_DECODED = 1,    /**< decoded TOW is available */
  TRACK_TOW_PROPAGATED = 2  /**< propagated TOW is available */
} track_tow_status_t;

/** Tracking loop status flags */
typedef enum {
  TRACK_LOOP_NO_LOCK = 0,              /**< no locks */
  TRACK_LOOP_FLL_LOCK = 1,             /**< FLL/DLL lock */
  TRACK_LOOP_PLL_OPTIMISTIC_LOCK = 2,  /**< PLL optimistic lock */
  TRACK_LOOP_PLL_PESSIMISTIC_LOCK = 3, /**< PLL pessimistic lock */
  TRACK_LOOP_PLL = (1 << 3),           /**< PLL is active */
  TRACK_LOOP_FLL = (1 << 4)            /**< FLL is active */
} track_loop_status_t;

/** SV health status */
typedef enum {
  TRACK_HEALTH_UNKNOWN = 0,  /**< health status is unknown */
  TRACK_HEALTH_GOOD = 1,     /**< SV is healthy */
  TRACK_HEALTH_BAD = 2       /**< SV is unhealthy */
} sv_health_status_t;

/** Ephemeris data is available */
#define TRACK_NAV_STATE_EPHEMERIS (1 << 3)
/** Almanac data is available */
#define TRACK_NAV_STATE_ALMANAC   (1 << 4)

/** Parameter sets */
typedef enum {
  TRACK_PARAM_SET_1MS = 0,      /**< 1ms integration time */
  TRACK_PARAM_SET_5MS = 1,      /**< 5ms integration time */
  TRACK_PARAM_SET_10MS = 2,     /**< 10ms integration time */
  TRACK_PARAM_SET_20MS = 3      /**< 20ms integration time */
} track_param_set_t;

/** Tracking channel status */
enum {
  TRACK_STATUS_RE_ACQ = 0,      /**< re-acquisition state */
  TRACK_STATUS_RUNNING = 1      /**< tracking state */
};

/** Carrier half cycle ambiguity is resolved */
#define TRACK_HALF_CYCLE_AMBIGUITY_RESOLVED (1 << 3)
/** Acceleration is valid */
#define TRACK_ACCELERATION_VALID            (1 << 4)
/** Pseudorange is valid */
#define TRACK_PSEUDORANGE_VALID             (1 << 5)

/** CN0 scaling factor */
#define TRACK_CN0_SCALING_FACTOR 4

bool track_sbp_send_state(const tracker_channel_info_t *channel_info,
                          tracker_common_data_t *common_data,
                          const tp_tracker_data_t *data);

#endif  /* _TRACK_SBP_H */
