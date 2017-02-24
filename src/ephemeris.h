/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_EPHEMERIS_H
#define SWIFTNAV_EPHEMERIS_H

#include <libswiftnav/constants.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/signal.h>

/** Ephemeris handling status */
typedef enum {
  EPH_NEW_OK,    /**< Ephemeris has been processed */
  EPH_NEW_ERR,   /**< Ephemeris processing error */
  EPH_NEW_XCORR, /**< Ephemeris is cross-correlated */
} eph_new_status_t;

/**
 * Earth-centered earth-fixed position
 */
typedef struct {
  double xyz[3];
} pos_ecef_t;

/**
 * SV positions computed for cross-correlation checks.
 */
typedef struct {
  u32 time_s;         /**< GPS time for prompt position [s] */
  u32 interval_s;     /**< Time interval between position [s] */
  union {
    struct {
      pos_ecef_t early, prompt, late;
    };
    pos_ecef_t epl[3];
  };
} xcorr_positions_t;

/**
 * Cross-correlation position match status
 */
typedef enum {
  XCORR_MATCH_RES_OK,        /**< Position match detected */
  XCORR_MATCH_RES_NO_ALMANAC,/**< Position match check is not done (no almanac,
                              *   not enough data, error) */
  XCORR_MATCH_RES_NO_MATCH   /**< Position mismatch detected */
} xcorr_match_res_t;

#ifdef __cplusplus
extern "C" {
#endif

void ephemeris_setup(void);
eph_new_status_t ephemeris_new(const ephemeris_t *e);
bool xcorr_calc_alm_positions(gnss_signal_t sid,
                              u32 time_s,
                              u32 interval_s,
                              xcorr_positions_t *pos);
bool xcorr_calc_eph_positions(const ephemeris_t *e,
                              u32 time_s,
                              xcorr_positions_t *pos);
bool xcorr_get_alm_positions(gnss_signal_t sid,
                             u32 time_s,
                             u32 interval_s,
                             xcorr_positions_t *pos);
bool xcorr_match_positions(gnss_signal_t sid0,
                           gnss_signal_t sid1,
                           const xcorr_positions_t *pos0,
                           const xcorr_positions_t *pos1);
xcorr_match_res_t xcorr_match_alm_position(gnss_signal_t sid0,
                                           gnss_signal_t sid,
                                           const xcorr_positions_t *eph_pos);

/**
 * Helper for converting 32-bit timestamp into GPS time structure.
 *
 * \param time_s Time in seconds since GPS epoch.
 *
 * \return GPS time
 */
static inline gps_time_t make_gps_time(s32 time_s)
{
  return (gps_time_t){ time_s % WEEK_SECS, time_s / WEEK_SECS};
}

#ifdef __cplusplus
}
#endif

#endif

