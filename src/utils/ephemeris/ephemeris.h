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

#include <libswiftnav/almanac.h>
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
typedef struct { double xyz[3]; } pos_ecef_t;

#ifdef __cplusplus
extern "C" {
#endif

void ephemeris_setup(void);
eph_new_status_t ephemeris_new(const ephemeris_t *e);
s8 update_azel_from_ephemeris(const ephemeris_t *e,
                              const gps_time_t *t,
                              const double pos_ecef[]);
s8 update_azel_from_almanac(const almanac_t *a,
                            const gps_time_t *t,
                            const double pos_ecef[]);

/**
 * Helper for converting 32-bit timestamp into GPS time structure.
 *
 * \param time_s Time in seconds since GPS epoch.
 *
 * \return GPS time
 */
static inline gps_time_t make_gps_time(s32 time_s) {
  return (gps_time_t){time_s % WEEK_SECS, time_s / WEEK_SECS};
}

#ifdef __cplusplus
}
#endif

#endif
