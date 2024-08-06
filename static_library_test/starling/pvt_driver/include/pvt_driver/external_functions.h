/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef PVT_DRIVER_EXTERNAL_FUNCTIONS_H
#define PVT_DRIVER_EXTERNAL_FUNCTIONS_H

#include <swiftnav/ephemeris.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/signal.h>

#include <pvt_engine/obss.h>

#ifdef __cplusplus
extern "C" {
#endif

/* For the time being, Starling has a couple unresolved external dependencies.
 * They are enumerated here.
 *
 * As we work to the functionality provided in these functions with
 * implementations
 * internal to Starling, this file should vanish. */

typedef enum {
  /* Cached data was read successfully. */
  PVT_DRIVER_CACHE_OK = 0,
  /* Cached data was read successfully, but the data is not confirmed to be
     valid. */
  PVT_DRIVER_CACHE_OK_UNCONFIRMED_DATA,
  /* Attempt to read from cache failed. */
  PVT_DRIVER_CACHE_ERROR,
} pvt_driver_cache_ret_t;

typedef struct {
  pvt_driver_cache_ret_t (*cache_read_ephemeris)(const gnss_signal_t sid,
                                                 ephemeris_t *ephemeris,
                                                 void *ctx);
  pvt_driver_cache_ret_t (*cache_read_iono_corr)(ionosphere_t *iono, void *ctx);

  bool (*track_sid_db_elevation_degrees_get)(const gnss_signal_t sid,
                                             double *elevation, void *ctx);

  bool (*shm_navigation_unusable)(const gnss_signal_t sid, void *ctx);

  bool (*simulation_enabled)(void *ctx);
  void (*simulation_run)(void *ctx);

  bool (*disable_raim)(void *ctx);

  void *ctx;
} pvt_driver_external_functions_t;

#ifdef __cplusplus
}
#endif

#endif  // PVT_DRIVER_EXTERNAL_FUNCTIONS_H
