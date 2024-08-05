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

#ifndef STARLING_EXTERNAL_DEPENDENCIES_H_
#define STARLING_EXTERNAL_DEPENDENCIES_H_

#include <starling/starling.h>

#ifdef __cplusplus
extern "C" {
#endif

/* For the time being, Starling has a couple unresolved external dependencies.
 * They are enumerated here.
 *
 * As we work to the functionality provided in these functions with
 * implementations
 * internal to Starling, this file should vanish. */

enum {
  /* Cached data was read successfully. */
  CACHE_OK = 0,
  /* Cached data was read successfully, but the data is not confirmed to be
     valid. */
  CACHE_OK_UNCONFIRMED_DATA,
  /* Attempt to read from cache failed. */
  CACHE_ERROR,
};
typedef int cache_ret_t;

typedef struct external_functions_t {
  cache_ret_t (*cache_read_ephemeris)(const gnss_signal_t sid,
                                      ephemeris_t *ephemeris);
  cache_ret_t (*cache_read_iono_corr)(ionosphere_t *iono);

  bool (*track_sid_db_elevation_degrees_get)(const gnss_signal_t sid,
                                             double *elevation);

  bool (*shm_navigation_unusable)(const gnss_signal_t sid);

  bool (*starling_integration_simulation_enabled)(void);
  void (*starling_integration_simulation_run)(void);

  bool (*disable_raim)(void);
} external_functions_t;

void starling_set_external_functions_implementation(external_functions_t *impl);

extern external_functions_t external_functions;

#ifdef __cplusplus
}
#endif

#endif
