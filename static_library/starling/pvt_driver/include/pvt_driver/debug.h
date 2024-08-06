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

#ifndef PVT_DRIVER_DEBUG_H
#define PVT_DRIVER_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PVT_DRIVER_PROFILE_BEGIN,
  PVT_DRIVER_PROFILE_END,
} pvt_driver_profile_directive_t;

typedef struct {
  /* Will be called at the beginning and end of a low-latency thread iteration.
   */
  void (*profile_low_latency_thread)(pvt_driver_profile_directive_t directive,
                                     void *ctx);

  void *ctx;
} pvt_driver_debug_functions_t;

#ifdef __cplusplus
}
#endif

#endif  // PVT_DRIVER_DEBUG_H
