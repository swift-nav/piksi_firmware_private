/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_INTERFACE_H
#define SWIFTNAV_TRACK_INTERFACE_H

#include "tracker.h"

/** Tracker interface function template. */
typedef void(tracker_interface_function_t)(tracker_t *tracker);

/** Interface to a tracker implementation. */
typedef struct _tracker_interface_t {
  /** Code type for which the implementation may be used. */
  code_t code;
  /** Init function. Called to set up tracker instance when tracking begins. */
  tracker_interface_function_t *init;
  /** Disable function. Called when tracking stops. */
  tracker_interface_function_t *disable;
  /** Update function. Called when new correlation outputs are available. */
  tracker_interface_function_t *update;
} tracker_interface_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void tracker_interface_register(const tracker_interface_t *element);

const tracker_interface_t *tracker_interface_lookup(const code_t code);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_INTERFACE_H */
