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

#include "track.h"

/** Tracker interface function template. */
typedef void(tracker_interface_function_t)(tracker_channel_t *tracker_channel);

/** Interface to a tracker implementation. */
typedef struct tracker_interface {
  /** Code type for which the implementation may be used. */
  code_t code;
  /** Init function. Called to set up tracker instance when tracking begins. */
  tracker_interface_function_t *init;
  /** Disable function. Called when tracking stops. */
  tracker_interface_function_t *disable;
  /** Update function. Called when new correlation outputs are available. */
  tracker_interface_function_t *update;
} tracker_interface_t;

/** List element passed to tracker_interface_register(). */
typedef struct tracker_interface_list_element_t {
  const tracker_interface_t *interface;
  struct tracker_interface_list_element_t *next;
} tracker_interface_list_element_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

tracker_interface_list_element_t **tracker_interface_list_ptr_get(void);

void tracker_interface_register(tracker_interface_list_element_t *element);

const tracker_interface_t *tracker_interface_lookup(
    const me_gnss_signal_t mesid);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_INTERFACE_H */
