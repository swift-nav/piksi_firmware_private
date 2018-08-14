/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef STARLING_OBS_CONVERTER_H_
#define STARLING_OBS_CONVERTER_H_

#include <libswiftnav/gnss_time.h>
#include <libswiftnav/nav_meas.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <starling/starling.h>

int convert_starling_obs_array_to_obss(obs_array_t *obs_array, obss_t *obss);

#endif
