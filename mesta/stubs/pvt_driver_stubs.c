/*
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

#include <pvt_driver/pvt_driver_c.h>

pvt_driver_t pvt_driver = NULL;

obs_array_t *pvt_driver_alloc_rover_obs(pvt_driver_t ctx) {
  (void)ctx;
  return NULL;
}

obs_array_t *pvt_driver_alloc_base_obs(pvt_driver_t ctx) {
  (void)ctx;
  return NULL;
}

void pvt_driver_free_rover_obs(pvt_driver_t ctx, obs_array_t *obs_array) {
  (void)ctx;
  (void)obs_array;
}

void pvt_driver_free_base_obs(pvt_driver_t ctx, obs_array_t *obs_array) {
  (void)ctx;
  (void)obs_array;
}

bool pvt_driver_send_rover_obs(pvt_driver_t ctx, obs_array_t *obs_array) {
  (void)ctx;
  (void)obs_array;
}

bool pvt_driver_send_sbas_data(pvt_driver_t ctx,
                               const sbas_raw_data_t *sbas_data) {
  (void)ctx;
  (void)sbas_data;
  return true;
}

void pvt_driver_set_elevation_mask(pvt_driver_t ctx, float elevation_mask) {
  (void)ctx;
  (void)elevation_mask;
}

bool pvt_driver_set_solution_frequency(pvt_driver_t ctx,
                                       double requested_frequency_hz) {
  (void)ctx;
  (void)requested_frequency_hz;
  return true;
}
