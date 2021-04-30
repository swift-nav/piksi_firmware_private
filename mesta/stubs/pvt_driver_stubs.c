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

bool pvt_driver_set_max_sats(pvt_driver_t ctx, s32 max_sats) {
  (void)ctx;
  (void)max_sats;
  return true;
}

pvt_driver_solution_mode_t pvt_driver_get_solution_mode(pvt_driver_t ctx) {
  return PVT_DRIVER_SOLN_MODE_LOW_LATENCY;
}
