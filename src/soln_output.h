/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Leith Bade <leith@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SOLN_OUTPUT_H
#define SWIFTNAV_SOLN_OUTPUT_H

#include <libswiftnav/common.h>
#include <libswiftnav/pvt.h>

/** Structure holding all inputs to send_solutions */
typedef struct {
  gps_time_t rec_time; /**< Receiver propagated GPS time */
  bool rec_time_valid; /**< True if rec_time contains valid data */

  /* TODO(Leith): in future add RTC chip time */

  gps_time_t spp_time; /**< SPP computed GPS time */
  bool spp_time_valid; /**< True if spp_time contains valid data */

  double spp_position[3]; /**< SPP computed ECEF position [m] */
  bool spp_position_valid; /**< True if spp_position contains valid data */
  u8 spp_position_num_sats; /**< Number of sats used in SPP position solution */

  double spp_velocity[3]; /**< SPP computed ECEF velocity [m/s] */
  bool spp_velocity_valid; /**< True if spp_velocity contains valid data */
  u8 spp_velocity_num_sats; /**< Number of sats used in SPP velocity solution */

  dops_t spp_dops; /**< SPP dilution of precision values */
  bool spp_dops_valid; /**< True if spp_dopscontains valid data */

  double base_position[3]; /**< Base station ECEF position [m] */
  bool base_position_valid; /**< True if base_position contains valid data */

  double rtk_baseline[3]; /**< RTK baseline vector in ECEF frame [m] */
  bool rtk_baseline_valid; /**< True if rtk_baseline contains valid data */
  bool rtk_baseline_fixed; /**< True if RTK solution is fixed, false if float */
  u8 rtk_baseline_num_sats; /**< Number of sats used in RTK baseline solution */
  double rtk_base_age; /**< RTK base observations age [s] */
  u16 rtk_base_id; /**< RTK base sender ID */

  /* TODO(Leith): in future could add RTK velocity */

  /* TODO(Leith): in future could add RTK DOPs */
} send_solutions_t;

void send_solutions(const send_solutions_t *in);

#endif /* SWIFTNAV_SOLN_OUTPUT_H */
