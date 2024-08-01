/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_SBAS_CORRECTIONS_MANAGER_H
#define LIBSWIFTNAV_PVT_ENGINE_SBAS_CORRECTIONS_MANAGER_H

#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sbas/sbas.h>
#include <pvt_engine/sbas/sbas_fast_corrections.h>
#include <pvt_engine/sbas/sbas_iono_corrections.h>
#include <pvt_engine/sbas/sbas_long_term_corrections.h>
#include <pvt_engine/sbas/sbas_prn_mask.h>

namespace pvt_engine {

class SBASCorrectionsManager {
 public:
  explicit SBASCorrectionsManager();

  PRC process_sbas_message(const SBASRawData &message);

  SBASCorrections get_sbas_corrections(const gnss_signal_t &sid,
                                       const gps_time_t &epoch_time,
                                       const u8 &IODE,
                                       const Eigen::Vector3d &user_pos_ecef,
                                       const Eigen::Vector3d &user_pos_llh,
                                       const Eigen::Vector3d &sat_pos_ecef,
                                       const AzimuthElevation &sat_az_el) const;

  bool has_sbas_corrections(const SatIdentifier &sat,
                            const gps_time_t &epoch_time, const u8 &IODE) const;

  // How long after msg#0 data is accepted again
  static constexpr u8 cmsg0_cooldowns_s = 60;
  // After receiving msg#0, how much history needs to be cleared
  static constexpr u16 cmsg0_clear_range_s = 300;

 private:
  void clear_all_data();

  PRC process_msg0(const SBASRawData &message);
  bool is_msg0_cooldown_in_progress(const SBASRawData &message);

  SBASPrnMask prn_mask_;
  SBASFastCorrections fast_corrections_;
  SBASLongTermCorrections long_term_corrections_;
  SBASIonoCorrections iono_corrections_;

  pvt_common::containers::Map<gnss_signal_t, gps_time_t, NUM_SATS_SBAS>
      time_of_prev_msg_;
  pvt_common::containers::Map<gnss_signal_t, gps_time_t, NUM_SATS_SBAS>
      time_of_msg0_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SBAS_CORRECTIONS_MANAGER_H
