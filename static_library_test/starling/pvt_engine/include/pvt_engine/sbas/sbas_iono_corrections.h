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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SBAS_IONO_CORRECTIONS_H
#define LIBSWIFTNAV_PVT_ENGINE_SBAS_IONO_CORRECTIONS_H

#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#include <pvt_common/containers/map.h>
#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sbas/internal/sbas_common.h>
#include <pvt_engine/sbas/sbas.h>

namespace pvt_engine {

class SBASIonoCorrections {
 public:
  explicit SBASIonoCorrections();

  PRC decode_msg(const SBASRawData &message);

  optional<double> calc_L1_iono_correction(
      const gps_time_t &epoch_time, const Eigen::Vector3d &user_pos_llh,
      const AzimuthElevation &sat_az_el) const;

  void clear_all_iono_data();

 private:
  optional<IonoGridPoint> get_igp(const gps_time_t &time, const u8 &band,
                                  const u8 &iodi, const u8 &idx) const;
  PRC decode_msg_18(const SBASRawData &raw);
  PRC decode_msg_26(const SBASRawData &raw);
  IonoGridPointMask sbas_igp_list[4];  // indexed by IODI 0-3
  IonoGrid iono_grid;
  bool igp_band_mask_has_aged(const gps_time_t &time, const u8 &band,
                              const u8 &iodi) const;
  bool igp_grid_has_aged(const gps_time_t &time,
                         const IonoGridPoint &igp) const;
  s8 calc_band_0_to_8_lat_deg(const s8 &lat_start_deg, const u8 &idx_start,
                              const u8 &idx) const;
  s8 calc_band_9_to_10_lat_deg(const s8 &lat_start_deg, const s8 &lat_delta_deg,
                               const u8 &row) const;
  IonoGridPoint band_0_to_8_igp(const u8 &band, const u8 &idx) const;
  IonoGridPoint band_9_to_10_igp(const u8 &band, const u8 &idx) const;
  IonoGridPoint idx2igp(const u8 &band, const u8 &idx) const;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SBAS_IONO_CORRECTIONS_H
