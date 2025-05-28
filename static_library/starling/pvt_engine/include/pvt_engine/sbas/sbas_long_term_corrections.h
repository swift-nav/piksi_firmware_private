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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SBAS_LONG_TERM_CORRECTIONS_H
#define LIBSWIFTNAV_PVT_ENGINE_SBAS_LONG_TERM_CORRECTIONS_H

#include <swiftnav/gnss_time.h>

#include <pvt_common/containers/map.h>
#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/sbas/internal/sbas_long_term_corrections_data.h>
#include <pvt_engine/sbas/sbas.h>
#include <pvt_engine/sbas/sbas_prn_mask.h>

namespace pvt_engine {

// WAAS Signal Specification states that message type 25 is split to two halfs.
// Each half can contain one detailed or two less detailed data packages. Quote
// from the specification: "the message can consist of error estimates for
// 1, 2, 3 or 4 satellites".
//
// Possible combinations for type 25:
// 0 = velocity code 0 (1 quarter)
// 1 = velocity code 1 (always 1 quarter pair ie. half message)
// x = empty quarter
//
// x-x-x-x
// 0-x-x-x
// 0-0-x-x
// 0-0-0-x
// 0-0-0-0
// 0-0-1-1
// 1-1-x-x
// 1-1-0-x
// 1-1-0-0
// 1-1-1-1
//
// Message type 24 contains long term corrections only in the latter half.
// First half contains fast corrections.
//
// This iterator-like enum class can be used to indicate which message quarter
// is processed and which is next in line. Notice the prefix and postfix ++
// operators implemented in the .cc file.
enum class LongTermMessageQuarter { FIRST, SECOND, THIRD, FOURTH, END };

using LongTermCorrectionsMap =
    pvt_common::containers::Map<SatIdentifier, LongTermCorrection,
                                cMaxSbasAugmentedSatellites>;

class SBASLongTermCorrections {
 public:
  explicit SBASLongTermCorrections();

  void clear_all_data(void);

  PRC decode_msg(const SBASRawData &message, const SBASPrnMask &mask);

  optional<double> get_long_term_range_correction(
      const SatIdentifier &sat, const gps_time_t &epoch_time, const u16 &IODE,
      const Eigen::Vector3d &line_of_sight_ecef) const;

  optional<double> get_long_term_range_rate_correction(
      const SatIdentifier &sat, const gps_time_t &epoch_time, const u16 &IODE,
      const Eigen::Vector3d &line_of_sight_ecef) const;

  bool has_long_term_corrections(const SatIdentifier &sat,
                                 const gps_time_t &epoch_time,
                                 const u16 &IODE) const;

 private:
  LongTermCorrectionsMap sbas_long_term_corrections_current_;
  LongTermCorrectionsMap sbas_long_term_corrections_previous_;

  PRC decode_eigen_vector(const SBASRawData &msg, const u8 &idx,
                          const u8 &element_len, const double &factor,
                          Eigen::Vector3d *vctr) const;
  PRC decode_vel_code0(const SBASRawData &message,
                       const LongTermMessageQuarter &quarter,
                       LongTermCorrection *corrections, u8 *prn_mask_slot,
                       u8 *iodp) const;
  PRC decode_vel_code1_toa(const SBASRawData &message,
                           const u8 &half_msg_start_idx,
                           LongTermCorrection *corrections) const;
  PRC decode_vel_code1(const SBASRawData &message,
                       const LongTermMessageQuarter &quarter,
                       LongTermCorrection *corrections, u8 *prn_mask_slot,
                       u8 *iodp) const;
  PRC decode_quarters(const SBASRawData &message, const SBASPrnMask &mask,
                      LongTermMessageQuarter *quarter);

  PRC save_corrections(const SatIdentifier &sat,
                       const LongTermCorrection &corrections);
  optional<const LongTermCorrection &> find_corrections(
      const SatIdentifier &sat, const gps_time_t &epoch_time,
      const u16 &IODE) const;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SBAS_LONG_TERM_CORRECTIONS_H
