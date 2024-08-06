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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SBAS_FAST_CORRECTIONS_H
#define LIBSWIFTNAV_PVT_ENGINE_SBAS_FAST_CORRECTIONS_H

#include <swiftnav/bits.h>
#include <swiftnav/gnss_time.h>

#include <pvt_common/containers/map.h>
#include <pvt_common/optional.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/sbas/internal/sbas_fast_corrections_data.h>
#include <pvt_engine/sbas/sbas.h>
#include <pvt_engine/sbas/sbas_prn_mask.h>

namespace pvt_engine {

class SBASFastCorrections {
 public:
  explicit SBASFastCorrections();

  void clear_all_data(void);

  PRC decode_msg(const SBASRawData &message, const SBASPrnMask &prn_mask);

  optional<double> get_fast_correction(const SatIdentifier &sat,
                                       const gps_time_t &epoch_time) const;

 private:
  pvt_common::containers::Map<SatIdentifier, FastCorrection,
                              cMaxSbasAugmentedSatellites>
      sbas_fast_corrections;

  bool fast_correction_has_aged(const FastCorrection &corr,
                                const gps_time_t &epoch_time) const;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SBAS_FAST_CORRECTIONS_H
