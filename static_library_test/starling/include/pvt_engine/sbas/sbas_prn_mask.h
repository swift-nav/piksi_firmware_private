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

#ifndef LIBSWIFTNAV_PVT_ENGINE_SBAS_PRN_MASK_H
#define LIBSWIFTNAV_PVT_ENGINE_SBAS_PRN_MASK_H

#include <swiftnav/bits.h>
#include <swiftnav/gnss_time.h>

#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/sat_identifier.h>
#include <pvt_engine/sbas/internal/sbas_prn_mask_data.h>
#include <pvt_engine/sbas/sbas.h>

namespace pvt_engine {

class SBASPrnMask {
 public:
  SBASPrnMask &operator=(const SBASPrnMask &other);

  void clear_all_data(void);

  PRC decode_msg(const SBASRawData &message);

  SatIdentifier get_prn_from_slot(const u8 &slot_id, const u8 &IODP,
                                  const gps_time_t &epoch_time) const;

 private:
  SatIdentifier index2sat(const u16 &i) const;

  PrnMask sbas_prn_mask[IODP_MAX + 1];  // indexed by IODP 0-3

  bool prn_mask_has_aged(const gps_time_t &time, const u8 &IODP) const;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_SBAS_PRN_MASK_H
