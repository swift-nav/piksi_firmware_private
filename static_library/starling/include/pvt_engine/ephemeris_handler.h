/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_EPHEMERIS_HANDLER_H
#define LIBSWIFTNAV_EPHEMERIS_HANDLER_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sat_identifier.h>
#include <swiftnav/ephemeris.h>

namespace pvt_engine {

class EphemerisHandler {
 public:
  EphemerisHandler() : ephemeris_map_() {}

  void clear();

  void overwrite_ephemerides(s16 num_ephs, const ephemeris_t *stored_ephs[]);

  optional<SatPVA> calc_sat_PVA(const gnss_signal_t &sid,
                                const gps_time_t *t) const;

 private:
  using EphemerisMap =
      pvt_common::containers::Map<SatIdentifier, ephemeris_t, NUM_SATS>;
  EphemerisMap ephemeris_map_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_EPHEMERIS_HANDLER_H
