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

#ifndef LIBSWIFTNAV_CONSTELLATION_FREQUENCY_H
#define LIBSWIFTNAV_CONSTELLATION_FREQUENCY_H
#include <pvt_engine/pvt_types.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

struct ConstellationFrequency {
  constellation_t constellation;
  pvt_engine::FREQUENCY frequency;
  bool operator<(const ConstellationFrequency &rhs) const {
    if (constellation == rhs.constellation) {
      return frequency < rhs.frequency;
    }
    return constellation < rhs.constellation;
  }
  bool operator==(const ConstellationFrequency &rhs) const {
    return constellation == rhs.constellation && frequency == rhs.frequency;
  }
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_CONSTELLATION_FREQUENCY_H
