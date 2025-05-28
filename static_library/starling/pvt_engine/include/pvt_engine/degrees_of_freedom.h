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

#ifndef LIBSWIFTNAV_DEGREES_OF_FREEDOM_H
#define LIBSWIFTNAV_DEGREES_OF_FREEDOM_H

#include <pvt_common/containers/map.h>
#include <pvt_common/containers/set.h>
#include <pvt_engine/observation.h>

namespace pvt_engine {

class DoF_container {
 public:
  DoF_container();
  DoF_container(const DoF_container &rhs) = default;
  DoF_container &operator-=(const DoF_container &rhs);
  DoF_container &operator+=(const DoF_container &rhs);
  DoF_container &operator*=(const double &rhs);
  DoF_container operator*(const double &rhs) const;

  // Degrees of freedom can be from states or observations...
  void add(const ObservationIdentifier &id, double value);
  void clear();
  double get_DoF(
      const pvt_common::containers::Set<constellation_t, CONSTELLATION_COUNT>
          &constellations) const;

 private:
  double general_DoF_;
  pvt_common::containers::Map<constellation_t, double, CONSTELLATION_COUNT>
      constellation_DoF_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_DEGREES_OF_FREEDOM_H
