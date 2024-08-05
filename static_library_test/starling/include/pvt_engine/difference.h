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

#ifndef LIBSWIFTNAV_PVT_ENGINE_DIFFERENCE_H
#define LIBSWIFTNAV_PVT_ENGINE_DIFFERENCE_H

#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/observation.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/propagate.h>

namespace pvt_engine {

namespace difference {

optional<Observation> single(const ObservationIdentifier &obs_id,
                             const Observation &rover,
                             const Observation &reference,
                             const EphemerisHandler &eph_handler);

}  // namespace difference

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_DIFFERENCE_H
