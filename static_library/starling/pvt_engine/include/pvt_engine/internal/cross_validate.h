/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_INTERNAL_CROSS_VALIDATE_H
#define LIBSWIFTNAV_PVT_ENGINE_INTERNAL_CROSS_VALIDATE_H

#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/configuration.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

namespace ambiguities {

PRC validate_number(const TransformedIntegerAmbiguities &ambs,
                    const CrossValidationConfiguration &config);

PRC reduce_snapshots(const TransformedIntegerAmbiguities snapshots[],
                     s32 n_snapshots, TransformedIntegerAmbiguities *reduced);

}  // namespace ambiguities

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_INTERNAL_CROSS_VALIDATE_H
