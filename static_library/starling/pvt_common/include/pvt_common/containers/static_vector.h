/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE'
 * which must be distributed together with this source. All other
 * rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
 * ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#ifndef PVT_COMMON_CONTAINERS_STATIC_VECTOR_H
#define PVT_COMMON_CONTAINERS_STATIC_VECTOR_H

#include "pvt_common/eigen_custom.h"

namespace pvt_common {
namespace containers {

template <typename T, uint32_t max_size>
using StaticVector = Eigen::Array<T, Eigen::Dynamic, 1, 0, max_size, 1>;

}  // namespace containers
}  // namespace pvt_common

#endif
