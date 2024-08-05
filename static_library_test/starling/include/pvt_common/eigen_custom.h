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

#ifndef LIBSWIFTNAV_PVT_ENGINE_EIGEN_H
#define LIBSWIFTNAV_PVT_ENGINE_EIGEN_H

/* Top-level wrapper for Eigen library.
 * Defines configuration options and suppresses warnings for includes.
 */

#include <swiftnav/common.h>
#include <type_traits>

// do not re-order! Fragile.
#include "newlib_workarounds.h"

#if defined(__ARM_EABI__)
#pragma GCC system_header
#endif

#ifndef NETWORK
#define EIGEN_NO_MALLOC
#endif
// Don't run Eigen debug code in order to reduce CPU load
// NB! This will elimiate bounds checking on matrcies and arrays.
// TODO (Martin): ORI-663 move to non-network builds only after CPU load is
// assessed.
#define EIGEN_NO_DEBUG

// TODO(https://github.com/swift-nav/estimation_team_planning/issues/223)
#if defined(__ARM_EABI__)
#define EIGEN_DONT_VECTORIZE
#endif

#define EIGEN_DEFAULT_DENSE_INDEX_TYPE s32
#define EIGEN_STACK_ALLOCATION_LIMIT 10000000
#define EIGEN_ARRAY_PLUGIN "pvt_common/eigen_array_base_addons.h"

namespace Eigen {
template <typename MatrixType>
class FractionFreeLU;
}

#define EIGEN_MATRIXBASE_PLUGIN "pvt_common/eigen_matrix_base_addons.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace pvt_common {
namespace type_traits {
template <typename T, typename = void>
struct is_dynamic_sized : std::false_type {};

template <typename T>
struct is_dynamic_sized<
    T, std::enable_if_t<T::RowsAtCompileTime == Eigen::Dynamic ||
                        T::ColsAtCompileTime == Eigen::Dynamic>>
    : std::true_type {};

template <typename T>
static constexpr bool is_dynamic_sized_v =
    is_dynamic_sized<std::remove_reference_t<T>>::value;

}  // namespace type_traits
}  // namespace pvt_common

#endif  // LIBSWIFTNAV_PVT_ENGINE_EIGEN_H
