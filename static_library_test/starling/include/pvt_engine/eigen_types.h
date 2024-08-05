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

#ifndef LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_H
#define LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_H

#include <pvt_common/eigen_custom.h>
#include <starling/config.h>
#include <swiftnav/signal.h>

#ifdef NETWORK
#include <pvt_engine/eigen_types_sizes_network.h>
#else
#include <pvt_engine/eigen_types_sizes.h>
#endif

namespace pvt_engine {

/** \defgroup eigen_types Eigen Types
 * Defines some custom max fixed sized matrix and vector types dependent on
 * the number of observations and state dimensions expected in the PVT engine.
 *
 * Note that Eigen actually defines array access and size in terms of
 * Eigen::Index, which is a signed type. As such, the template parameters and
 * constexpr expressions below are left as ints.
 *
 **/

template <typename T, int N>
using Vector = Eigen::Matrix<T, N, 1>;
template <int N>
using Vectord = Vector<double, N>;
template <typename T, int M, int N>
using MatrixMaxX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, M, N>;
template <int M, int N>
using MatrixMaxXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, M, N>;
template <typename T, int N>
using VectorMaxX = Eigen::Matrix<T, Eigen::Dynamic, 1, 0, N, 1>;
template <int N>
using VectorMaxXd = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, N, 1>;

using GeometryMatrix = Eigen::Matrix<double, Eigen::Dynamic, NUM_G_COLS, 0,
                                     cMaxFilterTrackingChannels, NUM_G_COLS>;

using VectorMaxBaselineStatesd_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxBaselineStates, 1>;
using VectorMaxBaselines32_t =
    Eigen::Matrix<s32, Eigen::Dynamic, 1, 0, cMaxBaselineStates, 1>;
using MatrixMaxBaselineStatesd_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, cMaxBaselineStates,
                  cMaxBaselineStates>;

using VectorMaxAmbsd_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxAmbiguities, 1>;
using VectorMaxAmbss32_t =
    Eigen::Matrix<s32, Eigen::Dynamic, 1, 0, cMaxAmbiguities, 1>;
using MatrixMaxAmbsd_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       0, cMaxAmbiguities, cMaxAmbiguities>;

using MatrixMaxAmbss32_t = Eigen::Matrix<s32, Eigen::Dynamic, Eigen::Dynamic, 0,
                                         cMaxAmbiguities, cMaxAmbiguities>;

using VectorMaxObsd_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxFilterObservations, 1>;
using MatrixMaxObsd_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0,
                  cMaxFilterObservations, cMaxFilterObservations>;

using VectorMaxStateDimd_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxStateDim, 1>;
using MatrixMaxStateDimd_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, cMaxStateDim,
                  cMaxStateDim>;
using VectorMaxStateDims32_t =
    Eigen::Matrix<s32, Eigen::Dynamic, 1, 0, cMaxStateDim, 1>;

constexpr s32 cMaxDim = MAX(cMaxFilterObservations, 2 * cMaxStateDim);
template <typename T>
using VectorMaxSize_t = Eigen::Matrix<T, Eigen::Dynamic, 1, 0, cMaxDim, 1>;
template <typename T>
using MatrixMaxSize_t =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, cMaxDim, cMaxDim>;
using VectorMaxSized_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxDim, 1>;
using MatrixMaxSized_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, cMaxDim, cMaxDim>;

using MatrixMaxAmbiguitySetDimd_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, cMaxStateDim,
                  cMaxIntegerAmbiguitySets>;
using VectorMaxAmbiguitySetDimd_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0, cMaxIntegerAmbiguitySets, 1>;

/** For H (observation model) matrix and its pseudoinverse **/
using MatrixMaxObsByStatesd_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0,
                  cMaxFilterObservations, cMaxStateDim>;

using MatrixMaxStatesByObsd_t =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, cMaxStateDim,
                  cMaxFilterObservations>;

template <typename T, int M, int N>
using RMMatrix = Eigen::Matrix<T, M, N, Eigen::RowMajor>;
template <typename T>
using RMMatrixX =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
template <int M, int N>
using RMMatrixd = Eigen::Matrix<double, M, N, Eigen::RowMajor>;

namespace containers {
template <typename T, s32 max_size>
using StaticVector = Eigen::Array<T, Eigen::Dynamic, 1, 0, max_size, 1>;
}  // namespace containers

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_EIGEN_TYPES_H
