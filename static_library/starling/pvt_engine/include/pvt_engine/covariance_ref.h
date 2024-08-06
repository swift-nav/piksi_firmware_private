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

#ifndef LIBSWIFTNAV_PVT_ENGINE_COVARIANCE_REF_H
#define LIBSWIFTNAV_PVT_ENGINE_COVARIANCE_REF_H

#include <pvt_common/optional.h>
#include <pvt_engine/covariance_interface.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

// This class contains a reference to the covariance information with
// which it was constructed.
class CovarianceRef : public CovarianceInterface {
 public:
  // U-D-friendly constructor
  CovarianceRef(const TriangularMatrixAsVector &u,
                const VectorMaxStateDimd_t &d);

  // KF-friendly constructor
  explicit CovarianceRef(const MatrixMaxStateDimd_t &p);

  MatrixMaxStateDimd_t to_covariance_matrix() const override;
  void to_covariance_matrix(MatrixMaxStateDimd_t *M) const override;
  s32 size() const override;
  double get_single_covariance(const s32 row, const s32 col) const override;

  const MatrixMaxStateDimd_t &get_P() const override;
  const TriangularMatrixAsVector &get_U() const override;
  const VectorMaxStateDimd_t &get_D() const override;

 private:
  // All data members are declared optional because we can't have
  // an uninitialized reference (and optional is no worse than a
  // pointer).
  //
  // See
  // https://github.com/swift-nav/estimation_team_planning/issues/690
  // for some more detail on the way this is all represented.
  //
  // This holds P in the case of a vanilla KF
  optional<const MatrixMaxStateDimd_t &> P;
  // This holds U in the case of a UD filter.
  optional<const TriangularMatrixAsVector &> U;
  // This holds D in the case of a UD filter and anything you want in
  // the case of a Kalman filter.
  optional<const VectorMaxStateDimd_t &> D;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_COVARIANCE_REF_H
