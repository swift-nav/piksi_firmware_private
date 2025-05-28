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

#ifndef LIBSWIFTNAV_PVT_ENGINE_COVARIANCE_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_COVARIANCE_INTERFACE_H

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_types.h>
#include <pvt_engine/triangular_matrix_as_vector.h>

namespace pvt_engine {

class CovarianceInterface {
 public:
  explicit CovarianceInterface(const ESTIMATOR_TYPE &type)
      : filter_type(type) {}
  virtual s32 size() const = 0;
  ESTIMATOR_TYPE get_filter_type() const { return filter_type; }
  virtual const MatrixMaxStateDimd_t &get_P() const = 0;
  virtual const TriangularMatrixAsVector &get_U() const = 0;
  VectorMaxObsd_t get_HPHt_diag(const MatrixMaxObsByStatesd_t &H) const {
    switch (filter_type) {
      case KALMAN_FILTER:
        return (H * get_P() * H.transpose()).diagonal();
      case UD_FILTER: {
        const MatrixMaxObsByStatesd_t HU(get_U().pre_multiply_matrix(H));
        return (HU * get_D().asDiagonal() * HU.transpose()).diagonal();
      }
      case NO_FILTER:
      case MAX_ESTIMATOR_TYPE:
      default:
        assert(false);
        exit(EXIT_FAILURE);
    }
  }
  virtual const VectorMaxStateDimd_t &get_D() const = 0;
  virtual double get_single_covariance(const s32 row, const s32 col) const = 0;
  virtual MatrixMaxStateDimd_t to_covariance_matrix() const = 0;
  virtual void to_covariance_matrix(MatrixMaxStateDimd_t *M) const = 0;

  virtual ~CovarianceInterface() = default;

 protected:
  ESTIMATOR_TYPE filter_type;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_COVARIANCE_INTERFACE_H
