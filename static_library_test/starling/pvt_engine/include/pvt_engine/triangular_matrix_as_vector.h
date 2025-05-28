/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_TRIANGULAR_MATRIX_AS_VECTOR_H
#define LIBSWIFTNAV_PVT_ENGINE_TRIANGULAR_MATRIX_AS_VECTOR_H

#include <ostream>

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {
using TriangularVector_t =
    Eigen::Matrix<double, Eigen::Dynamic, 1, 0,
                  (cMaxStateDim * (cMaxStateDim + 1)) / 2, 1>;

// This is a class to represent an upper triangular matrix as a vector,
// meaning that only the values corresponding to elements in an upper
// triangular are stored. This enables size reductions in comparison with
// using a matrix which also stores the zeros corresponding to a lower
// triangular matrix.
class TriangularMatrixAsVector {
 public:
  TriangularMatrixAsVector();
  TriangularMatrixAsVector(const MatrixMaxStateDimd_t &M);  // NOLINT

  double &operator()(const s32 row, const s32 col);
  const double *operator()(const s32 row, const s32 col) const;

  // The following methods are used to multiply a triangular vector U with a
  // matrix or vector respectively, from the left hand side.
  // Example: F * U or h * U
  MatrixMaxStateDimd_t pre_multiply_matrix(const MatrixMaxStateDimd_t &m) const;
  VectorMaxStateDimd_t pre_multiply_vector(const VectorMaxStateDimd_t &v) const;
  // The following methods are used to multiply a triangular vector U with a
  // matrix or vector respectively, from the right hand side.
  // Example: U * F or U * h
  MatrixMaxStateDimd_t post_multiply_matrix(
      const MatrixMaxStateDimd_t &m) const;
  VectorMaxStateDimd_t post_multiply_vector(
      const VectorMaxStateDimd_t &v) const;
  // This specific method to calculate UD uses the triangular vector U and a
  // vector D which is interpreted as a diagonal matrix.
  MatrixMaxStateDimd_t calc_UD(const VectorMaxStateDimd_t &v) const;
  // Use this method to add a new state along the diagonal at the specific
  // index.
  PRC add_state(const s32 idx, const double value);
  // Fill the triangular vector U from a matrix.
  void fill(const MatrixMaxStateDimd_t &M);
  // Return the diagonal of the triangular vector U.
  VectorMaxStateDimd_t diagonal() const;
  // Return the triangular vector U with ones along the diagonal.
  TriangularMatrixAsVector unitUpper() const;
  // Return the row specified by the input argument `row` of the triangular
  // vector U.
  VectorMaxStateDimd_t row(const s32 row) const;
  // Replace the diagonal of the triangular vector U with a vector.
  void replace_diag(const VectorMaxStateDimd_t &v);
  // The following methods are copies of Eigen methods to support the
  // same functionality.
  void setIdentity(const s32 rows, const s32 cols);
  void setZero(const s32 rows, const s32 cols);
  void resize(const s32 rows, const s32 cols);
  void conservativeResize(const s32 rows, const s32 cols);
  s32 rows() const { return dim_; }
  s32 cols() const { return dim_; }

  MatrixMaxStateDimd_t get_matrix() const;
  TriangularVector_t get_vector() const { return upper_; }

 private:
  s32 dim_;
  TriangularVector_t upper_;
};

inline std::ostream &operator<<(std::ostream &os,
                                const TriangularMatrixAsVector &TV) {
  os << TV.get_vector();
  return os;
}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_TRIANGULAR_MATRIX_AS_VECTOR_H
