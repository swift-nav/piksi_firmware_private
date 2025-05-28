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

#ifndef LIBSWIFTNAV_PVT_ENGINE_FILTER_UTILS_H
#define LIBSWIFTNAV_PVT_ENGINE_FILTER_UTILS_H

#include <swiftnav/common.h>

#include <pvt_engine/eigen_types.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>

namespace pvt_engine {

namespace utils {

template <typename vector_type, typename value_type>
PRC add_vector_element(vector_type *x, const s32 idx, const value_type value) {
  if (idx < 0 || idx > x->size()) {
    PRC rc(RC_E_STATE_DOESNT_EXIST);
    log_error("%s", prc::message(rc));
    return rc;
  }
  const vector_type tail_elements = x->tail(x->size() - idx);
  x->conservativeResize(x->size() + 1);
  x->tail(x->size() - idx - 1) = tail_elements;
  (*x)[idx] = value;
  return RC_S_OK;
}

template <typename matrix_type>
PRC add_matrix_row(matrix_type *mat, const s32 idx_row) {
  if (idx_row < 0 || idx_row > mat->rows()) {
    PRC rc(RC_E_STATE_DOESNT_EXIST);
    log_error("%s", prc::message(rc));
    return rc;
  }
  const matrix_type bottom_rows = mat->bottomRows(mat->rows() - idx_row);
  mat->conservativeResize(mat->rows() + 1, Eigen::NoChange_t());
  mat->bottomRows(mat->rows() - idx_row - 1) = bottom_rows;
  mat->row(idx_row).setZero();
  return RC_S_OK;
}

template <typename matrix_type>
PRC add_matrix_col(matrix_type *mat, const s32 idx_col) {
  if (idx_col < 0 || idx_col > mat->cols()) {
    PRC rc(RC_E_STATE_DOESNT_EXIST);
    log_warn("%s", prc::message(rc));
    return rc;
  }
  const matrix_type right_cols = mat->rightCols(mat->cols() - idx_col);
  mat->conservativeResize(Eigen::NoChange_t(), mat->cols() + 1);
  mat->rightCols(mat->cols() - idx_col - 1) = right_cols;
  mat->col(idx_col).setZero();
  return RC_S_OK;
}

template <typename matrix_type, typename value_type>
PRC add_matrix_row_col_value(matrix_type *mat, const s32 idx_row,
                             const s32 idx_col, const value_type value) {
  PRC rc = add_matrix_row(mat, idx_row);
  if (!prc::success(rc)) {
    return rc;
  }
  rc = add_matrix_col(mat, idx_col);
  if (!prc::success(rc)) {
    return rc;
  }
  (*mat)(idx_row, idx_col) = value;
  return RC_S_OK;
}

template <typename vector_type>
PRC remove_vector_element(vector_type *x, s32 idx) {
  if (idx < 0 || idx >= x->size()) {
    PRC rc(RC_E_STATE_DOESNT_EXIST);
    log_warn("%s", prc::message(rc));
    return rc;
  }

  for (s32 next = idx + 1; next < x->size(); ++idx, ++next) {
    (*x)(idx) = (*x)(next);
  }
  x->conservativeResize(x->size() - 1);
  return RC_S_OK;
}

// This function seems to have some problems triggering a
// strict-overflow warning under GCC 5 in some places.  Other
// compilers don't seem upset.
template <typename matrix_type>
PRC remove_matrix_row(matrix_type *mat, const s32 idx_row) {
  if (idx_row < 0 || idx_row >= mat->rows()) {
    PRC rc(RC_E_STATE_DOESNT_EXIST);
    log_warn("%s", prc::message(rc));
    return rc;
  }
  const matrix_type bottom_rows = mat->bottomRows(mat->rows() - idx_row - 1);
  mat->conservativeResize(mat->rows() - 1, Eigen::NoChange_t());
  mat->bottomRows(mat->rows() - idx_row) = bottom_rows;
  return RC_S_OK;
}

template <typename matrix_type>
PRC remove_matrix_col(matrix_type *mat, const s32 idx_col) {
  if (idx_col < 0 || idx_col >= mat->cols()) {
    PRC rc(RC_E_STATE_DOESNT_EXIST);
    log_warn("%s", prc::message(rc));
    return rc;
  }
  const matrix_type right_cols = mat->rightCols(mat->cols() - idx_col - 1);
  mat->conservativeResize(Eigen::NoChange_t(), mat->cols() - 1);
  mat->rightCols(mat->cols() - idx_col) = right_cols;
  return RC_S_OK;
}

template <typename matrix_type>
PRC remove_matrix_row_col(matrix_type *mat, const s32 idx_row,
                          const s32 idx_col) {
  PRC rc = remove_matrix_row(mat, idx_row);
  if (!prc::success(rc)) {
    return rc;
  }
  rc = remove_matrix_col(mat, idx_col);
  if (!prc::success(rc)) {
    return rc;
  }
  return RC_S_OK;
}

template <typename vector_type, typename index_type>
PRC get_sub_vector(const vector_type &vec_in, const index_type &idx,
                   vector_type *vec_out) {
  assert(vec_out != nullptr);
  const s32 n = idx.size();
  *vec_out = vector_type::Zero(n);
  for (s32 i = 0; i < n; ++i) {
    const s32 idx_i = static_cast<s32>(idx(i));
    (*vec_out)(i) = vec_in(idx_i);
  }
  return RC_S_OK;
}

template <typename matrix_type, typename index_type>
PRC get_sub_matrix(const matrix_type &mat_in, const index_type &idx,
                   matrix_type *mat_out) {
  assert(mat_out != nullptr);
  const s32 n = idx.size();
  *mat_out = matrix_type::Zero(n, n);
  for (s32 i = 0; i < n; ++i) {
    const s32 idx_i = static_cast<s32>(idx(i));
    for (s32 j = i; j < n; ++j) {
      const s32 idx_j = static_cast<s32>(idx(j));
      (*mat_out)(j, i) = (*mat_out)(i, j) = mat_in(idx_i, idx_j);
    }
  }
  return RC_S_OK;
}

template <typename vector_type>
PRC find_first_index(vector_type *x, const s32 value, s32 *index) {
  assert(x != nullptr);
  assert(index != nullptr);
  PRC rc(RC_E_STATE_DOESNT_EXIST);
  *index = INVALID_INDEX;
  s32 num_elements = (*x).size();
  for (s32 idx = 0; idx < num_elements; idx++) {
    if (value == (*x)(idx)) {
      *index = idx;
      rc = RC_S_OK;
      break;
    }
  }

  return rc;
}

template <typename matrix_type>
void get_unique_rows(const matrix_type &x, matrix_type *x_unique,
                     double tolerance) {
  // TODO(https://github.com/swift-nav/estimation_team_planning/issues/566)
  assert(x_unique != nullptr);
  x_unique->conservativeResize(x.rows(), x.cols());
  s32 x_unique_index(0);

  for (s32 x_index = 0; x_index < x.rows(); x_index++) {
    bool add_row(true);
    for (s32 valid_rows = 0; valid_rows < x_unique->rows(); valid_rows++) {
      if (x_unique->row(valid_rows).isApprox(x.row(x_index), tolerance)) {
        add_row = false;
        continue;
      }
    }

    if (add_row) {
      x_unique->row(x_unique_index) = x.row(x_index);
      x_unique_index++;
    }
  }
  x_unique->conservativeResize(x_unique_index, Eigen::NoChange);
}

}  // namespace utils
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FILTER_UTILS_H
