/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_ESTIMATION_STATE_MANAGEMENT_H
#define STARLING_ESTIMATION_STATE_MANAGEMENT_H

#include <array>
#include <cstddef>
#include <numeric>

#include <pvt_common/containers/static_vector.h>
#include <pvt_common/eigen_custom.h>
#include <pvt_common/functional/fold.h>
#include <sensorfusion/core/error_types.h>

#include <math_routines/kalman/UD.h>

namespace sensorfusion {

namespace detail {

template <typename T, typename = void>
struct Resizer {
  Resizer(int rows, int cols, T *M) {}
};

template <typename T>
struct Resizer<T, std::enable_if_t<pvt_common::type_traits::is_dynamic_sized<
                      std::decay_t<T>>::value>> {
  Resizer(int rows, int cols, T *M) { M->conservativeResize(rows, cols); }
};

template <typename T>
void typesafe_resize(int rows, int cols, T *M) {
  Resizer<T>(rows, cols, M);
}

}  // namespace detail

// This could be made into a template parameter for `StateManager` and
// `StateManagerFactory`
static constexpr std::size_t MAX_DIM = 32;

enum class StateMode {
  Estimate = 0,
  Consider = 1,
  Exclude = 2,
};

struct StateHandle {
  std::size_t index;
};

class StateManagerFactory;

/**
 * Manager for filter state stored in UD format.
 *
 * @brief This class provides storage and management functionality for the mean
 * and covariance values for a set of variables.
 *
 * @details This class allows access to the mean and covariance values for
 * arbitrary sets of filter variables. You're able to request the mean and
 * covariance values separately or together using the `get_mean()`, `get_cov()`,
 * and `get()` functions.
 *
 *
 * Interior covariance matrix format:
 *   _________________________________
 *   |          |          |         |
 *   | Estimate |          |         |
 *   |__________|          |         |
 *   |                     |         |
 *   |                     |         |
 *   |            Consider |         |
 *   |_____________________|         |
 *   |                               |
 *   |                               |
 *   |                       Exclude |
 *   |_______________________________|
 */
class StateManager {
  friend StateManagerFactory;
  class VariableInfo {
    StateMode mode_;
    std::size_t dimension_;
    std::size_t offset_;

   public:
    VariableInfo() : mode_(StateMode::Exclude), dimension_(0), offset_(0) {}
    VariableInfo(StateMode mode, std::size_t offset, std::size_t dimension)
        : mode_(mode), dimension_(dimension), offset_(offset) {}
    bool is_active() const { return mode_ != StateMode::Exclude; }
    bool is_inactive() const { return !is_active(); }

    StateMode mode() const { return mode_; }

    void set_mode(StateMode new_mode) {
      assert(new_mode != mode_);
      mode_ = new_mode;
    }

    std::size_t dimension() const { return dimension_; }

    std::size_t offset() const {
      assert(is_active());
      return offset_;
    }

    void change_offset_by(int delta) { offset_ += delta; }
    void set_offset_to(std::size_t new_value) { offset_ = new_value; }
  };

 public:
  using MeanStorage = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MAX_DIM, 1>;
  using CovStorage = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0,
                                   MAX_DIM, MAX_DIM>;

  ~StateManager() = default;
  StateManager(const StateManager &) = default;
  StateManager(StateManager &&) = default;

  StateManager &operator=(const StateManager &) = default;
  StateManager &operator=(StateManager &&) = delete;

  ValueOrError<StateMode> get_mode(const StateHandle &handle) const {
    if (handle.index >= variable_info_.size()) {
      return Error("Invalid variable handle");
    }
    return variable_info_[handle.index].mode();
  }

  template <typename MeanVectorT, s32 var_max_length>
  MaybeError get_mean(
      const pvt_common::containers::StaticVector<StateHandle, var_max_length>
          &vars,
      MeanVectorT *mean_out) const {
    if (vars.size() > variable_info_.size()) {
      return Error("Too many variables requested");
    }
    if (mean_out->cols() != 1 || mean_out->rows() < vars.size()) {
      return Error("Mean vector incorrectly sized");
    }

    std::size_t output_dim = 0;
    for (const auto &row_handle : vars) {
      if (row_handle.index >= variable_info_.size()) {
        return Error("Invalid variable handle");
      }

      const VariableInfo &row_info = variable_info_[row_handle.index];
      if (row_info.is_inactive()) {
        return Error("Requesting mean of an excluded variable!");
      }

      output_dim += row_info.dimension();
    }

    detail::typesafe_resize(output_dim, 1, mean_out);

    std::size_t output_row_offset = 0;
    for (const auto &row_handle : vars) {
      const VariableInfo &row_info = variable_info_[row_handle.index];
      mean_out->middleRows(output_row_offset, row_info.dimension()) =
          means_.middleRows(row_info.offset(), row_info.dimension());
      output_row_offset += row_info.dimension();
    }

    return Ok();
  }

  /***
   * @brief Gets a subset of the covariance matrix for the specified variables
   *
   * @details Due to the covariance matrix being stored in UD form, this
   * function is a non-trivial operation.
   *
   * @tparam CovMatrixT The output covariance matrix type
   * @tparam var_max_length The maximum length of the variable info vector
   * @param cov_out The covariance matrix to store the output
   * @param vars The selection of variables to get the covariance for
   * @return An error message if something went wrong, `Ok()` otherwise.
   */
  template <typename CovMatrixT, s32 var_max_length>
  MaybeError get_cov(
      const pvt_common::containers::StaticVector<StateHandle, var_max_length>
          &vars,
      CovMatrixT *cov_out) const {
    if (vars.size() > variable_info_.size()) {
      return Error("Too many variables requested");
    }
    if (cov_out->cols() < vars.size() || cov_out->rows() < vars.size() ||
        cov_out->rows() != cov_out->cols()) {
      return Error("Covariance matrix incorrectly sized");
    }

    // First construct an F detailing where
    CovStorage F(MAX_DIM, MAX_DIM);
    F.setZero();
    std::size_t output_offset = 0;
    for (const auto &handle : vars) {
      if (handle.index >= variable_info_.size()) {
        return Error("Invalid variable handle");
      }
      const VariableInfo &info = variable_info_[handle.index];
      if (info.is_inactive()) {
        return Error("Requesting covariance of an excluded variable!");
      }

      F.block(output_offset, info.offset(), info.dimension(), info.dimension())
          .setIdentity();
      output_offset += info.dimension();
    }

    std::size_t col_count = get_active_var_dim();
    F.conservativeResize(output_offset, col_count);

    detail::typesafe_resize(output_offset, output_offset, cov_out);
    reshuffle_matrix(covs_, F, cov_out);

    return Ok();
  }

  template <typename MeanVectorT, typename CovMatrixT, s32 var_max_length>
  MaybeError get(
      const pvt_common::containers::StaticVector<StateHandle, var_max_length>
          &vars,
      MeanVectorT *mean_out, CovMatrixT *cov_out) const {
    MaybeError mean_result = get_mean(vars, mean_out);
    if (mean_result.is_error()) {
      return mean_result;
    }

    MaybeError cov_result = get_cov(vars, cov_out);
    if (cov_result.is_error()) {
      return cov_result;
    }

    return Ok();
  }

  template <typename MeanVectorT, typename CovMatrixT>
  MaybeError set(const MeanVectorT &new_mean, const CovMatrixT &new_cov) {
    if (new_mean.rows() != means_.rows() || new_mean.cols() != 1) {
      return Error("Invalid size of the new Mean vector");
    }
    if (new_cov.rows() != covs_.rows() || new_cov.cols() != covs_.cols()) {
      return Error("Invalid size of the new Covariance matrix");
    }

    means_ = new_mean;
    covs_ = new_cov;
    return Ok();
  }

  template <s32 var_max_length>
  MaybeError activate(
      StateMode newMode,
      const pvt_common::containers::StaticVector<StateHandle, var_max_length>
          &vars,
      double init_mean_value, double init_cov_value) {
    if (newMode == StateMode::Exclude) {
      return Error("Can't activate a variable into the exclude mode");
    }
    if (vars.size() > variable_info_.size()) {
      return Error("Too many variables requested");
    } else if (vars.size() == 0) {
      return Ok();
    }

    // First validate all the input variables before changing anything
    for (const auto &handle : vars) {
      if (handle.index >= variable_info_.size()) {
        return Error("Invalid variable handle");
      } else if (variable_info_[handle.index].mode() != StateMode::Exclude) {
        return Error("Trying to activate an already active variable");
      }
    }

    std::size_t active_size = get_active_var_dim();
    std::size_t init_offset = active_size;

    // Mark all activated vars as consider
    for (const auto &handle : vars) {
      VariableInfo &info = variable_info_[handle.index];
      info.set_mode(StateMode::Consider);
      info.set_offset_to(active_size);
      active_size += info.dimension();
    }

    // Expand the storage appropriately
    means_.conservativeResize(active_size);
    covs_.conservativeResize(active_size, active_size);

    // Set the new spaces with the given initial values
    std::size_t new_vars_size = active_size - init_offset;
    means_.bottomRows(new_vars_size).array() = init_mean_value;
    covs_.bottomRows(new_vars_size).array() = init_cov_value;
    covs_.rightCols(new_vars_size).array() = init_cov_value;

    if (newMode == StateMode::Estimate) {
      return change_mode(newMode, vars);
    } else {
      return Ok();
    }
  }

  template <s32 var_max_length>
  MaybeError deactivate(
      const pvt_common::containers::StaticVector<StateHandle, var_max_length>
          &vars) {
    if (vars.size() > variable_info_.size()) {
      return Error("Too many variables requested");
    } else if (vars.size() == 0) {
      return Ok();
    }

    // First validate all the input variables before changing anything
    for (const auto &handle : vars) {
      if (handle.index >= variable_info_.size()) {
        return Error("Invalid variable handle");
      } else if (variable_info_[handle.index].is_inactive()) {
        return Error("Trying to deactivate a variable that's already inactive");
      }
    }

    std::size_t active_count = get_active_var_count();
    CovStorage F(covs_.rows(), covs_.cols());
    F.setIdentity();

    // Go through all of the given variables and mark each one as excluded
    // At the same time update it's offset to be the top of the excluded area
    std::size_t new_offset = 0;
    for (const auto &handle : vars) {
      VariableInfo &info = variable_info_[handle.index];
      std::size_t offset = info.offset();
      std::size_t length = info.dimension();

      info.set_mode(StateMode::Exclude);

      drop_rows(offset, length, &F);
      drop_rows(offset, length, &means_);
      // Shift all the remaining variables up by the number of
      for (std::size_t i = handle.index + 1; i < active_count; ++i) {
        variable_info_[i].change_offset_by(-length);
      }
      active_count--;
    }

    reshuffle_matrix(covs_, F, &covs_);

    // Shrink the active area
    auto new_active_size = get_active_var_dim();
    covs_.conservativeResize(new_active_size, new_active_size);

    return Ok();
  }

  template <s32 var_max_length>
  MaybeError change_mode(
      const StateMode newMode,
      const pvt_common::containers::StaticVector<StateHandle, var_max_length>
          &vars) {
    if (vars.size() > variable_info_.size()) {
      return Error("Too many variables requested");
    } else if (vars.size() == 0) {
      return Ok();
    } else if (newMode == StateMode::Exclude) {
      return Error("Invalid new mode, to exclude use the deactivate function");
    }

    // First validate all the input variables before changing anything
    for (const auto &handle : vars) {
      if (handle.index >= variable_info_.size()) {
        return Error("Invalid variable handle");
      }
      VariableInfo &info = variable_info_[handle.index];
      if (info.is_inactive() || info.mode() == newMode) {
        return Error("Invalid state mode transition requested");
      }
    }

    CovStorage F(covs_.rows(), covs_.cols());
    F.setIdentity();

    for (const auto &handle : vars) {
      VariableInfo &current_var = variable_info_[handle.index];
      int offset_delta = 0;
      std::size_t new_offset = 0;

      if (newMode == StateMode::Estimate) {
        offset_delta = current_var.dimension();
        new_offset = get_estimate_var_dim();
      } else {
        offset_delta = -current_var.dimension();
        new_offset = get_active_var_dim() - current_var.dimension();
      }

      std::size_t min_search_bounds = 0;
      std::size_t max_search_bounds = 0;
      if (newMode == StateMode::Estimate) {
        min_search_bounds = new_offset;
        max_search_bounds = current_var.offset();
      } else {
        min_search_bounds = current_var.offset() + 1;
        max_search_bounds = new_offset + current_var.dimension() + 1;
      }

      for (auto &var : variable_info_) {
        if (var.is_active() && (var.offset() >= min_search_bounds &&
                                var.offset() < max_search_bounds)) {
          var.change_offset_by(offset_delta);
        }
      }

      shift_rows_by(current_var.offset(), current_var.dimension(), new_offset,
                    &F);
      shift_rows_by(current_var.offset(), current_var.dimension(), new_offset,
                    &means_);

      current_var.set_mode(newMode);
      current_var.set_offset_to(new_offset);
    }

    reshuffle_matrix(covs_, F, &covs_);

    return Ok();
  }

 private:
  explicit StateManager(
      const pvt_common::containers::StaticVector<VariableInfo, MAX_DIM> &info)
      : variable_info_(info), means_(), covs_() {}

  std::size_t get_active_var_count() const {
    auto is_active = [](const auto &info) { return info.is_active(); };
    return std::count_if(variable_info_.begin(), variable_info_.end(),
                         is_active);
  }

  template <typename... Ts>
  static auto sum_modes(Ts &&... modes) {
    return [=](std::size_t sum, const VariableInfo &var) {
      auto select_modes = [&var](const auto &expected_mode, bool prev_result) {
        return prev_result || var.mode() == expected_mode;
      };
      if (pvt_common::functional::fold_right_to_left(select_modes, modes...,
                                                     false)) {
        sum += var.dimension();
      }
      return sum;
    };
  }

  std::size_t get_active_var_dim() const {
    return std::accumulate(variable_info_.begin(), variable_info_.end(), 0,
                           sum_modes(StateMode::Estimate, StateMode::Consider));
  }

  std::size_t get_estimate_var_dim() const {
    return std::accumulate(variable_info_.begin(), variable_info_.end(), 0,
                           sum_modes(StateMode::Estimate));
  }

  std::size_t get_inactive_var_dim() const {
    return std::accumulate(variable_info_.begin(), variable_info_.end(), 0,
                           sum_modes(StateMode::Exclude));
  }

  template <typename MatrixT>
  static void reshuffle_matrix(const CovStorage &cov_in, const CovStorage &F,
                               MatrixT *M_out) {
    CovStorage W(F.rows(), F.cols());
    MeanStorage d(F.cols());
    math::kalman::UD::fill_mwgs_matrices(W, d, cov_in, F);
    M_out->conservativeResize(F.rows(), F.rows());
    math::kalman::UD::modified_weighted_gram_schmidt(*M_out, W, d);
  }

  template <typename MatrixT>
  static void shift_rows_by(std::size_t current_offset, std::size_t length,
                            std::size_t new_offset, MatrixT *M) {
    if (current_offset == new_offset) {
      return;
    }

    std::size_t A_offset = 0;
    std::size_t A_length = 0;
    std::size_t B_offset = 0;
    std::size_t B_length = 0;

    if (current_offset < new_offset) {
      A_offset = current_offset;
      A_length = length;
      B_offset = current_offset + length;
      B_length = new_offset - current_offset;
    } else {
      A_offset = new_offset;
      A_length = current_offset - new_offset;
      B_offset = current_offset;
      B_length = length;
    }

    auto A = M->middleRows(A_offset, A_length).eval();
    auto B = M->middleRows(B_offset, B_length).eval();

    M->middleRows(A_offset, B_length) = B;
    M->middleRows(A_offset + B_length, A_length) = A;
  }

  template <typename MatrixT>
  static void drop_rows(std::size_t row_offset, std::size_t row_count,
                        MatrixT *M) {
    M->middleRows(row_offset, M->rows() - row_offset - row_count) =
        M->bottomRows(M->rows() - (row_offset + row_count));
    M->conservativeResize(M->rows() - row_count, M->cols());
  }

  pvt_common::containers::StaticVector<VariableInfo, MAX_DIM> variable_info_;
  MeanStorage means_;
  CovStorage covs_;
  // W, d, F
};

class StateManagerFactory {
 public:
  StateHandle add_state(std::size_t state_dim) {
    variable_info_.append({StateMode::Exclude, 0, state_dim});
    return StateHandle{static_cast<std::size_t>(variable_info_.size() - 1)};
  }

  ValueOrError<StateManager> create_manager() {
    auto var_size_adder = [](std::size_t sum,
                             const StateManager::VariableInfo &info) {
      return sum + info.dimension();
    };
    std::size_t total_random_size = std::accumulate(
        variable_info_.begin(), variable_info_.end(), 0, var_size_adder);

    if (total_random_size > MAX_DIM) {
      return Error("Maximum dimension exceeded");
    }

    return StateManager(variable_info_);
  }

 private:
  pvt_common::containers::StaticVector<StateManager::VariableInfo, MAX_DIM>
      variable_info_ = {};
};

}  // namespace sensorfusion

#endif  // STARLING_ESTIMATION_STATE_MANAGEMENT_H
