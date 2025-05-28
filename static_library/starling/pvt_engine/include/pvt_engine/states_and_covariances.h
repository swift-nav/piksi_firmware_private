/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_STATES_AND_COVARIANCES_H
#define LIBSWIFTNAV_STATES_AND_COVARIANCES_H

#include <pvt_engine/eigen_types.h>

namespace pvt_engine {

// Class to manipulate states and covariances generically
class StatesAndCovariances {
 public:
  StatesAndCovariances();

  StatesAndCovariances(const VectorMaxStateDimd_t &states,
                       const MatrixMaxStateDimd_t &covariances);

  StatesAndCovariances(const StatesAndCovariances &other) = default;

  StatesAndCovariances &operator=(const StatesAndCovariances &rhs) = default;

  void transform(const MatrixMaxStateDimd_t &transformation_matrix);

  const VectorMaxStateDimd_t &get_states() const;

  const MatrixMaxStateDimd_t &get_covariances() const;

 private:
  VectorMaxStateDimd_t states_;
  MatrixMaxStateDimd_t covariances_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_STATES_AND_COVARIANCES_H
