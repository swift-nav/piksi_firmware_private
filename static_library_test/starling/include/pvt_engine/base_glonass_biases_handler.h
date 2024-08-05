/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_BASE_GLONASS_BIASES_HANDLER_H
#define LIBSWIFTNAV_BASE_GLONASS_BIASES_HANDLER_H

#include "optional.h"
#include "swiftnav/glonass_phase_biases.h"

namespace pvt_engine {

class GlonassBiases {
 public:
  GlonassBiases();

  void initialize();

  optional<glo_biases_t> get_glonass_biases() const;

  bool is_reset_needed(const glo_biases_t &biases) const;

  void set_known_glonass_biases(const glo_biases_t &biases);

 private:
  optional<glo_biases_t> known_base_glonass_biases_;
};
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_BASE_GLONASS_BIASES_HANDLER_H
