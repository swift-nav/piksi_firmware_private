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

#ifndef LIBSWIFTNAV_GLONASS_BIASES_APPLICATION_H
#define LIBSWIFTNAV_GLONASS_BIASES_APPLICATION_H
#include <pvt_engine/ambiguity_map.h>
#include <pvt_engine/ambiguity_types.h>
#include <pvt_engine/optional.h>
#include <pvt_engine/pvt_return_codes.h>

namespace pvt_engine {

namespace ambiguities {

PRC select_ambiguities(
    const optional<glo_biases_t> &biases,
    const ambiguities::CodeSet &codes_to_fix,
    const ambiguities::AmbiguitiesAndCovariances &float_ambiguities,
    ambiguities::AmbiguitiesAndCovariances *selected_float_ambiguities);

bool is_fix_possible_on_sid(const gnss_signal_t &sid,
                            const optional<glo_biases_t> &biases,
                            const ambiguities::CodeSet &codes_to_fix);

bool glonass_biases_check(const optional<glo_biases_t> &biases);

PRC apply_glonass_biases(
    const optional<glo_biases_t> &biases,
    ambiguities::FloatAmbiguityStates *processed_float_ambs);
}  // namespace ambiguities
}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_GLONASS_BIASES_APPLICATION_H
