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

#ifndef LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_UTILS_H
#define LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_UTILS_H

#include <pvt_common/containers/map.h>
#include <pvt_engine/eigen_types.h>
#include <pvt_engine/observation.h>
#include <swiftnav/nav_meas.h>

#define MAX_INPUT_OBSERVATION_COUNT 250

namespace pvt_engine {

namespace obs_utils {

bool is_observation_plausible(const double observation);

bool is_flag_set(const nav_meas_flags_t &obs_flags,
                 const nav_meas_flags_t &flag);

u64 mask_secondary_codes(u64 found_codes);
u64 mark_found_code(const u64 found_codes, const code_t code);
bool is_code_in_mask(const u64 masked_codes, const code_t code);
code_t to_supported_code_t(const code_t code);

bool usable_sid(const gnss_signal_t &sid);

bool only_gps_l2cm_sid(const navigation_measurement_t &a);
bool not_gps_l2cm_sid(const navigation_measurement_t &a);
bool has_mixed_l2_obs(u8 n, navigation_measurement_t *nav_meas);
void collapse_navmeas(u8 n, navigation_measurement_t *nav_meas);
s8 filter_navmeas(
    u8 *n, navigation_measurement_t nav_meas[],
    pvt_common::containers::Map<gnss_signal_t, measurement_std_t,
                                MAX_INPUT_OBSERVATION_COUNT> *observations_std,
    bool prefer_l2c);

/* Comparison function that orders the code types so that the highest priority
 * codes come first - used to sort observation arrays so that the primary
 * frequency comes first*/
bool single_freq_sid_comparison(const gnss_signal_t &sid_a,
                                const gnss_signal_t &sid_b);

/* Comparison function that orders the code types so that the highest priority
 * codes come first - used to sort observation arrays so that desired dual
 * frequency signals come first */
bool dual_freq_sid_comparison(const gnss_signal_t &sid_a,
                              const gnss_signal_t &sid_b);

typedef bool (*navigation_measurement_predicate_f)(
    const navigation_measurement_t &);

typedef bool (*navigation_measurement_predicate_extra_f)(
    const navigation_measurement_t &, void *);

/* Given an array of measurements `nav_meas` containing `n` elements,
 * remove all elements for which the selection function `predicate`
 * returns `false`.  The remaining elements (for which `predicate`
 * returned `true`) will be in their original order.  Returns the
 * number of elements in the array after filtering.
 */
u8 filter_nav_meas(u8 n, navigation_measurement_t nav_meas[],
                   navigation_measurement_predicate_f predicate);

/* Given an array of measurements `nav_meas` containing `n` elements,
 * remove all elements for which the selection function `predicate`
 * returns `false`.  The remaining elements (for which `predicate`
 * returned `true`) will be in their original order. The given pointer
 * `extra_data` will be passed as the second argument to the selection
 * function on each call.  Returns the number of elements in the array
 * after filtering.
 */
u8 filter_nav_meas_extra(u8 n, navigation_measurement_t nav_meas[],
                         navigation_measurement_predicate_extra_f predicate,
                         void *extra_data);

}  // namespace obs_utils

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_OBSERVATION_UTILS_H
