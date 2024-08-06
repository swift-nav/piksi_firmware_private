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

#ifndef LIBSWIFTNAV_AMBIGUITY_LAMBDA_INTERFACE_H
#define LIBSWIFTNAV_AMBIGUITY_LAMBDA_INTERFACE_H

namespace pvt_engine {

namespace ambiguities {

struct ReferenceIndex_t {
  gnss_signal_t sid;
  s32 index;

  bool operator<(const ReferenceIndex_t &other) const;
};

using ReferenceIndexMap =
    pvt_common::containers::Map<code_t, ReferenceIndex_t, CODE_COUNT>;

struct SDiffToDDiffMap_t {
  pvt_common::containers::Map<
      code_t, pvt_common::containers::StaticVector<s32, cMaxAmbiguities>,
      CODE_COUNT>
      index;
  s32 num_signals;

  s32 get_num_sdiffs(const code_t &code) const;

  SDiffToDDiffMap_t() : index(), num_signals(0) {}
};

bool select_code(const gnss_signal_t &sid, const code_t &code);

PRC get_single_to_double_difference_operator_single_code(
    const SDiffToDDiffMap_t &sd_map, const ReferenceIndexMap &references,
    const code_t &code, MatrixMaxAmbss32_t *T_single_to_double);

PRC get_single_to_double_difference_operator(
    const SDiffToDDiffMap_t &sd_map, const ReferenceIndexMap &references,
    MatrixMaxAmbss32_t *T_single_to_double);

}  // namespace ambiguities
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_AMBIGUITY_LAMBDA_INTERFACE_H