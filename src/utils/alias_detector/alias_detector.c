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

#include <swiftnav/constants.h>

#include <string.h>

#include "alias_detector.h"

/** Initialise alias lock detection structure.
 *
 * \param a         The alias lock detect struct.
 * \param acc_len   The number of points to accumulate before calculating error.
 * \param time_diff Time difference between calls to alias_detect_first and
 *                  alias_detect_second.
 */
void alias_detect_init(alias_detect_t *a, u32 acc_len, float time_diff) {
  memset(a, 0, sizeof(*a));
  a->acc_len = acc_len;
  a->k = (float)(0.5 * M_1_PI) / time_diff;
  a->first_I = 1.f;
}

/** Update alias lock detection structure.
 *
 * \param a         The alias lock detect struct.
 * \param acc_len   The number of points to accumulate before calculating error.
 * \param time_diff Time difference between calls to alias_detect_first and
 *                  alias_detect_second.
 */
void alias_detect_reinit(alias_detect_t *a, u32 acc_len, float time_diff) {
  /* Just reset averaging. To preserve state it would be necessary to rescale
   * the dot and cross terms based on the new and old time_diff values.
   */
  alias_detect_init(a, acc_len, time_diff);
}

/** Load first I/Q sample into alias lock detect structure.
 *
 * \param a The alias lock detect struct.
 * \param I The prompt in-phase correlation.
 * \param Q The prompt quadrature-phase correlation.
 */
void alias_detect_first(alias_detect_t *a, float I, float Q) {
  a->first_I = I;
  a->first_Q = Q;
}

/** Load second I/Q sample into alias lock detect structure and return
 *  frequency error if ready.
 *
 * \param a The alias lock detect struct.
 * \param I The prompt in-phase correlation.
 * \param Q The prompt quadrature-phase correlation.
 * \returns Calculated frequency error or zero.
 */
float alias_detect_second(alias_detect_t *a, float I, float Q) {
  a->dot += I * a->first_I + Q * a->first_Q;
  a->cross += a->first_I * Q - I * a->first_Q;
  a->first_I = I;
  a->first_Q = Q;

  a->fl_count++;
  if (a->fl_count == a->acc_len) {
    float err = atan2f(a->cross, a->dot) * a->k;
    a->fl_count = 0;
    a->cross = 0;
    a->dot = 0;
    return err;
  }
  return 0;
}
