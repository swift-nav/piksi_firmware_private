/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tpaakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_TRACK_UTILS_H
#define SWIFTNAV_TRACK_UTILS_H

#include <libswiftnav/constants.h>
#include <libswiftnav/track.h>

/* FLL saturation threshold in [Hz]. When signal is lost, filtered frequency
 * error can grow fast.
 * When the signal comes back, the saturation threshold helps the filter to
 * converge quickly below error threshold.
*/
#define TP_FLL_SATURATION_THRESHOLD_HZ (15.f)
/* FLL error threshold in [Hz]. Used to assess FLL frequency lock.
 * The threshold should be less than the expected aliased frequency, < 25 Hz.
 * Another factor is to avoid false positives from high dynamics.
*/
#define TP_FLL_ERR_THRESHOLD_HZ (10.f)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void freq_lock_detect_init(
    lock_detect_t *l, float k1, float k2, u16 lp, u16 lo);
void freq_lock_detect_update(lock_detect_t *l, float err);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SWIFTNAV_TRACK_UTILS_H */
