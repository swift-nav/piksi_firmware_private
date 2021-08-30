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

#ifndef LEGACY_STARLING_UTIL_SBP_MISC_H
#define LEGACY_STARLING_UTIL_SBP_MISC_H

#include <libsbp/observation.h>
#include <libsbp/system.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSG_HEADING_SCALE_FACTOR 1000.0
#define MSG_OBS_P_MULTIPLIER (5e1)
#define MSG_OBS_CN0_MULTIPLIER (4.0)
#define MSG_OBS_LF_OVERFLOW (1 << 8)
#define MSG_OBS_DF_OVERFLOW (1 << 8)
#define MSG_OBS_LF_MULTIPLIER (256.0)
#define MSG_OBS_DF_MULTIPLIER (256.0)
#define MSG_OBS_FLAGS_CODE_VALID ((u8)(1 << 0))
#define MSG_OBS_FLAGS_PHASE_VALID ((u8)(1 << 1))
#define MSG_OBS_FLAGS_HALF_CYCLE_KNOWN ((u8)(1 << 2))
#define MSG_OBS_FLAGS_MEAS_DOPPLER_VALID ((u8)(1 << 3))
#define MSG_OBS_FLAGS_RAIM_EXCLUSION ((u8)(1 << 7))

typedef enum MeasurementQualityEnum {
  kPseudorangeValid = 0,
  kCarrierPhaseValid,
  kHalfCycleCarrierPhaseKnown,
  kValidMeasuredDoppler,
} MeasurementQualityEnum;

#define MSG_OSR_IONO_STD_MULTIPLIER (2e2)
#define MSG_OSR_TROPO_STD_MULTIPLIER (2e2)
#define MSG_OSR_RANGE_STD_MULTIPLIER (2e2)

/* Correction fixing consistency flags for MSG_OSR */
#define MSG_OSR_FLAGS_CORRECTION_VALID ((u8)(1 << 0))
#define MSG_OSR_FLAGS_PARTIAL_FIXING ((u8)(1 << 1))
#define MSG_OSR_FLAGS_FULL_FIXING ((u8)(1 << 2))
#define MSG_OSR_FLAGS_INVALID_CODE_CORRECTIONS ((u8)(1 << 3))
#define MSG_OSR_FLAGS_INVALID_PHASE_CORRECTIONS ((u8)(1 << 4))

typedef enum CorrectionsQualityEnum {
  kCorrectionsValid = 0,
  kPartialFix,
  kFullFix,
} CorrectionsQualityEnum;

u8 sbp_get_time_quality_flags(u8 time_qual);
/**
 * Helper function for rounding tow to integer milliseconds, taking care of
 * week roll-over
 * @param[in] tow Time-of-week in seconds
 * @return Time-of-week in milliseconds
 */
u32 round_tow_ms(double tow);
/**
 * Helper function for converting GPS time to integer milliseconds with
 * nanosecond remainder, taking care of week roll-over
 * @param[in] t_in GPS time
 * @param[out] t_out SBP time
 */
void round_time_nano(const gps_time_t *t_in, sbp_gps_time_t *t_out);

sbp_gnss_signal_t sid_to_sbp(gnss_signal_t from);

gnss_signal_t sid_from_sbp(sbp_gnss_signal_t from);

#ifdef __cplusplus
}
#endif

#endif  // LEGACY_STARLING_UTIL_SBP_MISC_H
