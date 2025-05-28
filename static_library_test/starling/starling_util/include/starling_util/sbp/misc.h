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

#ifndef STARLING_UTIL_SBP_MISC_H
#define STARLING_UTIL_SBP_MISC_H

#include <libsbp/v4/gnss/GPSTime.h>
#include <libsbp/v4/gnss/GnssSignal.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

namespace starling {
namespace util {
namespace sbp {

constexpr size_t cMsgObsHeaderSeqShift = 4u;
constexpr size_t cMsgObsHeaderSeqMask = ((1u << 4u) - 1u);
constexpr size_t cMsgObsHeaderMaxSize = cMsgObsHeaderSeqMask;

constexpr double cMsgHeadingScaleFactor = 1000.0;
constexpr double cMsgObsPMultiplier = 5e1;
constexpr double cMsgObsCn0Multiplier = 4.0;
constexpr uint16_t cMsgObsLfOverflow = 1u << 8u;
constexpr uint16_t cMsgObsDfOverflow = 1u << 8u;
constexpr double cMsgObsLfMultiplier = 256.0;
constexpr double cMsgObsDfMultiplier = 256.0;
constexpr uint8_t cMsgObsFlagsCodeValid = 1u << 0u;
constexpr uint8_t cMsgObsFlagsPhaseValid = 1u << 1u;
constexpr uint8_t cMsgObsFlagsHalfCycleKnown = 1u << 2u;
constexpr uint8_t cMsgObsFlagsMeasDopplerValid = 1u << 3u;
constexpr uint8_t cMsgObsFlagsRaimExclusion = 1u << 7u;

enum class MeasurementQuality {
  kPseudorangeValid = 0,
  kCarrierPhaseValid,
  kHalfCycleCarrierPhaseKnown,
  kValidMeasuredDoppler,
};

constexpr double cMsgOsrIonoStdMultiplier = 2e2;
constexpr double cMsgOsrTropoStdMultiplier = 2e2;
constexpr double cMsgOsrRangeStdMultiplier = 2e2;

/* Correction fixing consistency flags for MSG_OSR */
constexpr uint8_t cMsgOsrFlagsCorrectionValid = 1u << 0u;
constexpr uint8_t cMsgOsrFlagsPartialFixing = 1u << 1u;
constexpr uint8_t cMsgOsrFlagsFullFixing = 1u << 2u;
constexpr uint8_t cMsgOsrFlagssInvalidCodeCorrections = 1u << 3u;
constexpr uint8_t cMsgOsrFlagsInvalidPhaseCorrections = 1u << 4u;

enum class CorrectionsQuality {
  kCorrectionsValid = 0,
  kPartialFix,
  kFullFix,
};

uint8_t get_time_quality_flags(uint8_t time_qual);
/**
 * Helper function for rounding tow to integer milliseconds, taking care of
 * week roll-over
 * @param[in] tow Time-of-week in seconds
 * @return Time-of-week in milliseconds
 */
uint32_t round_tow_ms(double tow);
/**
 * Helper function for converting GPS time to integer milliseconds with
 * nanosecond remainder, taking care of week roll-over
 * @param[in] t_in GPS time
 * @param[out] t_out SBP time
 */
void round_time_nano(const gps_time_t &t_in, sbp_v4_gps_time_t *t_out);

sbp_v4_gnss_signal_t sid_to_sbp(const gnss_signal_t from);

gnss_signal_t sid_from_sbp(const sbp_v4_gnss_signal_t from);

}  // namespace sbp
}  // namespace util
}  // namespace starling

#endif  // STARLING_UTIL_SBP_MISC_H
