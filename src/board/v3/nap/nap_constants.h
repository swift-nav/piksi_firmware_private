/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAP_CONSTANTS_H
#define SWIFTNAV_NAP_CONSTANTS_H

/**
 * @brief   The sampling rate of the samples coming off the frontend
 */
#define NAP_FRONTEND_RAW_SAMPLE_RATE_Hz (99.375e6)

/**
 * @brief   The sample rate decimation used by tracking channels
 */
#define NAP_TRACK_DECIMATION_RATE (5)

/**
 * @brief   The tracking channel sample rate after decimation
 * @note    This is the effective sampling rate of the tracking results
 */
#define NAP_TRACK_SAMPLE_RATE_Hz \
  (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz / NAP_TRACK_DECIMATION_RATE)

/**
 * @brief   The sample rate used by the sample count for timing
 * @note    The sample count currently uses tracking samples
 */
#define NAP_FRONTEND_SAMPLE_RATE_Hz (NAP_TRACK_SAMPLE_RATE_Hz)

/**
 * @brief   This is 2 ms expressed as integer number of tracking samples
 * @note    needed for Glonass carrier phase stability
 */
#define FCN_NCO_RESET_COUNT ((u64)(NAP_TRACK_SAMPLE_RATE_Hz / 500))

#define NAP_PPS_TIMING_COUNT_OFFSET (-20)
#define NAP_EXT_TIMING_COUNT_OFFSET (8)

#define NAP_EPL_SPACING_SAMPLES (1)
#define NAP_VE_E_SPACING_SAMPLES (63)

#define NAP_KEY_LENGTH (16)
#define NAP_VERSION_OFFSET (0)
#define NAP_BUILD_TIME_OFFSET (1)
#define NAP_BUILD_DATE_OFFSET (2)
#define NAP_VERSION_STRING_OFFSET (12)
#define NAP_VERSION_STRING_LENGTH (36)
#define NAP_RANDOM_OFFSET (48)
#define NAP_DNA_OFFSET (52)
#define NAP_DNA_LENGTH (8)

#endif /* SWIFTNAV_NAP_CONSTANTS_H */
