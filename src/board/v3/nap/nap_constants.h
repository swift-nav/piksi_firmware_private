/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
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
#define NAP_FRONTEND_RAW_SAMPLE_RATE_Hz                            (99.375e6)

/**
 * @brief   The sample rate decimation used by acquisition
 */
#define NAP_ACQ_DECIMATION_RATE                                          (12)

/**
 * @brief   The acquisition sample rate after decimation
 * @note    This is the effective sampling rate of the acquisition results
 */
#define NAP_ACQ_SAMPLE_RATE_Hz             (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz \
                                                   / NAP_ACQ_DECIMATION_RATE)

/**
 * @brief   The sample rate decimation used by tracking channels
 */
#define NAP_TRACK_DECIMATION_RATE                                         (4)

/**
 * @brief   The tracking channel sample rate after decimation
 * @note    This is the effective sampling rate of the tracking results
 */
#define NAP_TRACK_SAMPLE_RATE_Hz           (NAP_FRONTEND_RAW_SAMPLE_RATE_Hz \
                                                 / NAP_TRACK_DECIMATION_RATE)

/**
 * @brief   The sample rate used by the sample count for timing
 * @note    The sample count currently uses tracking samples
 */
#define NAP_FRONTEND_SAMPLE_RATE_Hz                (NAP_TRACK_SAMPLE_RATE_Hz)

/**
 * @brief   The phase increment used to mix the frontend samples to baseband
 * @note    4294967296 is 2^32 and the .5 is for rounding
 */
#define IF_2_MIXER_PINC(freq)                (s32)((freq) * ((u64)1 << 32)  \
                                     / NAP_FRONTEND_RAW_SAMPLE_RATE_Hz + 0.5)


#define NAP_FE_GPS_L1CA_BASEBAND_MIXER_PINC          IF_2_MIXER_PINC(14.58e6)
#define NAP_FE_GLO_L1CA_BASEBAND_MIXER_PINC         IF_2_MIXER_PINC(-12.00e6)
#define NAP_FE_L2C_BASEBAND_MIXER_PINC                 IF_2_MIXER_PINC(7.4e6)

#define NAP_SPACING_CHIPS                                                 (0)
#define NAP_SPACING_SAMPLES                                               (1)

#define NAP_VE_E_SPACING_CHIPS                                            (7)
#define NAP_VE_E_SPACING_SAMPLES                                         (23)

#define NAP_KEY_LENGTH                                                   (16)

#define NAP_VERSION_STRING_OFFSET                                         (8)
#define NAP_VERSION_STRING_LENGTH                                        (44)

#define NAP_DNA_OFFSET                                                   (52)
#define NAP_DNA_LENGTH                                                    (8)

#define NAP_PPS_TIMING_COUNT_OFFSET                                     (-20)
#define NAP_EXT_TIMING_COUNT_OFFSET                                       (8)

#endif /* SWIFTNAV_NAP_CONSTANTS_H */
