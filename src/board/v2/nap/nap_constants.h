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
#define NAP_FRONTEND_SAMPLE_RATE_Hz                                (16368000)

/**
 * @brief   The tracking channel sample rate after decimation
 * @note    This is the effective sampling rate of the tracking results
 */
#define TRACK_SAMPLE_FREQ NAP_FRONTEND_SAMPLE_RATE_Hz

#endif /* SWIFTNAV_NAP_CONSTANTS_H */
