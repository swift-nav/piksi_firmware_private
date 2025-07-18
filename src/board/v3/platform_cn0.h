/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_PLATFORM_CN0_H
#define SWIFTNAV_PLATFORM_CN0_H

/** Platform noise figure (estimated) for C/N0 to SNR conversion. */
#define PLATFORM_NOISE_FIGURE (2.0f)
/** Normalized platform noise bandwidth for C/N0 estimator. */
#define PLATFORM_CN0_EST_BW_HZ (1.5f)
/** Scale coefficient for C/No estimator */
#define PLATFORM_CN0_EST_SCALE (1.025f)
/** C/No shift for Basic C/No estimation */
#define PLATFORM_CN0_EST_SHIFT (3.f)

#endif /* SWIFTNAV_PLATFORM_CN0_H */
