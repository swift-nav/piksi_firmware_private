/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Measurement Engine team <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef ME_CALC_PVT_H
#define ME_CALC_PVT_H

#include <libswiftnav/common.h>

/* If the residual in a pseudorange excluded by RAIM is larger than this, then
 * drop the channel and the corresponding ephemeris */
#define RAIM_DROP_CHANNEL_THRESHOLD_M 1000

void me_calc_pvt_setup(void);

#endif /* ME_CALC_PVT_H */
