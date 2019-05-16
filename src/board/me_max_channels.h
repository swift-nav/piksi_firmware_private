/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "starling/config.h"
#include "swiftnap.h"

#ifndef ME_MAX_CHANNELS_H
#define ME_MAX_CHANNELS_H

#if defined MAX_CHANNELS && MAX_CHANNELS < NAP_NUM_TRACKING_CHANNELS
#define ME_CHANNELS MAX_CHANNELS
#else
#define ME_CHANNELS NAP_NUM_TRACKING_CHANNELS
#endif

#endif /* ME_MAX_CHANNELS_H */
