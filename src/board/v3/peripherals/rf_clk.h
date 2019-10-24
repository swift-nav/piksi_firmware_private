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

#ifndef SWIFTNAV_RF_CLK_H
#define SWIFTNAV_RF_CLK_H

#include <stdbool.h>
#include <swiftnav/common.h>

extern bool is_clock_steering;

void rf_clk_init(bool allow_ext_clk);

void clock_steer(s32 clk_drift_ppb);

#endif /* SWIFTNAV_RF_CLK_H */
