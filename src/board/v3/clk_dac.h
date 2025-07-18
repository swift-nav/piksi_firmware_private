/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Jonathan Diamond <jonathan@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef _CLK_DAC_H_
#define _CLK_DAC_H_

#include <stdint.h>

/* Normal */
#define CLK_DAC_MODE_0 0
/* 1k to GND */
#define CLK_DAC_MODE_1 1
/* 100k to GND */
#define CLK_DAC_MODE_2 2
/* High Impedance */
#define CLK_DAC_MODE_3 3

/** @brief set the clk dac voltage
 *    Sets the voltage and mode
 *
 *  @param val valid range 0 to (2^12-1) linearly sets voltage 0 to Vcc
 *  @param mode dac mode. Only mode 0 is affected by val
 */
void set_clk_dac(uint16_t val, uint8_t mode);

#endif
