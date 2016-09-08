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

#include <stdint.h>

/** @brief Attempts to read NT1065 temperature sensor
 *
 *  @param timeout_ms number of milliseconds to wait for read to complete
 *  @param temperature if succesful temperature is written here
 *  @return True if temperature is valid. False if timed out
 */
bool nt1065_get_temperature(double* temperature);

uint8_t nt1065_read_reg(uint8_t reg_addr);

void nt1065_write_reg(uint8_t reg_addr, uint8_t value);
