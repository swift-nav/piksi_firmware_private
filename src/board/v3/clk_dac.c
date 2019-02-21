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

#include "clk_dac.h"

#include <assert.h>
#include <hal.h>

static const SPIConfig spi_config = CLK_DAC_SPI_CONFIG;

/* Controls for 8 bit DAC DAC081S101 or 12 bit DAC121S101*/
void set_clk_dac(uint16_t val, uint8_t mode) {
  assert(mode <= 3);
  assert(val < 4096);
  uint8_t out[2];
  out[0] = (mode << 4) | (val >> 8);
  out[1] = val & 0xFF;
  uint8_t in[2];
  spiAcquireBus(&CLK_DAC_SPI);
  spiStart(&CLK_DAC_SPI, &spi_config);

  spiSelect(&CLK_DAC_SPI);
  spiExchange(&CLK_DAC_SPI, 2, out, in);
  spiUnselect(&CLK_DAC_SPI);

  spiReleaseBus(&CLK_DAC_SPI);
}
