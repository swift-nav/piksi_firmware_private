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

#if defined(BOARD_PIKSIV3_EVT2)

static const SPIConfig spi_config = CLK_DAC_SPI_CONFIG;

void set_clk_dac(uint8_t val, uint8_t mode)
{
  assert(mode <= 3);
  uint16_t out = ((val & 0xF) << 12) | (val >> 4);
  out |= mode << 4;
  uint16_t in;
  spiStart(&CLK_DAC_SPI, &spi_config);
  spiAcquireBus(&CLK_DAC_SPI);

  spiSelect(&CLK_DAC_SPI);
  spiExchange(&CLK_DAC_SPI, 2, &out, &in);
  spiUnselect(&CLK_DAC_SPI);

  spiReleaseBus(&CLK_DAC_SPI);
}

#endif
