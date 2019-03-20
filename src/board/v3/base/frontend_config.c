/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>

static void configure_v1(void) { assert("unsupported" == 0); }

/* this config has VCO2 at 1200 MHz so
 * - GPS L2 is still on CH4 but in low-injection mode at +27.6 MHz
 * - BDS2 B2 is also on CH4 but in low-injection mode at +7.14 MHz
 * - Glonass L2OF is GONE
 * - Galileo E5b is like Beidou B2
 * - GPS L5 and Galileo E5b are on CH3 in high-injection mode at -23.55 MHz
 * */
static void configure_v2(void) {
  spi_write(2, 0x03);
  spi_write(3, 0x01); /* TCXO frequency setting and LO source*/
  spi_write(4, 0x03);
  spi_write(5, 0x00);
  spi_write(6, 0x1D);
  spi_write(11, 0x08); /* clock divider ratio (/2) e.g.
                          0x08==/16 and
                          0x0F==/30 */
  spi_write(12, 0x1C); /* clock source and signal type */
  spi_write(13, 0x01); /* channel 1 enabled and low-injection */
  spi_write(14, 0x26); /* Glonass L1OF LPF */
  spi_write(15, 0x1B); /* RF and IF AGC mode*/
  spi_write(16, 0x53); /* Max and minimum RF AGC threshholds*/
  spi_write(17, 0xF1);
  spi_write(18, 0xEA);
  spi_write(19, 0x0B);
  spi_write(20, 0x03); /* channel 2 enabled and high-injection */
  spi_write(21, 0x52); /* GPS L1 and BDS2 B11 LPF at 32.4 MHz */
  spi_write(22, 0x1B); /* RF and IF AGC mode*/
  spi_write(23, 0x53); /* Max and minimum RF AGC threshholds*/
  spi_write(24, 0xF1);
  spi_write(25, 0xEA);
  spi_write(26, 0x0B);
  spi_write(27, 0x03); /* channel 3 enabled and high-injection */
  spi_write(28, 0x52); /* LPF at 32.4 MHz for GPS L5 on -23.55 MHz */
  spi_write(29, 0x0B);
  spi_write(30, 0x34);
  spi_write(31, 0xF1);
  spi_write(32, 0xEA);
  spi_write(33, 0x0B);
  spi_write(34, 0x01); /* channel 4 enabled and low-injection */
  spi_write(35, 0x52); /* LPF at 32.4 MHz for GPS L2 on +27.6 MHz */
  spi_write(36, 0x0B);
  spi_write(37, 0x34);
  spi_write(38, 0xF1);
  spi_write(39, 0xEA);
  spi_write(40, 0x0B);
  spi_write(41, 0x03); /* PLL A band and enable */
  spi_write(42, 0x4F); /* PLL A N[8..1] divider (79*2=158) */
  spi_write(43, 0x89); /* PLL A N[0] (+1=159) and R divider (1) */
  spi_write(45, 0x01); /* PLL B band and enable */
  spi_write(46, 0x3C); /* PLL B N[8..1] divider (60*2=120) */
  spi_write(47, 0x09); /* PLL B N[0] (+0=120) and R divider (1) */
}
