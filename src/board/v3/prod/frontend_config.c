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
#include "init.h"

static void configure_v1(void) {
  assert(!hw_is_l5());
  for (u8 i = 0; i < 2; ++i) {
    spi_write(2, 0x03);
    spi_write(3, 0x01);
    spi_write(4, 0x03);
    spi_write(5, 0x00);
    /* Ignore AGC errors for AOK indicator */
    spi_write(6, 0b11101);
    spi_write(9, 0x00);
    spi_write(11, 0x08);
    spi_write(12, 0x1C);
    spi_write(13, 0x03);
    spi_write(14, 0x1E);
    spi_write(15, 0x2B);
    spi_write(16, 0x74);
    spi_write(17, 0xF1);
    spi_write(18, 0xEA);
    spi_write(19, 0x0B);
    spi_write(20, 0x01);
    spi_write(21, 0x28);
    spi_write(22, 0x2B);
    spi_write(23, 0x74);
    spi_write(24, 0xF1);
    spi_write(25, 0xEA);
    spi_write(26, 0x0B);
    spi_write(27, 0x01);
    spi_write(28, 0x28);
    spi_write(29, 0x2B);
    spi_write(30, 0x74);
    spi_write(31, 0xF1);
    spi_write(32, 0xEA);
    spi_write(33, 0x0B);
    spi_write(34, 0x03);
    spi_write(35, 0x7F);
    spi_write(36, 0x2B);
    spi_write(37, 0x74);
    spi_write(38, 0xF1);
    spi_write(39, 0xEA);
    spi_write(40, 0x0B);
    spi_write(41, 0x03);
    spi_write(42, 0x4F);
    spi_write(43, 0x89);
    spi_write(45, 0x01);
    spi_write(46, 0x7B);
    spi_write(47, 0x91);
    spi_write(49, 0xBD);
    spi_write(50, 0x0C);
    spi_write(51, 0x64);
    spi_write(52, 0xA8);
    spi_write(53, 0x5B);
    spi_write(54, 0xE5);
    spi_write(55, 0x57);
    spi_write(56, 0xE4);
    spi_write(57, 0xDD);
    spi_write(58, 0x0C);
    spi_write(59, 0x64);
    spi_write(60, 0xA8);
    spi_write(61, 0xC5);
    spi_write(62, 0x6D);
    spi_write(63, 0x57);
    spi_write(64, 0x71);
    spi_write(65, 0x7F);
    spi_write(66, 0x00);
    spi_write(67, 0xC9);
    spi_write(68, 0x86);
    spi_write(69, 0x7E);
    spi_write(70, 0x0F);
    spi_write(71, 0x11);
    spi_write(72, 0x00);
    spi_write(73, 0x00);
    spi_write(74, 0xFF);
    spi_write(75, 0x1C);
    spi_write(76, 0xCD);
    spi_write(77, 0x99);
    spi_write(78, 0x0B);
    spi_write(79, 0x40);
    spi_write(80, 0x08);
    spi_write(81, 0x30);
    spi_write(82, 0xFF);
    spi_write(83, 0x1C);
    spi_write(84, 0xCD);
    spi_write(85, 0x99);
    spi_write(86, 0x0B);
    spi_write(87, 0x40);
    spi_write(88, 0x08);
    spi_write(89, 0x30);
    spi_write(90, 0xFF);
    spi_write(91, 0x0C);
    spi_write(92, 0xCD);
    spi_write(93, 0x99);
    spi_write(94, 0x0B);
    spi_write(95, 0x40);
    spi_write(96, 0x08);
    spi_write(97, 0x30);
    spi_write(98, 0xFF);
    spi_write(99, 0x0C);
    spi_write(100, 0xCD);
    spi_write(101, 0x99);
    spi_write(102, 0x0B);
    spi_write(103, 0x40);
    spi_write(104, 0x08);
    spi_write(105, 0x30);
  }

  spi_write(15, 0x0B);
  spi_write(22, 0x0B);
  spi_write(29, 0x0B);
  spi_write(36, 0x0B);
}

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
  spi_write(13, 0x01); /* channel 1 enabled and Upper/Lower side-band */
  if (hw_is_l5()) {
    spi_write(14, 0x1D); /* Glonass L2OF LPF */
  } else {
    spi_write(14, 0x26); /* Glonass L1OF LPF */
  }
  spi_write(15, 0x1B); /* RF and IF AGC mode*/
  spi_write(16, 0x53); /* Max and minimum RF AGC threshholds*/
  spi_write(17, 0xF1);
  spi_write(18, 0xEA);
  spi_write(19, 0x0B);
  spi_write(20, 0x03); /* channel 2 enabled and Upper/Lower side-band */
  spi_write(21, 0x52); /* GPS L1 and BDS2 B11 LPF at 32.4 MHz */
                       /* Base has L2 and B2 here, but same config */
  spi_write(22, 0x1B); /* RF and IF AGC mode*/
  spi_write(23, 0x53); /* Max and minimum RF AGC threshholds*/
  spi_write(24, 0xF1);
  spi_write(25, 0xEA);
  spi_write(26, 0x0B);
  spi_write(27, 0x01); /* channel 3 enabled and Upper/Lower side-band */
  if (hw_is_l5()) {
    spi_write(28, 0x26); /* Glonass L1OF LPF */
  } else {
    spi_write(28, 0x1D); /* Glonass L2OF LPF */
  }
  spi_write(29, 0x1B); /* RF and IF AGC mode*/
  spi_write(30, 0x53); /* Max and minimum RF AGC threshholds*/
  spi_write(31, 0xF1);
  spi_write(32, 0xEA);
  spi_write(33, 0x0B);
  spi_write(34, 0x03); /* channel 4 enabled and Upper/Lower side-band */
  spi_write(35, 0x52); /* GPS L2 and BDS2 B2 LPF at 32.4 MHz */
                       /* Base has L1 and B1 here, but same config */
  spi_write(36, 0x1B); /* RF and IF AGC mode*/
  spi_write(37, 0x53); /* Max and minimum RF AGC threshholds*/
  spi_write(38, 0xF1);
  spi_write(39, 0xEA);
  spi_write(40, 0x0B);
  if (hw_is_l5()) {
    spi_write(41, 0x01); /* PLL A band and enable */
    spi_write(42, 0x7B); /* PLL A N[8..1] divider */
    spi_write(43, 0x91); /* PLL A N[0] and R divider */
    spi_write(45, 0x03); /* PLL B band and enable */
    spi_write(46, 0x4F); /* PLL B N[8..1] divider */
    spi_write(47, 0x89); /* PLL B N[0] and R divider */
  } else {
    spi_write(41, 0x03); /* PLL A band and enable */
    spi_write(42, 0x4F); /* PLL A N[8..1] divider */
    spi_write(43, 0x89); /* PLL A N[0] and R divider */
    spi_write(45, 0x01); /* PLL B band and enable */
    spi_write(46, 0x7B); /* PLL B N[8..1] divider */
    spi_write(47, 0x91); /* PLL B N[0] and R divider */
  }
}
