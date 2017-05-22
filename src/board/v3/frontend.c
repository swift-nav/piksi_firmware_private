/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *          Gareth McMullin <gareth@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ch.h>
#include <hal.h>
#include <libswiftnav/logging.h>

#include "frontend.h"
#include "nt1065.h"
#include "system_monitor.h"

#define SPI_READ_MASK (1 << 7)

static const SPIConfig spi_config = FRONTEND_SPI_CONFIG;

static void configure_v1(void);
static void configure_v2(void);

static void frontend_open_spi(void)
{
  spiAcquireBus(&FRONTEND_SPI);
  spiStart(&FRONTEND_SPI, &spi_config);
  spiSelect(&FRONTEND_SPI);
}

static void frontend_close_spi(void)
{
  spiUnselect(&FRONTEND_SPI);
  spiReleaseBus(&FRONTEND_SPI);
}

static u8 spi_write(u8 reg, u8 data)
{
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];

  spiExchange(&FRONTEND_SPI, sizeof(send_buf), send_buf, recv_buf);

  return recv_buf[1];
}

static u8 spi_read(u8 reg)
{
  const u8 dummy_data = 0x00;
  const u8 send_buf[2] = {reg | SPI_READ_MASK, dummy_data};
  u8 recv_buf[2];

  spiExchange(&FRONTEND_SPI, sizeof(send_buf), send_buf, recv_buf);

  return recv_buf[1];
}

static void frontend_isr(void *context)
{
  (void)context;
  frontend_error_notify_isr();
}

void frontend_configure(void)
{
  bool is_aok = true;
  /* If the NT1065 doesn't become healthy within a timeout, retry config */
  do {
    frontend_open_spi();

    /* Read chip ID and release */
    u16 id_release = ((u16)spi_read(0) << 8) | spi_read(1);
    u16 id = (id_release >> 3) & 0x1fff;
    u8 release = id_release & 0x7;

    if (id != 1065) {
      log_error("nt1065: invalid chip ID");
    }

    switch (release) {
    case 1:
      configure_v1();
      break;
    case 2:
      configure_v2();
      break;
    default:
      log_error("nt1065: unsupported chip release");
      break;
    }

    frontend_close_spi();

    /* Wait for frontend clock to stabilize, AOK status */
    u8 tries = 100;
    do {
      chThdSleepMilliseconds(1);
    } while (!nt1065_check_aok_status() && (--tries > 0));

    is_aok = tries != 0;
    if (!is_aok) {
      frontend_error_notify_sys();
      log_error("nt1065: config failed, retrying");
    }
  } while (!is_aok);

  /* Enable AOK interrupt */
  gic_handler_register(IRQ_ID_FRONTEND_AOK, frontend_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_FRONTEND_AOK, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_FRONTEND_AOK, FRONTEND_AOK_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_FRONTEND_AOK);

  /* Make sure AOK interrupt edge was not missed */
  if (!nt1065_check_aok_status()) {
    frontend_error_notify_sys();
  }
}

void frontend_setup(void)
{
  /* Register any setting... */
}

bool nt1065_get_temperature(double* temperature)
{
  int32_t temp_sensor = 0;
  //temperature is valid after about 30 milliseconds
  const uint32_t TEMP_READ_WAIT_MS = 30;

  //start a single temp measurement
  const u8 REG5 = 1;
  frontend_open_spi();
  spi_write(5, REG5);
  frontend_close_spi();

  chThdSleepMilliseconds(TEMP_READ_WAIT_MS);

  frontend_open_spi();
  //check if temperature read completed
  if ((spi_read(5) & 1) != 0) {
    frontend_close_spi();
    return false;
  }

  //lower 8 bits
  temp_sensor = spi_read(8);
  //upper 2 bits are addr=7 bits 1-0
  temp_sensor |= (spi_read(7) & 3) << 8;

  frontend_close_spi();

  *temperature = 27. - ((double)(temp_sensor - 551)) * 0.865;

  return true;
}

bool nt1065_check_plls()
{
  frontend_open_spi();
  u8 pll_a_status = spi_read(44) & 7;
  u8 pll_b_status = spi_read(48) & 7;
  frontend_close_spi();

  if (pll_a_status != 1 && pll_b_status != 1) {
    if ((pll_a_status & 1) == 0) {
      log_error("nt1065: PLL A not locked");
    }
    if (pll_a_status & 2) {
      log_error("nt1065: PLL A VCO voltage above max bound");
    }
    if (pll_a_status & 4) {
      log_error("nt1065: PLL A VCO voltage below min bound");
    }
    if ((pll_b_status & 1) == 0) {
      log_error("nt1065: PLL B not locked");
    }
    if (pll_b_status & 2) {
      log_error("nt1065: PLL B VCO voltage above max bound");
    }
    if (pll_b_status & 4) {
      log_error("nt1065: PLL B VCO voltage below min bound");
    }
    return false;
  }

  return true;
}

bool nt1065_check_standby()
{
  frontend_open_spi();
  u8 ic_mode = spi_read(2) & 3;
  frontend_close_spi();
  if (ic_mode != 3) {
    log_error("nt1065: Not in active mode");
    return false;
  }
  return true;
}

bool nt1065_check_calibration()
{
  frontend_open_spi();
  u8 calibration_status = spi_read(4) & 2;
  frontend_close_spi();
  if (calibration_status != 2) {
    log_error("nt1065: Auto-calibration error");
    return false;
  }
  return true;
}

bool nt1065_check_aok_status()
{
  u8 reg7 = nt1065_read_reg(7);
  return (reg7 & 0x10) != 0;
}

uint8_t nt1065_read_reg(uint8_t reg_addr)
{
  uint8_t value;
  frontend_open_spi();
  value = spi_read(reg_addr);
  frontend_close_spi();
  return value;
}

void nt1065_write_reg(uint8_t reg_addr, uint8_t value)
{
  frontend_open_spi();
  spi_write(reg_addr, value);
  frontend_close_spi();
}

static void configure_v1(void)
{
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

static void configure_v2(void)
{
  spi_write( 2, 0x03);
  spi_write( 3, 0x01); /* TCXO frequency setting and LO source*/
  spi_write( 4, 0x03);
  spi_write( 5, 0x00);
  spi_write( 6, 0x1D);
  spi_write(11, 0x0F); /* clock divider ratio (scaled by 2) e.g. 0x08 = /16 and 0x0F = /30 */
  spi_write(12, 0x1C); /* clock source and signal type */
  spi_write(13, 0x03); /* channel 1 enabled and Upper/Lower side-band */
  spi_write(14, 0x30); /* LPF setting for channel 1 - GPS L1 at about 22 MHz */
  spi_write(15, 0x0B);
  spi_write(16, 0x34);
  spi_write(17, 0xF1);
  spi_write(18, 0xEA);
  spi_write(19, 0x0B);
  spi_write(20, 0x01); /* channel 2 enabled and Upper/Lower side-band */
  spi_write(21, 0x26); /* LPF setting for channel 2 - Glonass G1 */
  spi_write(22, 0x0B);
  spi_write(23, 0x34);
  spi_write(24, 0xF1);
  spi_write(25, 0xEA);
  spi_write(26, 0x0B);
  spi_write(27, 0x01); /* channel 3 enabled and Upper/Lower side-band */
  spi_write(28, 0x1D); /* LPF setting for channel 3 - Glonass G2 */
  spi_write(29, 0x0B);
  spi_write(30, 0x34);
  spi_write(31, 0xF1);
  spi_write(32, 0xEA);
  spi_write(33, 0x0B);
  spi_write(34, 0x03); /* channel 4 enabled and Upper/Lower side-band */
  spi_write(35, 0x18); /* LPF setting for channel 4 - GPS L2 at about 16 MHz */
  spi_write(36, 0x0B);
  spi_write(37, 0x34);
  spi_write(38, 0xF1);
  spi_write(39, 0xEA);
  spi_write(40, 0x0B);
  spi_write(41, 0x03); /* PLL A band and enable */
  spi_write(42, 0x4F); /* PLL A N[8..1] divider */
  spi_write(43, 0x89); /* PLL A N[0] and R divider */
  spi_write(45, 0x01); /* PLL B band and enable */
  spi_write(46, 0x7B); /* PLL B N[8..1] divider */
  spi_write(47, 0x91); /* PLL B N[0] and R divider */
}
