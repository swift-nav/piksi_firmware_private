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

 /*CHANGES*/

#include "frontend.h"

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <swiftnav/logging.h>

#include "nt1065.h"
#include "system_monitor/system_monitor.h"

#define SPI_READ_MASK (1 << 7)
#define NUM_CHANNELS 4
static const SPIConfig spi_config = FRONTEND_SPI_CONFIG;

static void configure_v1(void);
static void configure_v2(void);

static void frontend_open_spi(void) {
  spiAcquireBus(&FRONTEND_SPI);
  spiStart(&FRONTEND_SPI, &spi_config);
  spiSelect(&FRONTEND_SPI);
}

static void frontend_close_spi(void) {
  spiUnselect(&FRONTEND_SPI);
  spiReleaseBus(&FRONTEND_SPI);
}

static u8 spi_write(u8 reg, u8 data) {
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];

  spiExchange(&FRONTEND_SPI, sizeof(send_buf), send_buf, recv_buf);

  return recv_buf[1];
}

static u8 spi_read(u8 reg) {
  const u8 dummy_data = 0x00;
  const u8 send_buf[2] = {reg | SPI_READ_MASK, dummy_data};
  u8 recv_buf[2];

  spiExchange(&FRONTEND_SPI, sizeof(send_buf), send_buf, recv_buf);

  return recv_buf[1];
}

static void frontend_isr(void* context) {
  (void)context;
  frontend_error_notify_isr();
}

void frontend_configure(void) {
  bool is_aok = true;
  /* If the NT1065 doesn't become healthy within a timeout, retry config */
  do {

    /* Enable AOK interrupt */
    gic_handler_register(IRQ_ID_FRONTEND_AOK, frontend_isr, NULL);
    gic_irq_sensitivity_set(IRQ_ID_FRONTEND_AOK, IRQ_SENSITIVITY_EDGE);
    gic_irq_priority_set(IRQ_ID_FRONTEND_AOK, FRONTEND_AOK_IRQ_PRIORITY);
    gic_irq_enable(IRQ_ID_FRONTEND_AOK);
    log_info("NT1065 IRQ set")

    /* Make sure AOK interrupt edge was not missed */
    if (!nt1065_check_aok_status()) {
      frontend_error_notify_sys();
    }

    frontend_open_spi();

    /* Read chip ID and release */
    u16 id_release = ((u16)spi_read(0) << 8) | spi_read(1);
    u16 id = (id_release >> 3) & 0x1fff;
    u8 release = id_release & 0x7;

    log_info("NT1065 ID: %d", id);
    log_info("NT1065 release: %d", release);

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
}

void frontend_setup(void) { /* Register any setting... */
  log_info("so it only goes until here? what about other stuff?");
}

bool nt1065_get_temperature(double* temperature) {
  int32_t temp_sensor = 0;
  // temperature is valid after about 30 milliseconds
  const uint32_t TEMP_READ_WAIT_MS = 30;

  // start a single temp measurement
  const u8 REG5 = 1;
  frontend_open_spi();
  spi_write(5, REG5);
  frontend_close_spi();

  chThdSleepMilliseconds(TEMP_READ_WAIT_MS);

  frontend_open_spi();
  // check if temperature read completed
  if ((spi_read(5) & 1) != 0) {
    frontend_close_spi();
    return false;
  }

  // lower 8 bits
  temp_sensor = spi_read(8);
  // upper 2 bits are addr=7 bits 1-0
  temp_sensor |= (spi_read(7) & 3) << 8;
  frontend_close_spi();

  *temperature = 27. - ((double)(temp_sensor - 551)) * 0.865;
  return true;
}

/* NT1065 AGC gain constants (refer to datasheet) */

#define AGC_CHANNEL_INDEX_SHIFT (4)
#define AGC_STATUS_REG (5)
#define AGC_RF_GAIN_REG (9)
#define AGC_IND_MASK (0x30)
#define AGC_IND_SHIFT (4)
#define AGC_RF_GAIN_MASK (0xf)
#define AGC_IF_GAIN_REG (10)
#define AGC_IF_GAIN_MASK (0x1f)
#define AGC_IF_GAIN_SHIFT (0)
#define AGC_RF_GAIN_SF (0.95)
#define AGC_RF_GAIN_OFFSET (12.0)
#define AGC_IF_GAIN_MAX_DB (64.0)
#define AGC_IF_GAIN_RANGE (23.0f)
#define AGC_IF_GAIN_SF (AGC_IF_GAIN_MAX_DB / AGC_IF_GAIN_RANGE)
#define AGC_IF_GAIN_OFFSET (-0.5)
#define AGC_RF_GAIN_RANGE (15.0f)
/* Invalid gain percentage is -1 in SBP */
#define AGC_INVALID_GAIN (-1)

void nt1065_get_agc(s8 rf_gain_array[], s8 if_gain_array[]) {
  u8 agc_indicator = 0;
  u8 agc_rf_gain = 0;
  u8 agc_if_gain = 0;
  if (rf_gain_array != NULL && if_gain_array != NULL) {
    /* Read each AGC gain value for each channel from registers 9 and 10 */
    for (int i = 0; i < NUM_CHANNELS; i++) {
      frontend_open_spi();
      /* set NT1065 channel for gain status */
      spi_write(AGC_STATUS_REG, i << AGC_CHANNEL_INDEX_SHIFT);
      agc_rf_gain = spi_read(AGC_RF_GAIN_REG);
      /* bits 5 & 6 of reg 9 are rf agc gain indicator */
      /* Note: the AGC indicator suggests that we may have non-optimal RF
       * levels to 1065 in many cases where the receiver appears to function
       * normally. Rather than spam users, only the conditionally compiled
       * LOG_DEBUG is sent for agc_indicator at this time. */
      agc_indicator = (agc_rf_gain & AGC_IND_MASK) >> AGC_IND_SHIFT;
      /* lower nible of reg 9 is the rf AGC gain */
      agc_rf_gain &= AGC_RF_GAIN_MASK;
      /* bits 4 - 0 are the IFA gain */
      agc_if_gain = spi_read(AGC_IF_GAIN_REG) & AGC_IF_GAIN_MASK;
      frontend_close_spi();
      log_debug(
          "RF channel %d: agc_indicator->%x, rf_gain->%f dB, if_gain->%f dB",
          i,
          agc_indicator,
          agc_rf_gain * AGC_RF_GAIN_SF + AGC_RF_GAIN_OFFSET,
          agc_if_gain * AGC_IF_GAIN_SF + AGC_IF_GAIN_OFFSET);

      /* non-dimensionalize RF gain to percent (max is 0b1111 or 15) */
      rf_gain_array[i] = (s8)lrintf(agc_rf_gain / AGC_RF_GAIN_RANGE * 100.0f);

      if (agc_indicator != 0) /* value of 0 indicates rf signal is in range */
      {
        log_debug(
            "NT1065 reported RF gain of %f dB out of range for channel %d",
            agc_rf_gain * AGC_RF_GAIN_SF + AGC_RF_GAIN_OFFSET,
            i + 1);
      }
      /* non-dimensionalize IF gain to percent (max is 0b10111 or 23) */
      if (agc_if_gain <= AGC_IF_GAIN_RANGE) {
        if_gain_array[i] = (s8)lrintf(agc_if_gain / AGC_IF_GAIN_RANGE * 100.0f);
      } else { /* Should never get here */
        log_error("NT1065 reported IFA gain of %f out of range for channel %d",
                  agc_if_gain * AGC_IF_GAIN_SF + AGC_IF_GAIN_OFFSET,
                  i);
        if_gain_array[i] = AGC_INVALID_GAIN;
      }
    }
  }
}

bool nt1065_check_plls() {
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

bool nt1065_check_standby() {
  frontend_open_spi();
  u8 ic_mode = spi_read(2) & 3;
  frontend_close_spi();
  if (ic_mode != 3) {
    log_error("nt1065: Not in active mode");
    return false;
  }
  return true;
}

bool nt1065_check_calibration() {
  frontend_open_spi();
  u8 calibration_status = spi_read(4) & 2;
  frontend_close_spi();
  if (calibration_status != 2) {
    log_error("nt1065: Auto-calibration error");
    return false;
  }
  return true;
}

bool nt1065_check_aok_status() {
  u8 reg7 = nt1065_read_reg(7);
  return (reg7 & 0x10) != 0;
}

uint8_t nt1065_read_reg(uint8_t reg_addr) {
  uint8_t value;
  frontend_open_spi();
  value = spi_read(reg_addr);
  frontend_close_spi();
  return value;
}

void nt1065_write_reg(uint8_t reg_addr, uint8_t value) {
  frontend_open_spi();
  spi_write(reg_addr, value);
  frontend_close_spi();
}

#include "frontend_config.c"
