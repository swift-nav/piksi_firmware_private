/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdlib.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "board/v3/nt1065.h"
#include "board/v3/xadc.h"
#include "hal.h"
#include "sbp.h"
#include "zynq7000.h"

#define LED_nRST_GPIO_LINE PAL_LINE(GPIO2, 18)
#define MODEM_PWR_EN_LINE PAL_LINE(GPIO2, 28)
#define IMU_EN_GPIO_LINE PAL_LINE(GPIO2, 24)

/* NOTE: On Duro, the LED reset is positive logic */
#define DURO_LED_RST_GPIO_LINE PAL_LINE(GPIO2, 7)

/* Gain for channels not present is char MAX (127 or 0x7f) */
#define CHAN_NOT_PRESENT_GAIN 0x7f

const PALConfig pal_default_config;
const WDGConfig board_wdg_config = {
    .period_ms = 30000,
};

static void cycle_counter_init(void) {
  /* Set up TTC0_2 with period of ZYNQ7000_CPU_1x_FREQUENCY_Hz / 2^10 */
  TTC0->CLKCTRL[2] = (TTC_CLKCTRL_SRC_PCLK << TTC_CLKCTRL_SRC_Pos) |
                     (9 << TTC_CLKCTRL_PSVAL_Pos) | (1 << TTC_CLKCTRL_PSEN_Pos);
  TTC0->INTERVAL[2] = 0xffff;
  TTC0->CNTCTRL[2] =
      (1 << TTC_CNTCTRL_RESET_Pos) | (1 << TTC_CNTCTRL_INTERVAL_Pos);
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
  /* Enable PCAP CLK */
  *(volatile uint32_t *)0xF8000168 |= (1 << 0);

  palSetLineMode(SPI_SS_IMU_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(SPI_SS_IMU_GPIO_LINE);

  palSetLineMode(SPI_SS_FRONTEND_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(SPI_SS_FRONTEND_GPIO_LINE);

  palSetLineMode(SPI_SS_CLK_DAC_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(SPI_SS_CLK_DAC_GPIO_LINE);

  palSetLineMode(LED_nRST_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(LED_nRST_GPIO_LINE);

  palSetLineMode(DURO_LED_RST_GPIO_LINE, PAL_MODE_OUTPUT);
  palClearLine(DURO_LED_RST_GPIO_LINE);

  palSetLineMode(MODEM_PWR_EN_LINE, PAL_MODE_OUTPUT);
  palSetLine(MODEM_PWR_EN_LINE);

  palSetLineMode(IMU_EN_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(IMU_EN_GPIO_LINE);

  cycle_counter_init();
}

void board_send_state(void) {
  msg_device_monitor_t dev_mon_msg;
  msg_front_end_gain_t gain_msg;
  memset(gain_msg.rf_gain, CHAN_NOT_PRESENT_GAIN, sizeof(gain_msg.rf_gain));
  memset(gain_msg.if_gain, CHAN_NOT_PRESENT_GAIN, sizeof(gain_msg.rf_gain));

  double fe_temp = 0;
  if (!nt1065_get_temperature(&fe_temp)) {
    fe_temp = -99.99;
  }
  dev_mon_msg.fe_temperature = (s16)(fe_temp * 100);

  dev_mon_msg.dev_vin = (s16)(xadc_vin_get() * 1000);
  dev_mon_msg.cpu_vint = (s16)(xadc_vccint_get() * 1000);
  dev_mon_msg.cpu_vaux = (s16)(xadc_vccaux_get() * 1000);
  dev_mon_msg.cpu_temperature = (s16)(xadc_die_temp_get() * 100);

  sbp_send_msg(SBP_MSG_DEVICE_MONITOR, sizeof(dev_mon_msg), (u8 *)&dev_mon_msg);
  nt1065_get_agc(gain_msg.rf_gain, gain_msg.if_gain);
  sbp_send_msg(SBP_MSG_FRONT_END_GAIN, sizeof(gain_msg), (u8 *)&gain_msg);
}

/* ENET0_RST_LINE (MIO pin 0) is a GPIO connected to the hardware WDT /en line.
 * This line is pulled up in the bootloader to disable the WDT. Pulling this
 * line low enables the WDT triggering a hard reset after WDT timeout
 */
void hard_reset(void) {
  const ioline_t ENET0_RST_LINE = PAL_LINE(GPIO0, 0);
  const int WDT_RESET_WAIT = 10;
  palClearLine(ENET0_RST_LINE);
  chThdSleepSeconds(WDT_RESET_WAIT);
}
