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

#include "hal.h"
#include "zynq7000.h"

#define LED_nRST_GPIO_LINE PAL_LINE(GPIO2, 18)
#define MODEM_PWR_EN_LINE PAL_LINE(GPIO2, 28)
#define IMU_EN_GPIO_LINE PAL_LINE(GPIO2, 24)

/* NOTE: On Duro, the LED reset is positive logic */
#define DURO_LED_RST_GPIO_LINE PAL_LINE(GPIO2, 7)

void boardRevInit(void) {
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
}

void boardRevUpdateModem(bool modem_enabled) {
  if (modem_enabled) {
    palSetLine(MODEM_PWR_EN_LINE);
  } else {
    palClearLine(MODEM_PWR_EN_LINE);
  }
}
