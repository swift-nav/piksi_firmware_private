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

#define CLK_SEL_GPIO_LINE PAL_LINE(GPIO2, 30)
#define ANT_PWR_SEL_1_GPIO_LINE PAL_LINE(GPIO2, 13)
#define ANT_PWR_SEL_2_GPIO_LINE PAL_LINE(GPIO2, 14)
#define ANT_IN_SEL_0_GPIO_LINE PAL_LINE(GPIO2, 21)
#define ANT_IN_SEL_1_GPIO_LINE PAL_LINE(GPIO2, 22)

void boardRevInit(void)
{
  /* Unlock SLCR */
  *(volatile uint32_t *)0xF8000008 = 0xDF0D;

  /* Assert FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0xf;

  /* Release FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0x0;

  /* Lock SLCR */
  *(volatile uint32_t *)0xF8000004 = 0x767B;

  palSetLineMode(SPI_SS_IMU_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(SPI_SS_IMU_GPIO_LINE);

  palSetLineMode(SPI_SS_FRONTEND_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(SPI_SS_FRONTEND_GPIO_LINE);

  palSetLineMode(CLK_SEL_GPIO_LINE, PAL_MODE_OUTPUT);
  palClearLine(CLK_SEL_GPIO_LINE);

  palSetLineMode(ANT_PWR_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_PWR_SEL_1_GPIO_LINE);

  palSetLineMode(ANT_PWR_SEL_2_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_PWR_SEL_2_GPIO_LINE);

  palSetLineMode(ANT_IN_SEL_0_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_IN_SEL_0_GPIO_LINE);

  palSetLineMode(ANT_IN_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
  palClearLine(ANT_IN_SEL_1_GPIO_LINE);
}
