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
#include "hal.h"
#include "zynq7000.h"

#include <libswiftnav/logging.h>

#define CLK_SEL_GPIO_LINE PAL_LINE(GPIO2, 30)
#define ANT_PWR_SEL_1_GPIO_LINE PAL_LINE(GPIO2, 13)
#define ANT_PWR_SEL_2_GPIO_LINE PAL_LINE(GPIO2, 14)
#define ANT_IN_SEL_0_GPIO_LINE PAL_LINE(GPIO2, 21)
#define ANT_IN_SEL_1_GPIO_LINE PAL_LINE(GPIO2, 22)

#define REBOOT_STATUS (*(volatile uint32_t *)0xF8000258)
#define REBOOT_STATUS_POR (1 << 22)
#define REBOOT_STATUS_SRST (1 << 21)
#define REBOOT_STATUS_DBG_RST (1 << 20)
#define REBOOT_STATUS_SLC_RST (1 << 19)
#define REBOOT_STATUS_AWDT1_RST (1 << 18)
#define REBOOT_STATUS_AWDT0_RST (1 << 17)
#define REBOOT_STATUS_SWDT_RST (1 << 16)
#define REBOOT_STATUS_REASON (0x3F << 16)

const PALConfig pal_default_config;
const WDGConfig board_wdg_config = {
  .period_ms = 30000,
};

static void cycle_counter_init(void)
{
  /* Set up TTC0_2 with period of ZYNQ7000_CPU_1x_FREQUENCY_Hz / 2^10 */
  TTC0->CLKCTRL[2] =  (TTC_CLKCTRL_SRC_PCLK << TTC_CLKCTRL_SRC_Pos) |
                      (9 << TTC_CLKCTRL_PSVAL_Pos) |
                      (1 << TTC_CLKCTRL_PSEN_Pos);
  TTC0->INTERVAL[2] = 0xffff;
  TTC0->CNTCTRL[2] =  (1 << TTC_CNTCTRL_RESET_Pos) |
                      (1 << TTC_CNTCTRL_INTERVAL_Pos);
}

/*
 * Board-specific initialization code.
 */
void boardInit(void)
{
  /* Unlock SLCR */
  *(volatile uint32_t *)0xF8000008 = 0xDF0D;

  /* Assert FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0xf;

  /* Release FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0x0;

  /* Lock SLCR */
  *(volatile uint32_t *)0xF8000004 = 0x767B;


  palSetLine(SPI_SS_FRONTEND_GPIO_LINE);
  palSetLineMode(SPI_SS_FRONTEND_GPIO_LINE, PAL_MODE_OUTPUT);

  palClearLine(CLK_SEL_GPIO_LINE);
  palSetLineMode(CLK_SEL_GPIO_LINE, PAL_MODE_OUTPUT);

  palSetLine(ANT_PWR_SEL_1_GPIO_LINE);
  palSetLineMode(ANT_PWR_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);

  palSetLine(ANT_PWR_SEL_2_GPIO_LINE);
  palSetLineMode(ANT_PWR_SEL_2_GPIO_LINE, PAL_MODE_OUTPUT);

  palSetLine(ANT_IN_SEL_0_GPIO_LINE);
  palSetLineMode(ANT_IN_SEL_0_GPIO_LINE, PAL_MODE_OUTPUT);

  palClearLine(ANT_IN_SEL_1_GPIO_LINE);
  palSetLineMode(ANT_IN_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);


  cycle_counter_init();
}

void board_preinit_hook(void)
{
  uint32_t s = REBOOT_STATUS;
  if (s & REBOOT_STATUS_REASON) {
    if (s & (REBOOT_STATUS_SWDT_RST | REBOOT_STATUS_AWDT1_RST |
                  REBOOT_STATUS_AWDT0_RST))
      log_error("Piksi has reset due to a watchdog timeout.");
    if (s & REBOOT_STATUS_SLC_RST)
      log_info("Software reset detected.");
    log_info("Reset reason: %02X", s >> 16);
  }
  REBOOT_STATUS &= 0xff000000;

}

