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
#include "slcr_mio.h"

#define LINE_TO_MIO(line) (32 * PAL_PORT(line) + PAL_PAD(line))

#define LED_GPIO_LINE PAL_LINE(GPIO1, 15)
#define BUTTON_GPIO_LINE PAL_LINE(GPIO1, 19)

#define SPI_MOSI_GPIO_LINE PAL_LINE(GPIO0, 10)
#define SPI_MISO_GPIO_LINE PAL_LINE(GPIO0, 11)
#define SPI_CLK_GPIO_LINE PAL_LINE(GPIO0, 12)
#define SPI_SS_GPIO_LINE PAL_LINE(GPIO0, 13)

static void mio_configure(uint8_t mio, uint8_t function, bool tristate,
                          bool pullup, bool fast)
{
  uint32_t clear_mask = (SLCR_MIO_PIN_PULLUP_Msk |
                         SLCR_MIO_PIN_SPEED_Msk |
                         SLCR_MIO_PIN_L3_SEL_Msk |
                         SLCR_MIO_PIN_L2_SEL_Msk |
                         SLCR_MIO_PIN_L1_SEL_Msk |
                         SLCR_MIO_PIN_L0_SEL_Msk |
                         SLCR_MIO_PIN_TRI_ENABLE_Msk);

  uint32_t set_mask = (function << SLCR_MIO_PIN_L0_SEL_Pos);
  if (tristate)
    set_mask |= SLCR_MIO_PIN_TRI_ENABLE_Msk;
  if (pullup)
    set_mask |= SLCR_MIO_PIN_PULLUP_Msk;
  if (fast)
    set_mask |= SLCR_MIO_PIN_SPEED_Msk;

  uint32_t mio_pin_reg = SLCR_MIO->MIO_PIN[mio];
  mio_pin_reg &= ~clear_mask;
  mio_pin_reg |= set_mask;
  SLCR_MIO->MIO_PIN[mio] = mio_pin_reg;
}

void boardRevInit(void)
{
  /* Unlock SLCR */
  *(volatile uint32_t *)0xF8000008 = 0xDF0D;

  /* Enable UART0 and UART1 clocks */
  *(volatile uint32_t *)0xF800012C |= (1 << 20) | (1 << 21);

  /* UART REFCLK = 1GHz / 20 = 50MHz */
  *(volatile uint32_t *)0xF8000154 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000154 |= (20 << 8);
  *(volatile uint32_t *)0xF8000154 |= (1 << 0) | (1 << 1);

  /* Enable SPI0 and SPI1 clocks */
  *(volatile uint32_t *)0xF800012C |= (1 << 14) | (1 << 15);

  /* SPI REFCLK = 1GHz / 20 = 50MHz */
  *(volatile uint32_t *)0xF8000158 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000158 |= (20 << 8);
  *(volatile uint32_t *)0xF8000158 |= (1 << 0) | (1 << 1);

  /* PCAP CLK = 1GHz / 5 = 200MHz */
  *(volatile uint32_t *)0xF8000168 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000168 |= (5 << 8);
  *(volatile uint32_t *)0xF8000168 |= (1 << 0);

  /* Assert FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0xf;

  /* FPGA_CLK0 = 1GHz / 10 = 100MHz */
  *(volatile uint32_t *)0xF8000170 &= ~(0x3F << 20);
  *(volatile uint32_t *)0xF8000170 |= (1 << 20);
  *(volatile uint32_t *)0xF8000170 &= ~(0x3F << 8);
  *(volatile uint32_t *)0xF8000170 |= (10 << 8);

  /* Release FPGA resets */
  *(volatile uint32_t *)0xF8000240 = 0x0;

  /* Configure MIOs */
  mio_configure(LINE_TO_MIO(SPI_MOSI_GPIO_LINE), 5 << 4, false, false, false);
  mio_configure(LINE_TO_MIO(SPI_MISO_GPIO_LINE), 5 << 4, true, false, false);
  mio_configure(LINE_TO_MIO(SPI_CLK_GPIO_LINE), 5 << 4, false, false, false);
  mio_configure(LINE_TO_MIO(SPI_SS_GPIO_LINE), 0, false, false, false);

  /* Lock SLCR */
  *(volatile uint32_t *)0xF8000004 = 0x767B;

  /* Configure button and LED pins */
  palSetLineMode(BUTTON_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(LED_GPIO_LINE, PAL_MODE_OUTPUT);

  /* Configure frontend SPI pins */
  palSetLineMode(SPI_MOSI_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLineMode(SPI_MISO_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(SPI_CLK_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLineMode(SPI_SS_GPIO_LINE, PAL_MODE_OUTPUT);
}
