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

#ifndef _BOARD_REV_H_
#define _BOARD_REV_H_

/*
 * Setup for the Piksiv3 EVT2 board.
 */

/*
 * Board identifier.
 */
#define BOARD_PIKSIV3_EVT2
#define BOARD_NAME "Piksiv3 EVT2"

#define SPI_SS_IMU_GPIO_LINE PAL_LINE(GPIO2, 31)
#define SPI_SS_FRONTEND_GPIO_LINE PAL_LINE(GPIO3, 0)
#define SPI_SS_CLK_DAC_GPIO_LINE PAL_LINE(GPIO3, 1)

#define FRONTEND_SPI SPID1
#define FRONTEND_SPI_CONFIG {0, SPI_MODE_0, \
                             SPI_CLK_DIV_16, SPI_SS_FRONTEND_GPIO_LINE}
#define CLK_DAC_SPI SPID2
#define CLK_DAC_SPI_CONFIG {0, SPI_MODE_1, \
                             SPI_CLK_DIV_16, SPI_SS_CLK_DAC_GPIO_LINE}

#define LED_I2C I2CD2
#define LED_I2C_CONFIG {.clk = 200000}

#define TCXO_FREQ_STAB  0.28f  /* Piksi V3 TCXO frequency stability [ppm] */

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardRevInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_REV_H_ */
