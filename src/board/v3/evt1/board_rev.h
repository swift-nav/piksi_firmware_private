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
 * Setup for the Piksiv3 EVT1 board.
 */

/*
 * Board identifier.
 */
#define BOARD_PIKSIV3_EVT1
#define BOARD_NAME "Piksiv3 EVT1"

#define SPI_SS_IMU_GPIO_LINE PAL_LINE(GPIO2, 31)
#define SPI_SS_FRONTEND_GPIO_LINE PAL_LINE(GPIO3, 0)

#define ANT_PWR_SEL_1_GPIO_LINE PAL_LINE(GPIO2, 13)
#define ANT_PWR_SEL_2_GPIO_LINE PAL_LINE(GPIO2, 14)
#define ANT_IN_SEL_0_GPIO_LINE PAL_LINE(GPIO2, 21)
#define ANT_IN_SEL_1_GPIO_LINE PAL_LINE(GPIO2, 22)
#define ANT_PRESENT_1_GPIO_LINE PAL_LINE(GPIO2, 15)
#define ANT_PRESENT_2_GPIO_LINE PAL_LINE(GPIO2, 16)
#define ANT_NFAULT_1_GPIO_LINE PAL_LINE(GPIO2, 27)
#define ANT_NFAULT_2_GPIO_LINE PAL_LINE(GPIO2, 26)

#define FRONTEND_SPI SPID1
#define FRONTEND_SPI_CONFIG {0, SPI_MODE_0, \
                             SPI_CLK_DIV_16, SPI_SS_FRONTEND_GPIO_LINE}

#define LED_I2C I2CD2
#define LED_I2C_CONFIG {.clk = 200000}

#define TCXO_FREQ_HZ 10e6          /* TCXO nominal frequency [Hz] */

/* Piksi V3 TCXO nominal temperature frequency stability [ppm] */
#define TCXO_FREQ_STAB_PPM  0.28f
/* Maximum TCXO offset. Includes TCXO nominal temperature frequency stability,
   aging and soldering [ppm] */
#define TCXO_FREQ_OFFSET_MAX_PPM (TCXO_FREQ_STAB_PPM + 2.42)

/* TCXO offset to Hz conversion factor.
   With TCXO frequency set to 10MHz the IF is computed like this:
   1575.42e6 - 10e6 * 159 = 14.58e6 [Hz] */
#define GPS_L1CA_TCXO_PPM_TO_HZ (TCXO_FREQ_HZ * 1e-6 * 159.)

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
