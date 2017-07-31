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

#ifndef _BOARD_H_
#define _BOARD_H_

#include <gic.h>
#include "board_rev.h"

#define IRQ_ID_NAP IRQ_ID_FPGA2
#define NAP_IRQ_PRIORITY 4

#define IRQ_ID_NAP_TRACK IRQ_ID_FPGA3
#define NAP_TRACK_IRQ_PRIORITY 4

#define IRQ_ID_RTC IRQ_ID_FPGA7
#define RTC_IRQ_PRIORITY 4

#define IRQ_ID_FRONTEND_AOK IRQ_ID_FPGA9
#define FRONTEND_AOK_IRQ_PRIORITY 4

#define IRQ_ID_IMU_INT1 IRQ_ID_FPGA5
#define IMU_INT1_IRQ_PRIORITY 4

#define IRQ_ID_IMU_INT2 IRQ_ID_FPGA6
#define IMU_INT2_IRQ_PRIORITY 4

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
  void board_send_state(void);
  bool board_is_duro(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
