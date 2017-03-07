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

#include "sbp.h"
#include "board/v3/xadc.h"
#include "board/v3/nt1065.h"

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
  boardRevInit();
  cycle_counter_init();
}

void board_send_state(void)
{
  msg_device_monitor_t msg;

  double fe_temp = 0;
  if (!nt1065_get_temperature(&fe_temp)) {
    fe_temp = -99.99;
  }
  msg.fe_temperature = (s16)(fe_temp * 100);

  msg.dev_vin = (s16)(xadc_vin_get() * 1000);
  msg.cpu_vint = (s16)(xadc_vccint_get() * 1000);
  msg.cpu_vaux = (s16)(xadc_vccaux_get() * 1000);
  msg.cpu_temperature = (s16)(xadc_die_temp_get() * 100);

  sbp_send_msg(SBP_MSG_DEVICE_MONITOR, sizeof(msg), (u8*)&msg);
}
