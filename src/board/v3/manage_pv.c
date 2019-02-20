/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* This file implements control of the PV (Position Valid) output signal.
 * The GPIO controller is still owned by firmware, so this code can't be moved
 * to Linux (yet).
 */
#include <assert.h>
#include <hal.h>

#include "calc_pvt_me.h"
#include "peripherals/antenna.h"
#include "piksi_systime.h"
#include "starling_integration.h"

#define MANAGE_PV_THREAD_PERIOD_MS 10

#define PV_MODE_TIMEOUT_MS 1500

#define MANAGE_PV_THREAD_STACK (1 * 1024)
#define MANAGE_PV_THREAD_PRIO (NORMALPRIO + 10)

typedef enum {
  DEV_NO_SIGNAL,
  DEV_ANTENNA_SHORTED,
  DEV_ANTENNA,
  DEV_TRK_AT_LEAST_FOUR,
  DEV_SPS,
  DEV_FLOAT,
  DEV_FIXED
} device_state_t;

static THD_WORKING_AREA(wa_manage_pv_thread, MANAGE_PV_THREAD_STACK);

/** Determine device state. Each LED has it's own specification how to behave
 *  under each state.
 *
 * \return Device state.
 */
static device_state_t get_device_state(void) {
  piksi_solution_info_t soln_info = {0}; 
  piksi_solution_info_get(&soln_info);

  /* Check for DGNSS. */
  bool has_any_rtk = piksi_systime_cmp(&PIKSI_SYSTIME_INIT, 
                                       &soln_info.last_time_rtk);
  if (has_any_rtk) {
    u32 elapsed = piksi_systime_elapsed_since_ms(&soln_info.last_time_rtk);
    if (elapsed < PV_MODE_TIMEOUT_MS) {
      return (soln_info.was_last_rtk_fix) ? DEV_FIXED : DEV_FLOAT;
    }
  }

  /* Check for SPS */
  bool has_any_spp = piksi_systime_cmp(&PIKSI_SYSTIME_INIT,
                                       &soln_info.last_time_spp);

  if (has_any_spp) {
    u32 elapsed = piksi_systime_elapsed_since_ms(&soln_info.last_time_spp);
    if (elapsed < PV_MODE_TIMEOUT_MS) {
      return DEV_SPS;
    }
  }

  /* Blink according to signals tracked */
  if (soln_info.num_spp_signals >= 4) {
    return DEV_TRK_AT_LEAST_FOUR;
  }

  if (antenna_shorted()) {
    return DEV_ANTENNA_SHORTED;
  }

  if (antenna_present()) {
    return DEV_ANTENNA;
  }

  return DEV_NO_SIGNAL;
}

/** Handle PV LED state.
 *
 * \param[in] dev_state   Current device state.
 *
 * \return bool, TRUE for ON, FALSE for OFF.
 */
static bool handle_pv(device_state_t dev_state) {
  switch (dev_state) {
    case DEV_NO_SIGNAL:
    case DEV_ANTENNA_SHORTED:
    case DEV_ANTENNA:
    case DEV_TRK_AT_LEAST_FOUR:
      return FALSE;
      break;
    case DEV_SPS:
    case DEV_FLOAT:
    case DEV_FIXED:
      return TRUE;
      break;
    default:
      assert(!"Unknown mode");
      break;
  }
}

static void manage_pv_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("manage PV");

  /* Configure GPIO */
  chSysLock();
  { palSetLineMode(POS_VALID_GPIO_LINE, PAL_MODE_OUTPUT); }
  chSysUnlock();

  palClearLine(POS_VALID_GPIO_LINE);

  while (TRUE) {
    device_state_t dev_state = get_device_state();
    bool pv_state = handle_pv(dev_state);
    palWriteLine(POS_VALID_GPIO_LINE, pv_state ? PAL_HIGH : PAL_LOW);

    piksi_systime_sleep_ms(MANAGE_PV_THREAD_PERIOD_MS);
  }
}

void manage_pv_setup(void) {
  chThdCreateStatic(wa_manage_pv_thread,
                    sizeof(wa_manage_pv_thread),
                    MANAGE_PV_THREAD_PRIO,
                    manage_pv_thread,
                    NULL);
}
