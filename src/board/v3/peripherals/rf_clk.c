/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>
#include <hal.h>
#include <math.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "antenna.h"
#include "clk_dac.h"
#include "piksi_systime.h"
#include "settings/settings_client.h"

static bool rf_clk_ext = false;
static bool clock_steering_active = false;
static uint16_t dac_value = 2100;
static piksi_systime_t next_clocksteering_deadline = PIKSI_SYSTIME_INIT;

static void antenna_configure(bool use_ext_clk) {
  if (use_ext_clk) {
    palSetLine(CLK_SEL_GPIO_LINE);
  } else {
    palClearLine(CLK_SEL_GPIO_LINE);
  }
}

static int rf_clk_config_notify(void *ctx) {
  (void)ctx;

  antenna_configure(rf_clk_ext);

  return SETTINGS_WR_OK;
}

static int clock_steering_notify(void *ctx) {
  (void)ctx;

  return SETTINGS_WR_OK;
}

void rf_clk_init(bool allow_ext_clk) {
  /* Start DAC off at it's midpoint if present */
  set_clk_dac(dac_value, CLK_DAC_MODE_0);

  palSetLineMode(CLK_SEL_GPIO_LINE, PAL_MODE_OUTPUT);

  if (allow_ext_clk) {
    SETTING_NOTIFY("frontend",
                   "use_ext_clk",
                   rf_clk_ext,
                   SETTINGS_TYPE_BOOL,
                   rf_clk_config_notify);
  } else {
    antenna_configure(rf_clk_ext);
  }

  SETTING_NOTIFY("frontend",
                 "activate_clock_steering",
                 clock_steering_active,
                 SETTINGS_TYPE_BOOL,
                 clock_steering_notify);
}

void clock_steer(s32 clk_drift_ppb) {
  if (!clock_steering_active) return;

  bool slow_down = (clk_drift_ppb > 0);
  s32 abs_clk_drift_ppb = ABS(clk_drift_ppb);
  /* Empirically, with our current DAC and the accuracy of the drift solve,
   * 5 ns/s is a safe threshold to avoid hysteresis
   */
  if (abs_clk_drift_ppb < 5) return;

  /* bound steering to 1 Hz */
  if (abs_clk_drift_ppb > 1024) {
    abs_clk_drift_ppb = 1024;
  }

  /* compare against the next deadline */
  piksi_systime_t now;
  piksi_systime_get(&now);
  if (piksi_systime_sub_us(&now, &next_clocksteering_deadline) <= 0) {
    return;
  }

  /* arbitrary [1:30] second delay function */
  u16 adj_deadline_s = (u16)lrintf(31.0f - 3.0f * log2f(abs_clk_drift_ppb));
  /* DAC set point bounded for safety */
  if (slow_down && (dac_value > 1900)) {
    dac_value--;
    set_clk_dac(dac_value, CLK_DAC_MODE_0);
    piksi_systime_inc_s(&now, adj_deadline_s);
    next_clocksteering_deadline = now;
    log_info("clk_drift_ppb %ld adj_deadline_s %d, set DAC to %d",
             clk_drift_ppb,
             adj_deadline_s,
             dac_value);
  }
  if (!slow_down && (dac_value < 2300)) {
    dac_value++;
    set_clk_dac(dac_value, CLK_DAC_MODE_0);
    piksi_systime_inc_s(&now, adj_deadline_s);
    next_clocksteering_deadline = now;
    log_info("clk_drift_ppb %ld adj_deadline_s %d, set DAC to %d",
             clk_drift_ppb,
             adj_deadline_s,
             dac_value);
  }
}
