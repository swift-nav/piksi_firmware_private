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

#include "antenna.h"

#include <ch.h>
#include <hal.h>
#include <swiftnav/logging.h>

#include "settings/settings.h"

#include <assert.h>
#include <string.h>
#include "clk_dac.h"

static bool rf_clk_ext = false;

static void antenna_configure(bool use_ext_clk) {
  if (use_ext_clk) {
    palSetLine(CLK_SEL_GPIO_LINE);
  } else {
    palClearLine(CLK_SEL_GPIO_LINE);
  }
}

static bool rf_clk_config_notify(struct setting *s, const char *val) {
  if (!s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return false;
  }

  antenna_configure(rf_clk_ext);

  return true;
}

void rf_clk_init(bool allow_ext_clk) {
  /* Start DAC off at it's midpoint if present */
  set_clk_dac(2222, CLK_DAC_MODE_0);

  palSetLineMode(CLK_SEL_GPIO_LINE, PAL_MODE_OUTPUT);

  if (allow_ext_clk) {
    SETTING_NOTIFY(
        "frontend", "use_ext_clk", rf_clk_ext, TYPE_BOOL, rf_clk_config_notify);
  }

  antenna_configure(rf_clk_ext);
}
