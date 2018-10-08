/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "firmware_starling.h"

#include "starling_input_bridge.h"
#include "starling_integration.h"

#include <ch.h>
#include <starling/starling.h>
#include <utils/settings/settings.h>

static bool is_starling_enabled;

static bool is_starling_enabled_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, &is_starling_enabled, s->len, val);
  if (!res) {
    return false;
  }
  log_info("Firmware Starling %s.", is_starling_enabled ? "on" : "off");
  return true;
}

static void add_starling_enable_setting(void) {
  SETTING_NOTIFY("system",
                 "firmware_starling_enable",
                 TYPE_BOOL,
                 is_starling_enabled,
                 is_starling_enabled_notify);
}

void firmware_starling_init(void) {
  add_starling_enable_setting();
  starling_input_bridge_init();
  starling_initialize_api();
}

void firmware_starling_run(void) {
  starling_calc_pvt_setup();
}

bool firmware_starling_is_enabled(void) {
  return is_starling_enabled;
}




