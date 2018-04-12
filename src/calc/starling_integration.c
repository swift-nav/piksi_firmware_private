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

#include "starling_integration.h"
#include "settings/settings.h"
#include "starling_threads.h"

/*******************************************************************************
 * Globals
 ******************************************************************************/
bool enable_glonass = true;

/*******************************************************************************
 * Local Helpers
 ******************************************************************************/

static bool enable_fix_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool is_fix_enabled = (value != 0);
  starling_set_is_fix_enabled(is_fix_enabled);
  *(dgnss_filter_t *)s->addr = value;
  return ret;
}

static bool set_max_age(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  starling_set_max_correction_age(value);
  *(int *)s->addr = value;
  return ret;
}

static bool set_is_glonass_enabled(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool is_glonass_enabled = (value != 0);
  starling_set_is_glonass_enabled(is_glonass_enabled);
  *(dgnss_filter_t *)s->addr = value;
  return ret;
}

static void initialize_starling_settings(void) {
  static const char *const dgnss_filter_enum[] = {"Float", "Fixed", NULL};
  static struct setting_type dgnss_filter_setting;
  static dgnss_filter_t dgnss_filter_mode = FILTER_FIXED;
  int TYPE_GNSS_FILTER =
      settings_type_register_enum(dgnss_filter_enum, &dgnss_filter_setting);

  SETTING_NOTIFY("solution",
                 "dgnss_filter",
                 dgnss_filter_mode,
                 TYPE_GNSS_FILTER,
                 enable_fix_mode);

  static u32 max_age_of_differential = 0;
  SETTING_NOTIFY("solution",
                 "correction_age_max",
                 max_age_of_differential,
                 TYPE_INT,
                 set_max_age);
  SETTING_NOTIFY("solution",
                 "enable_glonass",
                 enable_glonass,
                 TYPE_BOOL,
                 set_is_glonass_enabled);
}

/*******************************************************************************
 * Starling Integration API
 ******************************************************************************/

void starling_calc_pvt_setup() {
  starling_setup();
  initialize_starling_settings();
}
