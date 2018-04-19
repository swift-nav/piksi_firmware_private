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

#include <assert.h>
#include <ch.h>

#include "starling_integration.h"
#include "settings/settings.h"
#include "starling_threads.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define STARLING_THREAD_PRIORITY (HIGHPRIO - 4)
#define STARLING_THREAD_STACK (6 * 1024 * 1024)

/*******************************************************************************
 * Globals
 ******************************************************************************/
bool enable_glonass = true;

/*******************************************************************************
 * Locals
 ******************************************************************************/

/* Working area for the main starling thread. */
static THD_WORKING_AREA(wa_starling_thread, STARLING_THREAD_STACK);

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

static bool set_dgnss_soln_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  dgnss_solution_mode_t dgnss_soln_mode = value;
  starling_set_solution_mode(dgnss_soln_mode);
  *(dgnss_solution_mode_t *)s->addr = dgnss_soln_mode;
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
  *(bool *)s->addr = is_glonass_enabled;
  return ret;
}

static bool set_glonass_downweight_factor(struct setting *s, const char *val) {
  float value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  starling_set_glonass_downweight_factor(value);
  *(float *)s->addr = value;
  return ret;
}

static bool set_disable_klobuchar(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool disable_klobuchar = (value != 0);
  starling_set_is_time_matched_klobuchar_enabled(!disable_klobuchar);
  *(bool *)s->addr = disable_klobuchar;
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

  static float glonass_downweight_factor = 4.0;
  SETTING_NOTIFY("solution",
                 "glonass_measurement_std_downweight_factor",
                 glonass_downweight_factor,
                 TYPE_FLOAT,
                 set_glonass_downweight_factor);

  static bool disable_klobuchar = false;
  SETTING_NOTIFY("solution",
                 "disable_klobuchar_correction",
                 disable_klobuchar,
                 TYPE_BOOL,
                 set_disable_klobuchar);

  static const char *const dgnss_soln_mode_enum[] = {
      "Low Latency", "Time Matched", "No DGNSS", NULL};
  static struct setting_type dgnss_soln_mode_setting;
  int TYPE_GNSS_SOLN_MODE = settings_type_register_enum(
      dgnss_soln_mode_enum, &dgnss_soln_mode_setting);
  static dgnss_solution_mode_t dgnss_soln_mode = STARLING_SOLN_MODE_LOW_LATENCY;
  SETTING_NOTIFY("solution",
                 "dgnss_solution_mode",
                 dgnss_soln_mode,
                 TYPE_GNSS_SOLN_MODE,
                 set_dgnss_soln_mode);
}

static THD_FUNCTION(initialize_and_run_starling, arg) {
  (void)arg;
  chRegSetThreadName("starling");

  initialize_starling_settings();

  /* This runs forever. */
  starling_run();

  /* Never get here. */
  log_error("Starling Engine has unexpectedly terminated.");
  assert(0);
  for (;;) {
  }
}

/*******************************************************************************
 * Starling Integration API
 ******************************************************************************/

void starling_calc_pvt_setup() {
  /* Start main starling thread. */
  chThdCreateStatic(wa_starling_thread,
                    sizeof(wa_starling_thread),
                    STARLING_THREAD_PRIORITY,
                    initialize_and_run_starling,
                    NULL);
}
