/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "starling_sbp_settings.h"

#include <assert.h>
#include <calc/starling_integration.h>
#include <libsettings/settings.h>

#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "sbp_settings_client.h"

/********************************************************************************/
#define DFLT_CORRECTION_AGE_MAX_S 30

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

/********************************************************************************/
static SbpSettingsClient *settings_client = NULL;

/********************************************************************************/
static bool enable_glonass = true;
static bool enable_galileo = true;
static bool enable_beidou = true;
static dgnss_filter_t dgnss_filter_mode = FILTER_FIXED;
static pvt_driver_solution_mode_t dgnss_soln_mode =
    PVT_DRIVER_SOLN_MODE_LOW_LATENCY;
static float glonass_downweight_factor = 4.0;
static u32 corr_age_max = DFLT_CORRECTION_AGE_MAX_S;
static PROCESS_NOISE_MOTION_TYPE dynamic_motion_model = HIGH_DYNAMICS;

/* TODO(kevin) make this non global. */
bool send_heading = false;
bool disable_raim = false;
double heading_offset = 0.0;

/********************************************************************************/
static int heading_offset_changed(void *ctx) {
  (void)ctx;
  /* Check that -180.0 <= new heading_offset setting value <= 180.0. */
  if (fabs(heading_offset) > 180.0) {
    log_error(
        "Invalid heading offset setting of %3.1f, max is %3.1f, min is %3.1f",
        heading_offset,
        180.0,
        -180.0);
    return SETTINGS_WR_VALUE_REJECTED;
  }
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int enable_fix_mode(void *ctx) {
  (void)ctx;
  pvt_driver_set_is_rtk_fixed_enabled(pvt_driver,
                                      FILTER_FIXED == dgnss_filter_mode);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_dgnss_soln_mode(void *ctx) {
  (void)ctx;
  s32 max_sats;
  bool ret;

  pvt_driver_set_solution_mode(pvt_driver, dgnss_soln_mode);

  ret = get_max_sats(soln_freq_setting, dgnss_soln_mode, &max_sats);
  assert(ret);
  pvt_driver_set_max_sats(pvt_driver, max_sats);

  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_max_age(void *ctx) {
  (void)ctx;
  if (0 >= corr_age_max) {
    log_error("Invalid correction age max value %" PRIu32, corr_age_max);
    return SETTINGS_WR_SETTING_REJECTED;
  }
  pvt_driver_set_max_correction_age(pvt_driver, corr_age_max);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_is_glonass_enabled(void *ctx) {
  (void)ctx;
  pvt_driver_set_is_glonass_enabled(pvt_driver, enable_glonass);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_is_galileo_enabled(void *ctx) {
  (void)ctx;
  pvt_driver_set_is_galileo_enabled(pvt_driver, enable_galileo);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_is_beidou_enabled(void *ctx) {
  (void)ctx;
  pvt_driver_set_is_beidou_enabled(pvt_driver, enable_beidou);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_dynamic_motion_model(void *ctx) {
  (void)ctx;
  pvt_driver_set_process_noise_motion(pvt_driver, dynamic_motion_model);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static int set_glonass_downweight_factor(void *ctx) {
  (void)ctx;

  pvt_driver_set_downweight_factor(
      pvt_driver, CODE_GLO_L1OF, glonass_downweight_factor);
  pvt_driver_set_downweight_factor(
      pvt_driver, CODE_GLO_L2OF, glonass_downweight_factor);
  return SETTINGS_WR_OK;
}

/********************************************************************************/
static void init_settings_client(const SbpDuplexLink *sbp_link) {
  if (settings_client) {
    log_error("Unexpected attempt to reinitialize settings client.");
    return;
  }

  settings_client = sbp_settings_client_create(sbp_link);
  if (!settings_client) {
    log_error("Unable to create settings client.");
  }
}

/********************************************************************************/
void starling_register_sbp_settings(const SbpDuplexLink *sbp_link) {
  /* Make sure we have the client prepared. */
  init_settings_client(sbp_link);
  assert(settings_client);

  /* Prepare enums we will use for Starling settings. */
  static const char *const dgnss_filter_enum[] = {"Float", "Fixed", NULL};
  settings_type_t dgnss_filter_setting;
  sbp_settings_client_register_enum(
      settings_client, dgnss_filter_enum, &dgnss_filter_setting);

  static const char *const process_noise_enum[] = {
      "High Dynamics", "High Horizontal Dynamics", "Low Dynamics", NULL};
  settings_type_t process_noise_setting;
  sbp_settings_client_register_enum(
      settings_client, process_noise_enum, &process_noise_setting);

  static const char *const dgnss_soln_mode_enum[] = {
      "Low Latency", "Time Matched", "No DGNSS", NULL};
  settings_type_t dgnss_soln_mode_setting;
  sbp_settings_client_register_enum(
      settings_client, dgnss_soln_mode_enum, &dgnss_soln_mode_setting);

  /* Register actual settings. */
  sbp_settings_client_register(settings_client,
                               "solution",
                               "dgnss_filter",
                               &dgnss_filter_mode,
                               sizeof(dgnss_filter_mode),
                               dgnss_filter_setting,
                               enable_fix_mode,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "dynamic_motion_model",
                               &dynamic_motion_model,
                               sizeof(dynamic_motion_model),
                               process_noise_setting,
                               set_dynamic_motion_model,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "correction_age_max",
                               &corr_age_max,
                               sizeof(corr_age_max),
                               SETTINGS_TYPE_INT,
                               set_max_age,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "enable_glonass",
                               &enable_glonass,
                               sizeof(enable_glonass),
                               SETTINGS_TYPE_BOOL,
                               set_is_glonass_enabled,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "enable_galileo",
                               &enable_galileo,
                               sizeof(enable_galileo),
                               SETTINGS_TYPE_BOOL,
                               set_is_galileo_enabled,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "enable_beidou",
                               &enable_beidou,
                               sizeof(enable_beidou),
                               SETTINGS_TYPE_BOOL,
                               set_is_beidou_enabled,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "glonass_measurement_std_downweight_factor",
                               &glonass_downweight_factor,
                               sizeof(glonass_downweight_factor),
                               SETTINGS_TYPE_FLOAT,
                               set_glonass_downweight_factor,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "dgnss_solution_mode",
                               &dgnss_soln_mode,
                               sizeof(dgnss_soln_mode),
                               dgnss_soln_mode_setting,
                               set_dgnss_soln_mode,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "heading_offset",
                               &heading_offset,
                               sizeof(heading_offset),
                               SETTINGS_TYPE_FLOAT,
                               heading_offset_changed,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "send_heading",
                               &send_heading,
                               sizeof(send_heading),
                               SETTINGS_TYPE_BOOL,
                               NULL,
                               NULL);

  sbp_settings_client_register(settings_client,
                               "solution",
                               "disable_raim",
                               &disable_raim,
                               sizeof(disable_raim),
                               SETTINGS_TYPE_BOOL,
                               NULL,
                               NULL);
}
