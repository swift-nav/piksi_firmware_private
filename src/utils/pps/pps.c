/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Johannes Walter <johannes@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>

#include <ch.h>

#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "calc/calc_pvt_me.h"
#include "main.h"
#include "pps.h"
#include "settings/settings_client.h"
#include "timing/timing.h"

#define PPS_THREAD_INTERVAL_MS (10)
#define PPS_THREAD_STACK (2 * 1024)
#define PPS_THREAD_PRIORITY (NORMALPRIO + 15)

#define PPS_FW_OFFSET_S 620e-9

/** \defgroup pps Pulse-per-second (PPS)
 * Generate a pulse-per-second in alignment with GPS time.
 * \{ */

/** Number of microseconds the PPS will remain active */
static u32 pps_width_us = 2000;
/** Logic level on output pin when the PPS is active */
static u8 pps_polarity = 1;
/** Offset in nanoseconds between GPS time and the PPS */
static s32 pps_offset_ns = 0;
/** Generate a pulse with the given frequency */
static double pps_frequency_hz = 1.0;
static double pps_period = 1.0;

pps_propagation_mode_t pps_propagation_mode = PPS_PROP_MODE_TIMEOUT;

static float pps_propagation_timeout = 5.0;

/** Determines whether PPS should be sent from settings and state
 *
 * \return true if PPS should be sent, false otherwise.
 */
static bool output_pps(gps_time_t *in_time) {
  switch (pps_propagation_mode) {
    case PPS_PROP_MODE_NONE:
      /* if time update within 1 time solve solution interval
       * it means we have had a soln or have had one very recently,
       * therefore we return true */
      if (time_updated_within(in_time, ME_PVT_INTERVAL_S + 0.005)) {
        return true;
      }
      break;
    case PPS_PROP_MODE_TIMEOUT:
      /* it we are within timeout duration, return true*/
      if (time_updated_within(in_time, pps_propagation_timeout)) {
        return true;
      }
      break;
    case PPS_PROP_MODE_UNLIMITED:
      /* if set to unlimited, return true */
      return true;
      break;
    /* TODO: implement accuracy threshold */
    default:
      return false;
  }
  return false;
}

static THD_WORKING_AREA(wa_pps_thread, PPS_THREAD_STACK);
static void pps_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("PPS");

  while (TRUE) {
    if (get_time_quality() > TIME_UNKNOWN && !nap_pps_armed()) {
      gps_time_t t = get_current_time();
      if (output_pps(&t)) {
        t.tow = (t.tow - fmod(t.tow, pps_period)) + pps_period +
                ((double)pps_offset_ns / 1.0e9) + PPS_FW_OFFSET_S;

        u64 next = gpstime2napcount(&t);
        nap_pps((u32)next);
      }
    }
    chThdSleepMilliseconds(PPS_THREAD_INTERVAL_MS);
  }
}

/** Set PPS config.
 * Sets the PPS pulse width and poliarity.
 *
 * \param microseconds Pulse width in microseconds.
 * \param polarity Active logic level.
 * \return Returns true if value is within valid range, false otherwise.
 */
static bool pps_config(u32 microseconds, u8 polarity) {
  if (microseconds < 1 || microseconds >= 1e6) {
    log_info("Invalid PPS width. Valid range: 1-999999\n");
    return FALSE;
  }

  if (polarity > 1) {
    log_info("Invalid PPS polarity. Valid values: 0, 1\n");
    return FALSE;
  }

  nap_pps_config(microseconds, polarity);
  return TRUE;
}

/** Settings callback for PPS config.
 * Updates the PPS width and polarity whenever the setting is changed.
 *
 * \param s Pointer to settings config.
 * \param val Pointer to new value.
 * \return Returns true if the change was successful, false otherwise.
 */
static int pps_config_changed(void *ctx) {
  (void)ctx;

  if (!pps_config(pps_width_us, pps_polarity)) {
    return SETTINGS_WR_VALUE_REJECTED;
  }

  return SETTINGS_WR_OK;
}

/** Settings callback for PPS frequency.
 * Updates the PPS frequency and period whenever the setting is changed.
 *
 * \param s Pointer to settings config.
 * \param val Pointer to new value.
 * \return Returns true if the change was successful, false otherwise.
 */
static int pps_frequency_changed(void *ctx) {
  (void)ctx;

  if (pps_frequency_hz > 20.0) {
    log_info("Invalid PPS frequency. Maximum: 20 Hz\n");
    return SETTINGS_WR_VALUE_REJECTED;
  }

  double pps_period_cand = 1.0 / pps_frequency_hz;

  if (pps_width_us >= pps_period_cand * 1.0e6) {
    log_info("PPS width needs to be smaller than PPS period.\n");
    return SETTINGS_WR_VALUE_REJECTED;
  }

  pps_period = pps_period_cand;

  return SETTINGS_WR_OK;
}

/** Set up PPS generation.
 * Sets the default value for the PPS width and starts a thread to generate
 * the pulses.
 */
void pps_setup(void) {
  pps_config(pps_width_us, pps_polarity);
  static const char const *pps_propagation_mode_enum[] = {
      "None", "Time Limited", "Unlimited", NULL};

  settings_type_t pps_propagation_setting;
  settings_api_register_enum(pps_propagation_mode_enum,
                             &pps_propagation_setting);
  SETTING(
      "pps", "propagation_mode", pps_propagation_mode, pps_propagation_setting);

  SETTING_NOTIFY(
      "pps", "width", pps_width_us, SETTINGS_TYPE_INT, pps_config_changed);

  SETTING_NOTIFY(
      "pps", "polarity", pps_polarity, SETTINGS_TYPE_INT, pps_config_changed);

  SETTING("pps", "offset", pps_offset_ns, SETTINGS_TYPE_INT);

  SETTING_NOTIFY("pps",
                 "frequency",
                 pps_frequency_hz,
                 SETTINGS_TYPE_FLOAT,
                 pps_frequency_changed);

  SETTING("pps",
          "propagation_timeout",
          pps_propagation_timeout,
          SETTINGS_TYPE_FLOAT);

  chThdCreateStatic(wa_pps_thread,
                    sizeof(wa_pps_thread),
                    PPS_THREAD_PRIORITY,
                    pps_thread,
                    NULL);
}

/** \} */
