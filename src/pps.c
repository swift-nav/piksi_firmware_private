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

#include <libswiftnav/time.h>
#include <libswiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "settings.h"
#include "main.h"
#include "timing.h"
#include "pps.h"

#define PPS_THREAD_INTERVAL_MS (10)

/** \defgroup pps Pulse-per-second (PPS)
 * Generate a pulse-per-second in alignment with GPS time.
 * \{ */

/** Number of microseconds the PPS will remain active */
static u32 pps_width_microseconds = 200000;
/** Logic level on output pin when the PPS is active */
static u8 pps_polarity = 1;
/** Offset in microseconds between GPS time and the PPS */
static s32 pps_offset_microseconds = 0;
/** Generate a pulse with the given frequency */
static double pps_frequency_hz = 1.0;
static double pps_period = 1.0;

static THD_WORKING_AREA(wa_pps_thread, 256);
static void pps_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("PPS");

  while (TRUE) {
    if (time_quality == TIME_FINE && !nap_pps_armed()) {
      gps_time_t t = get_current_gps_time();

      t.tow = (t.tow - fmod(t.tow, pps_period)) + pps_period +
          ((double)pps_offset_microseconds / 1.0e6);

      u64 next = round(gpstime2napcount(&t));
      nap_pps((u32)next);
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
bool pps_config(u32 microseconds, u8 polarity)
{
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
bool pps_config_changed(struct setting *s, const char *val)
{
  (void)s;
  (void)val;

  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return pps_config(pps_width_microseconds, pps_polarity);
  }
  return FALSE;
}

/** Settings callback for PPS frequency.
 * Updates the PPS frequency and period whenever the setting is changed.
 *
 * \param s Pointer to settings config.
 * \param val Pointer to new value.
 * \return Returns true if the change was successful, false otherwise.
 */
bool pps_frequency_changed(struct setting *s, const char *val)
{
  (void)s;
  (void)val;

  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    if (pps_frequency_hz > 20.0) {
      log_info("Invalid PPS frequency. Maximum: 20 Hz\n");
      return FALSE;
    }

    pps_period = 1.0 / pps_frequency_hz;

    if (pps_width_microseconds >= pps_period * 1.0e6) {
      log_info("PPS width needs to be smaller than PPS period.\n");
      return FALSE;
    }

    return TRUE;
  }
  return FALSE;
}

/** Set up PPS generation.
 * Sets the default value for the PPS width and starts a thread to generate
 * the pulses.
 */
void pps_setup(void)
{
  pps_config(pps_width_microseconds, pps_polarity);

  SETTING_NOTIFY("pps", "width", pps_width_microseconds, TYPE_INT,
      pps_config_changed);

  SETTING_NOTIFY("pps", "polarity", pps_polarity, TYPE_INT,
      pps_config_changed);

  SETTING_NOTIFY("pps", "offset", pps_offset_microseconds, TYPE_INT,
      settings_default_notify);

  SETTING_NOTIFY("pps", "frequency", pps_frequency_hz, TYPE_FLOAT,
      pps_frequency_changed);

  chThdCreateStatic(wa_pps_thread, sizeof(wa_pps_thread), NORMALPRIO+15,
      pps_thread, NULL);
}

/** \} */
