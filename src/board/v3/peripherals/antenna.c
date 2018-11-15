/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
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

#include "settings/settings_client.h"

#include <assert.h>
#include <string.h>

typedef enum { ANTENNA_MODE_PRIMARY, ANTENNA_MODE_SECONDARY } antenna_mode_t;

static const char *const antenna_mode_strings[] = {
    "Primary", "Secondary", NULL};

static antenna_mode_t antenna_mode = ANTENNA_MODE_PRIMARY;
static bool antenna_bias = true;

static void antenna_configure(antenna_mode_t mode, bool bias) {
  /* SEL is active low, so "clear" is on and "set" is off. PWR is active high.
   */
  switch (mode) {
    case ANTENNA_MODE_PRIMARY: {
      palSetLine(ANT_IN_SEL_2_GPIO_LINE);
      palClearLine(ANT_PWR_SEL_2_GPIO_LINE);

      palClearLine(ANT_IN_SEL_1_GPIO_LINE);
      if (bias) {
        palSetLine(ANT_PWR_SEL_1_GPIO_LINE);
      } else {
        palClearLine(ANT_PWR_SEL_1_GPIO_LINE);
      }
    } break;

    case ANTENNA_MODE_SECONDARY: {
      palSetLine(ANT_IN_SEL_1_GPIO_LINE);
      palClearLine(ANT_PWR_SEL_1_GPIO_LINE);

      palClearLine(ANT_IN_SEL_2_GPIO_LINE);
      if (bias) {
        palSetLine(ANT_PWR_SEL_2_GPIO_LINE);
      } else {
        palClearLine(ANT_PWR_SEL_2_GPIO_LINE);
      }
    } break;

    default: { assert(!"Invalid antenna mode"); } break;
  }
}

static int antenna_config_notify(void *ctx) {
  (void)ctx;

  antenna_configure(antenna_mode, antenna_bias);

  return SETTINGS_WR_OK;
}

void antenna_init(void) {
  /* Configure GPIO */
  chSysLock();
  {
    palSetLineMode(ANT_PWR_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
    palSetLineMode(ANT_PWR_SEL_2_GPIO_LINE, PAL_MODE_OUTPUT);
    palSetLineMode(ANT_IN_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
    palSetLineMode(ANT_IN_SEL_2_GPIO_LINE, PAL_MODE_OUTPUT);
    palSetLineMode(ANT_PRESENT_1_GPIO_LINE, PAL_MODE_INPUT);
    palSetLineMode(ANT_PRESENT_2_GPIO_LINE, PAL_MODE_INPUT);
    palSetLineMode(ANT_NFAULT_1_GPIO_LINE, PAL_MODE_INPUT);
    palSetLineMode(ANT_NFAULT_2_GPIO_LINE, PAL_MODE_INPUT);
  }
  chSysUnlock();

  /* SEL is active low, so "clear" is on and "set" is off. PWR is active high.
   */
  palClearLine(ANT_PWR_SEL_1_GPIO_LINE);
  palClearLine(ANT_PWR_SEL_2_GPIO_LINE);
  palSetLine(ANT_IN_SEL_1_GPIO_LINE);
  palSetLine(ANT_IN_SEL_2_GPIO_LINE);

  settings_type_t antenna_mode_setting;
  settings_api_register_enum(antenna_mode_strings, &antenna_mode_setting);

  SETTING_NOTIFY("frontend",
                 "antenna_selection",
                 antenna_mode,
                 antenna_mode_setting,
                 antenna_config_notify);

  SETTING_NOTIFY("frontend",
                 "antenna_bias",
                 antenna_bias,
                 SETTINGS_TYPE_BOOL,
                 antenna_config_notify);
}

bool antenna_present(void) {
  switch (antenna_mode) {
    case ANTENNA_MODE_PRIMARY: {
      return antenna_bias && (palReadLine(ANT_PRESENT_1_GPIO_LINE) == PAL_HIGH);
    }

    case ANTENNA_MODE_SECONDARY: {
      return antenna_bias && (palReadLine(ANT_PRESENT_2_GPIO_LINE) == PAL_HIGH);
    }

    default: {
      assert(!"Invalid antenna mode setting");
      return false;
    }
  }

  return false;
}

bool antenna_shorted(void) {
  switch (antenna_mode) {
    case ANTENNA_MODE_PRIMARY: {
      return antenna_bias && (palReadLine(ANT_NFAULT_1_GPIO_LINE) == PAL_LOW);
    }

    case ANTENNA_MODE_SECONDARY: {
      return antenna_bias && (palReadLine(ANT_NFAULT_2_GPIO_LINE) == PAL_LOW);
    }

    default: {
      assert(!"Invalid antenna mode setting");
      return false;
    }
  }

  return false;
}
