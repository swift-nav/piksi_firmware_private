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
#include <libswiftnav/logging.h>
#include "settings.h"

#include <string.h>
#include <assert.h>

typedef enum {
  ANTENNA_MODE_PRIMARY,
  ANTENNA_MODE_SECONDARY
} antenna_mode_t;

static const char * const antenna_mode_strings[] = {
  "Primary",
  "Secondary",
  NULL
};

static antenna_mode_t antenna_mode = ANTENNA_MODE_PRIMARY;

static void antenna_mode_configure(antenna_mode_t mode)
{
  switch (mode) {
  case ANTENNA_MODE_PRIMARY: {
    palClearLine(ANT_IN_SEL_0_GPIO_LINE);
    palSetLine(ANT_IN_SEL_1_GPIO_LINE);
  }
  break;

  case ANTENNA_MODE_SECONDARY: {
    palClearLine(ANT_IN_SEL_1_GPIO_LINE);
    palSetLine(ANT_IN_SEL_0_GPIO_LINE);
  }
  break;

  default: {
    assert(!"Invalid antenna mode");
  }
  break;
  }
}

static bool antenna_mode_notify(struct setting *s, const char *val)
{
  if (!s->type->from_string(s->type->priv, s->addr, s->len, val))
  {
    return false;
  }

  antenna_mode_configure(antenna_mode);
  return true;
}

void antenna_init(void)
{
  palSetLineMode(ANT_PWR_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_PWR_SEL_1_GPIO_LINE);

  palSetLineMode(ANT_PWR_SEL_2_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_PWR_SEL_2_GPIO_LINE);

  palSetLineMode(ANT_IN_SEL_0_GPIO_LINE, PAL_MODE_OUTPUT);
  palClearLine(ANT_IN_SEL_0_GPIO_LINE);

  palSetLineMode(ANT_IN_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
  palClearLine(ANT_IN_SEL_1_GPIO_LINE);

  palSetLineMode(ANT_PRESENT_1_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(ANT_PRESENT_2_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(ANT_NFAULT_1_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(ANT_NFAULT_2_GPIO_LINE, PAL_MODE_INPUT);

  static struct setting_type antenna_mode_setting;
  int TYPE_ANTENNA_MODE = settings_type_register_enum(antenna_mode_strings,
                                                      &antenna_mode_setting);
  SETTING_NOTIFY("frontend", "antenna_selection",
                 antenna_mode, TYPE_ANTENNA_MODE, antenna_mode_notify);

  antenna_mode_configure(antenna_mode);
}

bool antenna_present(void)
{
  switch (antenna_mode) {
  case ANTENNA_MODE_PRIMARY: {
    return (palReadLine(ANT_PRESENT_1_GPIO_LINE) == PAL_HIGH);
  }

  case ANTENNA_MODE_SECONDARY: {
    return (palReadLine(ANT_PRESENT_2_GPIO_LINE) == PAL_HIGH);
  }

  default: {
    assert(!"Invalid antenna mode setting");
    return false;
  }
  }

  return false;
}
