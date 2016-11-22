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

#include <string.h>

void antenna_init(void)
{
  palSetLineMode(ANT_PWR_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_PWR_SEL_1_GPIO_LINE);

  palSetLineMode(ANT_PWR_SEL_2_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_PWR_SEL_2_GPIO_LINE);

  palSetLineMode(ANT_IN_SEL_0_GPIO_LINE, PAL_MODE_OUTPUT);
  palSetLine(ANT_IN_SEL_0_GPIO_LINE);

  palSetLineMode(ANT_IN_SEL_1_GPIO_LINE, PAL_MODE_OUTPUT);
  palClearLine(ANT_IN_SEL_1_GPIO_LINE);

  palSetLineMode(ANT_PRESENT_1_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(ANT_PRESENT_2_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(ANT_NFAULT_1_GPIO_LINE, PAL_MODE_INPUT);
  palSetLineMode(ANT_NFAULT_2_GPIO_LINE, PAL_MODE_INPUT);
}

bool antenna_present(void)
{
  return (palReadLine(ANT_PRESENT_2_GPIO_LINE) == PAL_HIGH);
}
