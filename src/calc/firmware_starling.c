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

#include <calc/starling_input_bridge.h>
#include <calc/starling_integration.h>
#include <ch.h>
#include <sbp/sbp_fileio.h>
#include <starling/starling.h>
#include <utils/settings/settings.h>

/* Persistent file indicating whether Starling should run on the firmware
 * or on Linux. */
#define STARLING_ON_LINUX_FILENAME "/persistent/flags/starling_on_linux"


/**
 * Read from the persistent file to find out if Starling is enabled for Linux
 * or for firmware. If the file exists and contains the character '1', then
 * Starling is running on Linux. In all other cases, we will default to running
 * on the firmware.
 */
static bool is_firmware_starling_enabled(void) {
  unsigned char value = '0';
  ssize_t n =
      sbp_fileio_read(STARLING_ON_LINUX_FILENAME, 0, &value, sizeof(value));
  if (n == sizeof(value) && value == '1') {
    return false;
  }
  return true;
}

void firmware_starling_setup(void) {
  starling_input_bridge_init();
  starling_calc_pvt_setup();

  if (!is_firmware_starling_enabled()) {
    /* Inform the bridge that it need not waste time buffering inputs. */
    starling_input_bridge_set_mode(STARLING_BRIDGE_MODE_BYPASS);
    log_info("Firmware Starling off.");
  } else {
    log_info("Firmware Starling on.");
  }
}
