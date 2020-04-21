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

#include <calc/starling_integration.h>
#include <ch.h>
#include <sbp/sbp_fileio.h>
#include <stdbool.h>
#include <utils/settings/settings_client.h>
#include <utils/system_monitor/system_monitor.h>

/* Persistent file indicating whether Starling should run on the firmware
 * or on Linux. */
#define STARLING_ON_LINUX_FILENAME "/etc/flags/starling_on_linux"

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

void firmware_starling_preinit(void) { pal_init(); }

void firmware_starling_setup(void) {
  starling_calc_pvt_setup();

  if (!is_firmware_starling_enabled()) {
    /* Inform the bridge that it need not waste time buffering inputs. */
    pvt_driver_set_input_bridge_mode(pvt_driver, PVT_DRIVER_BRIDGE_MODE_BYPASS);
    watchdog_thread_ignore(WD_NOTIFY_STARLING);
    log_debug("Firmware Starling off.");
  } else {
    log_debug("Firmware Starling on.");
  }
}
