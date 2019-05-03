/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "can_enabled.h"

#include <hal.h>
#include <stdio.h>
#include <stdlib.h>

#include "sbp/sbp_fileio.h"

#define CAN_ENABLED_FILENAME "/etc/flags/can_ports"
#define INS_ACTIVE_FILENAME "/etc/flags/ins_active"

/* If /etc/flags/can_ports exists and is 1, it means can ports are enabled. */

bool can_enabled(void) {
  unsigned char value = '0';
  ssize_t n =
      sbp_fileio_read(CAN_ENABLED_FILENAME, 0, &value, sizeof(value));
  if (n == sizeof(value) && value == '1') {
    return true;
  }
  return false;
}

