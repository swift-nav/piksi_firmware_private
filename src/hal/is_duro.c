/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Dennis Zollo <dzollo@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>
#include <stdio.h>
#include <stdlib.h>
#include "sbp_fileio.h"

#define DEVICE_IS_DURO_FILENAME "/etc/flags/is_duro"

/* If /etc/flags/is_duro exists and is 1, it means we have a Duro.
 * see package/common_init/overlay/etc/init.d/copy_duro_eeprom.sh
 * in buildroot for more info.
 * This has a dependence on boot order (i.e eeprom has to have been
 * read by linux already) but currently a race should be impossible.
 */

bool device_is_duro(void) {
  unsigned char value = '0';
  ssize_t n =
      sbp_fileio_read(DEVICE_IS_DURO_FILENAME, 0, &value, sizeof(value));
  if (n == sizeof(value) && value == '1') {
    return true;
  }
  return false;
}
