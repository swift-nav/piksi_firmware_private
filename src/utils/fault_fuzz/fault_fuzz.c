/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: dev <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>
#include <inttypes.h>
#include <string.h>

#include <libsbp/system.h>
#include <libsbp/version.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>

#include "board.h"
#include "board/nap/nap_common.h"
#include "board/v3/remoteproc/rpmsg.h"
#include "chdebug.h"
#include "main.h"
#include "manage.h"
#include "piksi_systime.h"
#include "sbp.h"
#include "settings/settings_client.h"

static bool do_assert = false;
static bool do_chassert = false;

/* Settings callback to trigger various fault routines */
static int fault_fuzz_handler(void *ctx) {
  (void)ctx;

  /* if multiple are true we aren't doing this right... */
  if (do_assert) {
    assert(false);
  } else if (do_chassert) {
    chDbgCheck(false);
  } else {
    return SETTINGS_WR_VALUE_REJECTED;
  }

  return SETTINGS_WR_OK;
}

void fault_fuzz_setup(void) {
  SETTING_NOTIFY("fault_fuzz",
                 "assert",
                 do_assert,
                 SETTINGS_TYPE_BOOL,
                 fault_fuzz_handler);
  SETTING_NOTIFY("fault_fuzz",
                 "chassert",
                 do_chassert,
                 SETTINGS_TYPE_BOOL,
                 fault_fuzz_handler);
}

