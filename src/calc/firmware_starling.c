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

#include "starling_input_bridge.h"
#include "starling_integration.h"

#include <starling/starling.h>

void firmware_starling_init(void) {
  starling_input_bridge_init();
  starling_initialize_api();
}

void firmware_starling_run(void) {
  starling_calc_pvt_setup();
}




