/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "position.h"

#include <math.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "ndb/ndb.h"
#include "timing/timing.h"

/** Get last saved position from NDB.
 */
void position_setup(void) {
  last_good_fix_t lgf;
  ndb_op_code_t res = ndb_lgf_read(&lgf);
  if ((NDB_ERR_NONE == res || NDB_ERR_GPS_TIME_MISSING == res) &&
      lgf.position_solution.valid) {
    log_info("Using last good fix from file: %.4f %.4f %.1f",
             lgf.position_solution.pos_llh[0] * (180 / M_PI),
             lgf.position_solution.pos_llh[1] * (180 / M_PI),
             lgf.position_solution.pos_llh[2]);
  }
}
