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

#include <math.h>
#include <string.h>

#include <libswiftnav/logging.h>

#include "position.h"
#include "timing.h"
#include "ndb.h"

/** Get last saved position from NDB.
 */
void position_setup(void)
{
  last_good_fix_t lgf;
  if(ndb_lgf_read(&lgf) == NDB_ERR_NONE && lgf.position_solution.valid) {
    log_info("Loaded last position solution from file: %.4f %.4f %.1f",
             lgf.position_solution.pos_llh[0] * (180 / M_PI),
             lgf.position_solution.pos_llh[1] * (180 / M_PI),
             lgf.position_solution.pos_llh[2]);
    /* Set position quality to POSITION_GUESS as it's unknown when it was
     * stored. Position quality will be changed on next position update. */
    lgf.position_quality = POSITION_GUESS;
    ndb_lgf_store(&lgf);
    set_time(TIME_GUESS, lgf.position_solution.time);
  } else {
    log_info("No LGF information available (%d, %d)",
             lgf.position_solution.valid,
             lgf.position_quality);
  }
}
