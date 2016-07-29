/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#define DEBUG 0
#define NDB_WEAK

#include <string.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"

#define LGF_FILE_NAME "lgf"
static last_good_fix_t last_good_fix _CCM;
static last_good_fix_t last_good_fix_saved _CCM;
static ndb_element_metadata_t last_good_fix_md _CCM;

static ndb_file_t lgf_file = {
  .name = LGF_FILE_NAME,
  .fh = -1,
  .expected_size =
        sizeof(last_good_fix)
      + sizeof(ndb_element_metadata_nv_t)
      + sizeof(ndb_file_end_mark),
  .data_size = sizeof(last_good_fix),
  .n_elements = 1
};

void ndb_lgf_init()
{
  ndb_load_data(&lgf_file, "LGF",
                &last_good_fix_saved, &last_good_fix_md,
                 sizeof(last_good_fix), 1);
  memcpy(&last_good_fix, &last_good_fix_saved, sizeof(last_good_fix_t));
}

enum ndb_op_code ndb_lgf_read(last_good_fix_t *lgf)
{
  if(!NDB_IE_IS_VALID(&last_good_fix_md)) {
    memset(lgf, 0, sizeof(last_good_fix_t));
    return NDB_ERR_MISSING_IE;
  }
  ndb_retrieve(lgf, &last_good_fix, sizeof(last_good_fix));
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_lgf_store(last_good_fix_t *lgf)
{
  double temp[3];
  double dt;

  ndb_lock();

  /* Update cached LGF unconditionally */
  memcpy(&last_good_fix.position_solution,
         &lgf->position_solution,
         sizeof(last_good_fix_t));
  last_good_fix.position_quality = POSITION_FIX;

  /* Compute difference in time and space between
   * last saved position and the passed one */
  vector_subtract(3, last_good_fix.position_solution.pos_ecef,
                  last_good_fix_saved.position_solution.pos_ecef, temp);
  dt = gpsdifftime(&last_good_fix.position_solution.time,
                   &last_good_fix_saved.position_solution.time);

  ndb_unlock();

  double dx = vector_norm(3, temp);

  if (dt > 30 * 60 || dx > 10e3) {
    /* Last saved is either too old or too far or both - update it */
    log_info("Position saved [%.4f, %.4f, %.1f]",
             last_good_fix.position_solution.pos_llh[0] * (180 / M_PI),
             last_good_fix.position_solution.pos_llh[1] * (180 / M_PI),
             last_good_fix.position_solution.pos_llh[2]);
    return ndb_update(&last_good_fix, NDB_DS_RECEIVER, &last_good_fix_md);
  }

  return NDB_ERR_NONE;
}
