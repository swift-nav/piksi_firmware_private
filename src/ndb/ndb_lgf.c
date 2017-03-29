/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
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
#include <libswiftnav/constants.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "settings.h"
#include "ndb_fs_access.h"
#include "sbp_utils.h"

/** Default NDB LGF interval update threshold [s] */
#define NDB_LGF_UPDATE_INTERVAL_S (30 * MINUTE_SECS)
/** Default NDB LGF distance update threshold [m] */
#define NDB_LGF_UPDATE_DISTANCE_M (10e3)

#define LGF_FILE_NAME "persistent/lgf"
#define LGF_FILE_TYPE "LGF"

static last_good_fix_t last_good_fix;       /**< Locally cached LGF */

/** NDB LGF interval update threshold [s] */
static s32             lgf_update_s = NDB_LGF_UPDATE_INTERVAL_S;
/** NDB LGF distance update threshold [m] */
static s32             lgf_update_m = NDB_LGF_UPDATE_DISTANCE_M;

static last_good_fix_t        last_good_fix_saved; /**< NDB LGF data block */
static ndb_element_metadata_t last_good_fix_md;    /**< NDB LGF metadata */

static ndb_file_t lgf_file = {
  .name = LGF_FILE_NAME,
  .type = LGF_FILE_TYPE,
  .block_data = (u8*)&last_good_fix_saved,
  .block_md = &last_good_fix_md,
  .block_size = sizeof(last_good_fix_saved),
  .block_count = 1
};

/**
 * Initializes the data source to NV for
 * non-volatile last good fix entry.
 * Applied at receiver start up.
 *
 */
void ndb_lgf_init_ds(void)
{
  last_good_fix_t lgf;
  ndb_op_code_t res = ndb_lgf_read(&lgf);
  if (NDB_ERR_NONE == res) {
    ndb_update_init_ds(&lgf, NDB_DS_NV, &last_good_fix_md);
  }
}

void ndb_lgf_init(void)
{
  static bool erase_lgf = true;
  SETTING("ndb", "erase_lgf", erase_lgf, TYPE_BOOL);
  SETTING("ndb", "lgf_update_s", lgf_update_s, TYPE_INT);
  SETTING("ndb", "lgf_update_m", lgf_update_m, TYPE_INT);

  ndb_load_data(&lgf_file, erase_lgf);

  ndb_lgf_init_ds();

  last_good_fix = last_good_fix_saved;
  if (0 != (last_good_fix_md.nv_data.state & NDB_IE_VALID)) {
    /* TODO check loaded LGF validity */
    log_info("Position loaded [%.4lf, %.4lf, %.1lf]",
             last_good_fix.position_solution.pos_llh[0] * R2D,
             last_good_fix.position_solution.pos_llh[1] * R2D,
             last_good_fix.position_solution.pos_llh[2]);
  } else {
    log_info("Position is not available");
  }
}

/**
 * Loads last good fix data from NDB
 *
 * NDB loads data that has been stored in NV memory.
 *
 * \param[out] lgf Destination container.
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 * \retval NDB_ERR_MISSING_IE No cached data block
 *
 * \sa ndb_lgf_store
 */
ndb_op_code_t ndb_lgf_read(last_good_fix_t *lgf)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  /* LGF is loaded only on boot, and then periodically saved to NV. Because of
   * this, use of `ndb_retrieve` here is unnecessary. */

  if (NULL != lgf) {
    ndb_lock();
    if (0 != (last_good_fix_md.nv_data.state & NDB_IE_VALID)) {
      *lgf = last_good_fix;
      res = NDB_ERR_NONE;
    } else {
      memset(lgf, 0, sizeof(*lgf));
      res = NDB_ERR_MISSING_IE;
    }
    ndb_unlock();
  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}

/**
 * Updates last good fix information.
 *
 * This method locally caches new information for later retrieval and also
 * schedules it for persistence only when previous fix is too old or too far.
 *
 * \param[in] lgf Position and clock parameters to update.
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_NO_CHANGE  New lgf is within thresholds
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 *
 * \sa ndb_lgf_read
 */
ndb_op_code_t ndb_lgf_store(const last_good_fix_t *lgf)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != lgf) {
    bool update_nv_data = true;

    ndb_lock();

    last_good_fix = *lgf;
    last_good_fix.position_quality = POSITION_FIX;

    if (0 != (last_good_fix_md.nv_data.state & NDB_IE_VALID)) {
      double dist_ecef[3] = {0}; /* Fix distance [ECEF] */
      double dt = 0;             /* Fix time difference [s] */
      double dx = 0;             /* Fix distance [m] */

      /* Compute difference in time and space between
       * last saved position and the passed one */
      vector_subtract(3, last_good_fix.position_solution.pos_ecef,
                      last_good_fix_saved.position_solution.pos_ecef, dist_ecef);
      dx = vector_norm(3, dist_ecef);
      dt = gpsdifftime(&last_good_fix.position_solution.time,
                       &last_good_fix_saved.position_solution.time);

      if (dt >= 0 && dt < lgf_update_s && dx <= lgf_update_m) {
        /* Do not update, if the time interval is too small and distance is too
         * short */
        update_nv_data = false;
      }
    }

    ndb_unlock();

    if (update_nv_data) {
      /* Last saved is either too old or too far or both - update it */
      log_info("Position saved [%.4lf, %.4lf, %.1lf]",
               lgf->position_solution.pos_llh[0] * R2D,
               lgf->position_solution.pos_llh[1] * R2D,
               lgf->position_solution.pos_llh[2]);
      res = ndb_update(lgf, NDB_DS_RECEIVER, &last_good_fix_md);
    } else {
      res = NDB_ERR_NO_CHANGE;
    }
  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  /* Save bandwidth and don't send the default result */
  if (res != NDB_ERR_NO_CHANGE) {
    sbp_send_ndb_event(NDB_EVENT_STORE,
                       NDB_EVENT_OTYPE_LGF,
                       res,
                       NDB_DS_RECEIVER,
                       NULL,
                       NULL,
                       NDB_EVENT_SENDER_ID_VOID);
  }

  return res;
}
