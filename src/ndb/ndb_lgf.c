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

#include <math.h>
#include <string.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/constants.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "settings.h"
#include "ndb_fs_access.h"

/** Default NDB LGF interval update threshold [s] */
#define NDB_LGF_UPDATE_INTERVAL_S (30 * MINUTE_SECS)
/** Default NDB LGF distance update threshold [m] */
#define NDB_LGF_UPDATE_DISTANCE_M (10e3)

#define LGF_FILE_NAME "persistent/lgf"
static last_good_fix_t last_good_fix;       /**< Locally cached LGF */

/** NDB LGF interval update threshold [s] */
static s32             lgf_update_s = NDB_LGF_UPDATE_INTERVAL_S;
/** NDB LGF distance update threshold [m] */
static s32             lgf_update_m = NDB_LGF_UPDATE_DISTANCE_M;

static last_good_fix_t        last_good_fix_saved; /**< NDB LGF data block */
static ndb_element_metadata_t last_good_fix_md;    /**< NDB LGF metadata */

static ndb_file_t lgf_file = {
  .name = LGF_FILE_NAME,
  .expected_size =
        sizeof(last_good_fix_t) * 1
      + sizeof(ndb_element_metadata_nv_t) * 1
      + sizeof(ndb_file_end_mark),
  .data_size = sizeof(last_good_fix_t),
  .n_elements = 1
};

static bool ndb_lgf_validate(const last_good_fix_t *lgf) {
  /* Check quality */
  if (!((lgf->position_quality == POSITION_UNKNOWN) ||
        (lgf->position_quality == POSITION_GUESS) ||
        (lgf->position_quality == POSITION_STATIC) ||
        (lgf->position_quality == POSITION_FIX))) {
    return false;
  }

  /* Check position valid flag */
  if (lgf->position_solution.valid != 1) {
    return false;
  }

  /* Check that values are not NaN */
  if (!isfinite(lgf->position_solution.pos_llh[0]) ||
      !isfinite(lgf->position_solution.pos_llh[1]) ||
      !isfinite(lgf->position_solution.pos_llh[2])) {
    return false;
  }
  if (!isfinite(lgf->position_solution.pos_ecef[0]) ||
      !isfinite(lgf->position_solution.pos_ecef[1]) ||
      !isfinite(lgf->position_solution.pos_ecef[2])) {
    return false;
  }

  /* Check that LLH is in valid range */
  if ((lgf->position_solution.pos_llh[0] > 90.0 * D2R) ||
      (lgf->position_solution.pos_llh[0] < -90.0 * D2R)) {
    return false;
  }
  if ((lgf->position_solution.pos_llh[1] > 180.0 * D2R) ||
      (lgf->position_solution.pos_llh[1] < -180.0 * D2R)) {
    return false;
  }
  /* From limits in calc_PVT */
  if ((lgf->position_solution.pos_llh[2] > 1e6) ||
      (lgf->position_solution.pos_llh[2] < -1e3)) {
    return false;
  }

  /* Check that ECEF and LLH is consistent within 1 mm */
  double check_pos_ecef[3];
  wgsllh2ecef(lgf->position_solution.pos_llh, check_pos_ecef);
  if ((abs(lgf->position_solution.pos_ecef[0] - check_pos_ecef[0]) > 0.001) ||
      (abs(lgf->position_solution.pos_ecef[1] - check_pos_ecef[1]) > 0.001) ||
      (abs(lgf->position_solution.pos_ecef[2] - check_pos_ecef[2]) > 0.001)) {
    return false;
  }

  /* Check velocity */
  if (lgf->position_solution.velocity_valid) {
    /* Check that values are not NaN */
    if (!isfinite(lgf->position_solution.vel_ned[0]) ||
        !isfinite(lgf->position_solution.vel_ned[1]) ||
        !isfinite(lgf->position_solution.vel_ned[2])) {
      return false;
    }
    if (!isfinite(lgf->position_solution.vel_ecef[0]) ||
        !isfinite(lgf->position_solution.vel_ecef[1]) ||
        !isfinite(lgf->position_solution.vel_ecef[2])) {
      return false;
    }

    /* From limits in calc_PVT */
    if (vector_norm(3, lgf->position_solution.vel_ned) >= 0.514444444 * 1000) {
      return false;
    }

    /* Check that ECEF and NED is consistent within 1 mm/s*/
    double check_vel_ecef[3];
    wgsned2ecef(lgf->position_solution.vel_ned,
                lgf->position_solution.pos_ecef, check_vel_ecef);
    if ((abs(lgf->position_solution.vel_ecef[0] - check_vel_ecef[0]) > 0.001) ||
        (abs(lgf->position_solution.vel_ecef[1] - check_vel_ecef[1]) > 0.001) ||
        (abs(lgf->position_solution.vel_ecef[2] - check_vel_ecef[2]) > 0.001)) {
      return false;
    }
  }

  /* Check the error covariance */
  if (!isfinite(lgf->position_solution.err_cov[0]) ||
      !isfinite(lgf->position_solution.err_cov[1]) ||
      !isfinite(lgf->position_solution.err_cov[2]) ||
      !isfinite(lgf->position_solution.err_cov[3]) ||
      !isfinite(lgf->position_solution.err_cov[4]) ||
      !isfinite(lgf->position_solution.err_cov[5]) ||
      !isfinite(lgf->position_solution.err_cov[6])) {
    return false;
  }

  /* Check the clock parameters */
  /* TODO(Leith): Not sure what reasonable ranges of values to check agaisnt */
  if (!isfinite(lgf->position_solution.clock_offset) ||
      !isfinite(lgf->position_solution.clock_bias)) {
    return false;
  }

  /* Check time */
  /* TODO(Leith): do we want LGF to expire after a certain amount of time? */
  if (!isfinite(lgf->position_solution.time.tow) ||
      !gps_current_time_valid(&lgf->position_solution.time)) {
    return false;
  }

  /* Check for valid number of sats */
  if (lgf->position_solution.n_used < 4) {
    return false;
  }

  return true;
}

void ndb_lgf_init(void)
{
  static bool erase_lgf = true;
  SETTING("ndb", "erase_lgf", erase_lgf, TYPE_BOOL);
  SETTING("ndb", "lgf_update_s", lgf_update_s, TYPE_INT);
  SETTING("ndb", "lgf_update_m", lgf_update_m, TYPE_INT);

  if (erase_lgf) {
    ndb_fs_remove(LGF_FILE_NAME);
  }

  ndb_load_data(&lgf_file, "LGF",
                (u8 *)&last_good_fix_saved, &last_good_fix_md,
                sizeof(last_good_fix_saved), 1);

  last_good_fix = last_good_fix_saved;
  if (0 != (last_good_fix_md.nv_data.state & NDB_IE_VALID)) {
    if (erase_lgf) {
      /* Log the error if the data is present after erase */
      log_error("NDB LGF erase is not working");
    }

    if (!ndb_lgf_validate(&last_good_fix)) {
      log_error("NDB: Invalid LGF data retreived. Erasing.");
      ndb_erase(&last_good_fix_md);
      memset(&last_good_fix, 0, sizeof(last_good_fix));
    } else {
      log_info("Position loaded [%.4lf, %.4lf, %.1lf]",
               last_good_fix.position_solution.pos_llh[0] * R2D,
               last_good_fix.position_solution.pos_llh[1] * R2D,
               last_good_fix.position_solution.pos_llh[2]);
    }
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
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 *
 * \sa ndb_lgf_read
 */
ndb_op_code_t ndb_lgf_store(const last_good_fix_t *lgf)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != lgf && ndb_lgf_validate(lgf)) {
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
      res = NDB_ERR_NONE;
    }
  } else {
    log_warn("NDB: Invalid LGF was attempted to be stored.");
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}
