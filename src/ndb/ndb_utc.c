/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Niilo Sirola <niilo.sirola@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libsbp/sbp.h>
#include <string.h>
#include <swiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "utils/timing/timing.h"

#define UTC_PARAMS_FILE_NAME "persistent/ndb/utc_params"
#define UTC_PARAMS_FILE_TYPE "utc parameters"

static utc_params_t utc_params;
static ndb_element_metadata_t utc_params_md;

static ndb_file_t utc_params_file = {.name = UTC_PARAMS_FILE_NAME,
                                     .type = UTC_PARAMS_FILE_TYPE,
                                     .block_data = (u8 *)&utc_params,
                                     .block_md = &utc_params_md,
                                     .block_size = sizeof(utc_params),
                                     .block_count = 1};

void ndb_utc_params_init(void) {
  static bool erase_utc_params = false;
  SETTING("ndb", "erase_utc_params", erase_utc_params, TYPE_BOOL);

  ndb_load_data(&utc_params_file, erase_utc_params || !NDB_USE_NV_UTC);
}

/**
 * Read the UTC parameters information
 *
 * \param[out] utc_params_p  Pointer to UTC parameters structure
 * \param[out] is_nv         Pointer to NV flag, set to true if data was from NV
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 * \retval NDB_ERR_MISSING_IE No cached data block
 */
ndb_op_code_t ndb_utc_params_read(utc_params_t *utc_params_p, bool *is_nv) {
  if (is_nv != NULL) {
    *is_nv = (0 != (utc_params_md.vflags & NDB_VFLAG_DATA_FROM_NV));
  }

  return ndb_retrieve(
      &utc_params_md, utc_params_p, sizeof(*utc_params_p), NULL);
}

/**
 * Store UTC parameters information
 *
 * \param[in] sid           GNSS signal identifier for the source of UTC data in
 *                          case of data source being NDB_DS_RECEIVER, NULL for
 *                          other cases.
 * \param[in] utc_params_p  UTC parameters structure
 * \param[in] src           Data source
 * \param[in] sender_id     Sender ID if data source is NDB_DS_SBP. In other
 * cases
 *                          set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE            On success. UTC parameters updated.
 * \retval NDB_ERR_NO_CHANGE       UTC parameters unchanged.
 * \retval NDB_ERR_OLDER_DATA      More relevant parameters already in NDB.
 * \retval NDB_ERR_BAD_PARAM       Parameter errors.
 */
ndb_op_code_t ndb_utc_params_store(const gnss_signal_t *sid,
                                   const utc_params_t *utc_params_p,
                                   ndb_data_source_t src,
                                   u16 sender_id) {
  ndb_op_code_t res;
  utc_params_t current;
  bool is_nv;
  bool have_current_params =
      (NDB_ERR_NONE == ndb_utc_params_read(&current, &is_nv));

  if (is_nv) {
    /* always overwrite parameters loaded from NV */
    have_current_params = false;
  }

  gps_time_t now = get_current_time();

  if (have_current_params && gps_time_valid(&now) &&
      fabs(gpsdifftime(&current.tot, &now)) <
          fabs(gpsdifftime(&utc_params_p->tot, &now))) {
    /* keep the current parameters if their reference time is closer to current
     * time than that of the candidate parameters */
    res = NDB_ERR_OLDER_DATA;
  } else {
    res = ndb_update(utc_params_p, src, &utc_params_md);

    if (NULL != utc_params_p && NDB_ERR_NONE == res) {
      double offset = get_gps_utc_offset(&now, utc_params_p);
      log_info_sid(*sid,
                   "Updating UTC parameters: tow=%.0f, a0=%.2g, a1=%.2g, "
                   "a2=%.2g, t_lse=(%d,%.1f), ls=%d, lsf=%d. Current offset: "
                   "%.0f s + %.3g ns",
                   utc_params_p->tot.tow,
                   utc_params_p->a0,
                   utc_params_p->a1,
                   utc_params_p->a2,
                   utc_params_p->t_lse.wn,
                   utc_params_p->t_lse.tow,
                   utc_params_p->dt_ls,
                   utc_params_p->dt_lsf,
                   round(offset),
                   SECS_NS * (offset - round(offset)));
    }
  }

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_UTC_PARAMS,
                     res,
                     src,
                     NULL,
                     sid,
                     sender_id);

  return res;
}

/* TODO: add reading from SBP message */
