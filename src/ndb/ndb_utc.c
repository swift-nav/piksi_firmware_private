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

#define NDB_WEAK

#include <libsbp/sbp.h>
#include <string.h>
#include <libswiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "settings.h"
#include "sbp.h"
#include "sbp_utils.h"

#define UTC_PARAMS_FILE_NAME "persistent/utc_params"
#define UTC_PARAMS_FILE_TYPE "utc parameters"

static utc_params_t utc_params;
static ndb_element_metadata_t utc_params_md;

static ndb_file_t utc_params_file = {
  .name = UTC_PARAMS_FILE_NAME,
  .type = UTC_PARAMS_FILE_TYPE,
  .block_data = (u8*)&utc_params,
  .block_md = &utc_params_md,
  .block_size = sizeof(utc_params),
  .block_count = 1
};

void ndb_utc_params_init(void)
{

  static bool erase_utc_params = true;
  SETTING("ndb", "erase_utc_params", erase_utc_params, TYPE_BOOL);

  ndb_load_data(&utc_params_file, erase_utc_params);

}

ndb_op_code_t ndb_utc_params_read(utc_params_t *utc_params_p)
{
  return ndb_retrieve(&utc_params_md, utc_params_p, sizeof(*utc_params_p),
                      NULL, NULL);
}

/**
 * Store UTC parameters information
 *
 * \param[in] sid         GNSS signal identifier for the source of utc data in
 *                        case of data source being NDB_DS_RECEIVER, NULL for
 *                        other cases.
 * \param[in] utc_params  UTC parameters structure
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP. In other cases
 *                        set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE            On success. UTC parameters updated.
 * \retval NDB_ERR_NO_CHANGE       UTC parameters unchanged.
 * \retval NDB_ERR_OLDER_DATA      Parameters older than ones already in NDB.
 * \retval NDB_ERR_BAD_PARAM       Parameter errors.
 */
ndb_op_code_t ndb_utc_params_store(const gnss_signal_t *sid,
                                   const utc_params_t *utc_params_p,
                                   ndb_data_source_t src,
                                   u16 sender_id)
{

  ndb_op_code_t res;
  utc_params_t current;

  if (NDB_ERR_NONE == ndb_utc_params_read(&current)
      && gpsdifftime(&utc_params_p->tot, &current.tot) <= 0) {

    res = NDB_ERR_OLDER_DATA;

  } else {

    res = ndb_update(utc_params_p, src, &utc_params_md);

    if (NULL != utc_params_p && NDB_ERR_NONE == res) {
      log_info_sid(*sid,
          "Updating UTC parameters: a0=%.1g, a1=%.1g, t_lse=(%d,%.1f), ls=%d, lsf=%d",
          utc_params_p->a0, utc_params_p->a1,
          utc_params_p->t_lse.wn, utc_params_p->t_lse.tow,
          utc_params_p->dt_ls, utc_params_p->dt_lsf);
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
