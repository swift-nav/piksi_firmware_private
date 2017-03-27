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

#define NDB_WEAK

#include <libsbp/sbp.h>
#include <string.h>
#include <libswiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "settings.h"
#include "ndb_fs_access.h"
#include "sbp.h"
#include "sbp_utils.h"

/** Ionospheric corrections file name */
#define IONO_CORR_FILE_NAME "persistent/iono"
/** Ionospheric corrections file type */
#define IONO_CORR_FILE_TYPE "iono corrections"

static ionosphere_t iono_corr;
static ndb_element_metadata_t iono_corr_md;
static sbp_msg_callbacks_node_t iono_callback_node;
static void iono_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context);

static ndb_file_t iono_corr_file = {
  .name = IONO_CORR_FILE_NAME,
  .type = IONO_CORR_FILE_TYPE,
  .block_data = (u8*)&iono_corr,
  .block_md = &iono_corr_md,
  .block_size = sizeof(iono_corr),
  .block_count = 1
};

void ndb_iono_init(void)
{
  static bool erase_iono = false;
  SETTING("ndb", "erase_iono", erase_iono, TYPE_BOOL);

  ndb_load_data(&iono_corr_file, erase_iono);

  /* register Iono SBP callback */
  sbp_register_cbk(
    SBP_MSG_IONO,
    &iono_msg_callback,
    &iono_callback_node
  );
}

/**
 * Loads iono parameters from NDB
 *
 * \param[out] iono Destination container.
 *
 * \retval NDB_ERR_NONE             On success
 * \retval NDB_ERR_BAD_PARAM        On parameter error
 * \retval NDB_ERR_MISSING_IE       No cached data block
 * \retval NDB_ERR_AGED_DATA        Data in NDB has aged out
 * \retval NDB_ERR_MISSING_GPS_TIME GPS time is unknown
 *
 * \sa ndb_iono_corr_store
 */
ndb_op_code_t ndb_iono_corr_read(ionosphere_t *iono)
{
  ndb_op_code_t ret = ndb_retrieve(&iono_corr_md, iono, sizeof(*iono), NULL,
                                   NDB_USE_NV_IONO);
  if (NDB_ERR_NONE == ret) {
    /* If NDB read was successful, check that data has not aged out */
    ret = ndb_check_age(&iono->toa, NDB_NV_IONO_AGE);
  }
  return ret;
}

/**
 * Store iono parameters
 *
 * \param[in] sid         GNSS signal identifier for the source of iono data in
 *                        case of data source being NDB_DS_RECEIVER, NULL for
 *                        other cases.
 * \param[in] iono        Ionospheric parameters
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP. In other cases
 *                        set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE            On success. Iono data is updated.
 * \retval NDB_ERR_NO_CHANGE       On success. Iono data is unchanged.
 * \retval NDB_ERR_BAD_PARAM       Parameter errors.
 */
ndb_op_code_t ndb_iono_corr_store(const gnss_signal_t *sid,
                                  const ionosphere_t *iono,
                                  ndb_data_source_t src,
                                  u16 sender_id)
{
  ndb_op_code_t res = ndb_update(iono, src, &iono_corr_md);

  if (NULL != iono && NDB_ERR_NONE == res) {
    log_info(
        "Updating iono parameters: alpha = [%g %g %g %g], beta = [%g, %g, %g, %g]",
        iono->a0, iono->a1, iono->a2, iono->a3,
        iono->b0, iono->b1, iono->b2, iono->b3);
  }
  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_IONO,
                     res,
                     src,
                     NULL,
                     sid,
                     sender_id);

  return res;
}

static void iono_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)len; (void) context;

  log_info("Iono correction received from peer");

  ionosphere_t iono;
  memset(&iono, 0, sizeof(iono));

  /* unpack received message */
  iono.toa.tow = ((msg_iono_t*)msg)->t_nmct.tow;
  iono.toa.wn = ((msg_iono_t*)msg)->t_nmct.wn;
  iono.a0 = ((msg_iono_t*)msg)->a0;
  iono.a1 = ((msg_iono_t*)msg)->a1;
  iono.a2 = ((msg_iono_t*)msg)->a2;
  iono.a3 = ((msg_iono_t*)msg)->a3;
  iono.b0 = ((msg_iono_t*)msg)->b0;
  iono.b1 = ((msg_iono_t*)msg)->b1;
  iono.b2 = ((msg_iono_t*)msg)->b2;
  iono.b3 = ((msg_iono_t*)msg)->b3;

  /* store message in NDB */
  ndb_iono_corr_store(NULL, &iono, NDB_DS_SBP, sender_id);
}
