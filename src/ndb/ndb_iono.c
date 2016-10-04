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

#define NDB_WEAK

#include <string.h>

#include <libswiftnav/logging.h>
#include <libsbp/observation.h>
#include <libsbp/sbp.h>

#include "ndb.h"
#include "ndb_internal.h"
#include "sbp.h"
#include "sbp_utils.h"

#define IONO_CORR_FILE_NAME "iono"
static ionosphere_t iono_corr _CCM;
static ndb_element_metadata_t iono_corr_md _CCM;

static ndb_file_t iono_corr_file = {
  .name = IONO_CORR_FILE_NAME,
  .fh = -1,
  .expected_size =
        sizeof(iono_corr)
      + sizeof(ndb_element_metadata_nv_t)
      + sizeof(ndb_file_end_mark),
  .data_size = sizeof(iono_corr),
  .n_elements = 1
};

void ndb_iono_init()
{
  ndb_load_data(&iono_corr_file, "iono corrections",
                &iono_corr, &iono_corr_md,
                 sizeof(iono_corr), 1);
}

enum ndb_op_code ndb_iono_corr_read(ionosphere_t *iono)
{
  if(!NDB_IE_IS_VALID(&iono_corr_md)) {
    return NDB_ERR_MISSING_IE;
  }
  ndb_retrieve(iono, &iono_corr, sizeof(iono_corr));
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_iono_corr_store(ionosphere_t *iono,
                                     enum ndb_data_source src)
{
  return ndb_update(iono, src, &iono_corr_md);
}

static void ndb_iono_msg_callback(u16 sender_id,
                                  u8 len,
                                  u8 msg[],
                                  void* context)
{
  (void)sender_id; (void)context;

  if (len != sizeof(msg_iono_t)) {
    log_warn("Received bad iono from peer");
    return;
  }

  ionosphere_t iono;
  unpack_iono((msg_iono_t *)msg, &iono);

  ndb_iono_corr_store(&iono, NDB_DS_SBP);
}

void ndb_sbp_iono_reg_cbk()
{
  static sbp_msg_callbacks_node_t cbk_node;
  sbp_register_cbk(SBP_MSG_IONO, &ndb_iono_msg_callback, &cbk_node);
}
