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
#include "ndb.h"
#include "ndb_internal.h"

#define IONO_CORR_FILE_NAME "persistent/iono"
static ionosphere_t iono_corr _CCM;
static ndb_element_metadata_t iono_corr_md _CCM;

static ndb_file_t iono_corr_file = {
  .name = IONO_CORR_FILE_NAME,
  .expected_size =
        sizeof(iono_corr)
      + sizeof(ndb_element_metadata_nv_t)
      + sizeof(ndb_file_end_mark),
  .data_size = sizeof(iono_corr),
  .n_elements = 1
};

void ndb_iono_init(void)
{
  ndb_load_data(&iono_corr_file, "iono corrections",
                (u8 *)&iono_corr, &iono_corr_md,
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
