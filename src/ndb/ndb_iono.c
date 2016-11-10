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
#include "settings.h"
#include "ndb_fs_access.h"

/** Ionospheric corrections file name */
#define IONO_CORR_FILE_NAME "persistent/iono"
/** Ionospheric corrections file type */
#define IONO_CORR_FILE_TYPE "iono corrections"

static ionosphere_t iono_corr;
static ndb_element_metadata_t iono_corr_md;

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
  static bool erase_iono = true;
  SETTING("ndb", "erase_iono", erase_iono, TYPE_BOOL);
  if (erase_iono) {
    ndb_fs_remove(IONO_CORR_FILE_NAME);
  }

  ndb_load_data(&iono_corr_file);

  if (0 != (iono_corr_md.nv_data.state & NDB_IE_VALID)) {
    if (erase_iono) {
      log_error("NDB iono erase is not working");
    }

    log_info("Loaded iono corrections");
  }
}

ndb_op_code_t ndb_iono_corr_read(ionosphere_t *iono)
{
  return ndb_retrieve(&iono_corr_md, iono, sizeof(*iono), NULL, NULL);
}

ndb_op_code_t ndb_iono_corr_store(const ionosphere_t *iono, ndb_data_source_t src)
{
  return ndb_update(iono, src, &iono_corr_md);
}
