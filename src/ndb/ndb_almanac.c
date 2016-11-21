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
#include <assert.h>
#include "ndb.h"
#include "ndb_internal.h"
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <timing.h>
#include <signal.h>
#include <sbp.h>
#include <sbp_utils.h>
#include "settings.h"
#include "ndb_fs_access.h"

/** Almanac file name */
#define NDB_ALMA_FILE_NAME   "persistent/almanac"
/** Almanac file type */
#define NDB_ALMA_FILE_TYPE   "almanac"

static almanac_t ndb_almanac[PLATFORM_SIGNAL_COUNT];
static ndb_element_metadata_t ndb_almanac_md[PLATFORM_SIGNAL_COUNT];

static ndb_file_t ndb_alma_file = {
    .name = NDB_ALMA_FILE_NAME,
    .type = NDB_ALMA_FILE_TYPE,
    .block_data = (u8*)&ndb_almanac[0],
    .block_md = &ndb_almanac_md[0],
    .block_size = sizeof(ndb_almanac[0]),
    .block_count = sizeof(ndb_almanac) / sizeof(ndb_almanac[0]),
};

void ndb_almanac_init(void)
{
  static bool erase_almanac = true;
  SETTING("ndb", "erase_almanac", erase_almanac, TYPE_BOOL);

  ndb_load_data(&ndb_alma_file, erase_almanac);
}

ndb_op_code_t ndb_almanac_read(gnss_signal_t sid, almanac_t *a)
{
  u16 idx = sid_to_global_index(sid);
  return ndb_retrieve(&ndb_almanac_md[idx], a, sizeof(*a), NULL, NULL);
}

ndb_op_code_t ndb_almanac_store(const almanac_t *a, ndb_data_source_t src)
{
  u16 idx = sid_to_global_index(a->sid);
  return ndb_update(a, src, &ndb_almanac_md[idx]);
}

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_almanac_sbp_update(void)
{
}
