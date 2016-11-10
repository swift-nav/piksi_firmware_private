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

#include <libsbp/sbp.h>
#include <string.h>
#include <libswiftnav/logging.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "sbp.h"

/** L2C capabilities file name */
#define GPS_L2C_CAPB_FILE_NAME "persistent/l2c_capb"
/** L2C capabilities file type */
#define GPS_L2C_CAPB_FILE_TYPE "l2c capabilities"
/** L2C capability default value */
#define GPS_L2C_CAPAB_DEFAULT 0xffffffff

static u32 gps_l2c_capabilities;
static ndb_element_metadata_t gps_l2c_capabilities_md;
static sbp_msg_callbacks_node_t l2c_mask_callback_node;
static void l2c_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context);

static ndb_file_t gps_l2c_capb_file = {
  .name = GPS_L2C_CAPB_FILE_NAME,
  .type = GPS_L2C_CAPB_FILE_TYPE,
  .block_data = (u8*)&gps_l2c_capabilities,
  .block_md = &gps_l2c_capabilities_md,
  .block_size = sizeof(gps_l2c_capabilities),
  .block_count = 1
};

void ndb_l2c_capb_init(void)
{
  ndb_load_data(&gps_l2c_capb_file, false);

  if (0 == (gps_l2c_capabilities_md.nv_data.state & NDB_IE_VALID) ||
      0 == gps_l2c_capabilities) {
    u32 new_val = GPS_L2C_CAPAB_DEFAULT;
    ndb_update(&new_val, NDB_DS_INIT, &gps_l2c_capabilities_md);
    log_info("Save default L2C capability 0x%08" PRIX32, new_val);
  } else {
    log_info("Loaded L2C capability 0x%08" PRIX32, gps_l2c_capabilities);
  }

  /* register L2C capability SBP callback */
  sbp_register_cbk(
    SBP_MSG_SV_CONFIGURATION_GPS,
    &l2c_msg_callback,
    &l2c_mask_callback_node
  );
}

ndb_op_code_t ndb_gps_l2cm_l2c_cap_read(u32 *l2c_cap)
{
  return ndb_retrieve(&gps_l2c_capabilities_md, l2c_cap, sizeof(*l2c_cap),
                      NULL, NULL);
}

ndb_op_code_t ndb_gps_l2cm_l2c_cap_store(const u32 *l2c_cap,
                                         ndb_data_source_t src)
{
  if (NULL != l2c_cap && GPS_L2C_CAPAB_DEFAULT != *l2c_cap) {
    log_info("Updating L2C capability 0x%08" PRIX32, *l2c_cap);
  }
  return ndb_update(l2c_cap, src, &gps_l2c_capabilities_md);
}

static void l2c_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)len; (void) context;

  log_info("L2C capabilities received from peer");

  /* unpack received message */
  u32 l2c_mask = ((msg_sv_configuration_gps_t*)msg)->l2c_mask;

  /* store message in NDB */
  ndb_gps_l2cm_l2c_cap_store(&l2c_mask, NDB_DS_SBP);
}
