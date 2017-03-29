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
#include "sbp.h"
#include "sbp_utils.h"
#include "timing.h"

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

/**
 * Initializes the data source to NV for
 * non-volatile GPS L2C capability entry.
 * Applied at receiver start up.
 *
 */
void ndb_l2c_capb_init_ds(void)
{
  u32 l2c_cap;
  ndb_op_code_t res = ndb_gps_l2cm_l2c_cap_read(&l2c_cap);
  if (NDB_ERR_NONE == res) {
    ndb_update_init_ds(&l2c_cap, NDB_DS_NV, &gps_l2c_capabilities_md);
  }
}

void ndb_l2c_capb_init(void)
{

  static bool erase_l2c_capb = true;
  SETTING("ndb", "erase_l2c_capb", erase_l2c_capb, TYPE_BOOL);

  ndb_load_data(&gps_l2c_capb_file, erase_l2c_capb);

  ndb_l2c_capb_init_ds();

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
                      NULL, NULL, NULL);
}

/**
 * Store L2C capability information
 *
 * \param[in] sid         GNSS signal identifier for the source of L2C
 *                        capability data in case of data source being
 *                        NDB_DS_RECEIVER, NULL for other cases.
 * \param[in] l2c_cap     L2C capability mask
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP.
 *                        In other cases set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE            On success. L2C capability is updated.
 * \retval NDB_ERR_NO_CHANGE       On success. L2C capability is unchanged.
 * \retval NDB_ERR_BAD_PARAM       Parameter errors.
 */
ndb_op_code_t ndb_gps_l2cm_l2c_cap_store(const gnss_signal_t *sid,
                                         const u32 *l2c_cap,
                                         ndb_data_source_t src,
                                         u16 sender_id)
{
  ndb_op_code_t res = NDB_ERR_NONE;
  if (TIME_FINE == time_quality) {
    /* If GPS time is known, save l2c capabilities to NDB. */
    res = ndb_update(l2c_cap, src, &gps_l2c_capabilities_md);

    if (NULL != l2c_cap && NDB_ERR_NONE == res) {
      log_info("Updating L2C capability 0x%08" PRIX32, *l2c_cap);
    }
  } else {
    /* If GPS time is unknown, no updates to NDB */
    res = NDB_ERR_TIME_UNKNOWN;
  }

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_L2C_CAP,
                     res,
                     src,
                     NULL,
                     sid,
                     sender_id);

  return res;
}

/**
 * Store pending L2C capability information
 *
 * \param[in] sid         GNSS signal identifier for the source of L2C
 *                        capability data in case of data source being
 *                        NDB_DS_RECEIVER, NULL for other cases.
 * \param[in] l2c_cap     L2C capability mask
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP.
 *                        In other cases set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval    r           Flag indicating successful update.
 *
 */
bool ndb_gps_l2cm_l2c_cap_pending(const gnss_signal_t *sid,
                                  const u32 *l2c_cap,
                                  ndb_data_source_t src,
                                  u16 sender_id)
{
  bool r = false;
  if (TIME_FINE == time_quality) {
    /* If GPS time is known, save pending l2c capabilities to NDB. */
    ndb_op_code_t res = ndb_update(l2c_cap, src, &gps_l2c_capabilities_md);

    if (NULL != l2c_cap && NDB_ERR_NONE == res) {
      log_info("Updating L2C capability 0x%08" PRIX32, *l2c_cap);
      sbp_send_ndb_event(NDB_EVENT_STORE,
                         NDB_EVENT_OTYPE_L2C_CAP,
                         res,
                         src,
                         NULL,
                         sid,
                         sender_id);
      r = true;
    }
  }
  return r;
}

static void l2c_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)len; (void) context;

  log_info("L2C capabilities received from peer");

  /* unpack received message */
  u32 l2c_mask = ((msg_sv_configuration_gps_t*)msg)->l2c_mask;

  /* store message in NDB */
  ndb_gps_l2cm_l2c_cap_store(NULL, &l2c_mask, NDB_DS_SBP, sender_id);
}
