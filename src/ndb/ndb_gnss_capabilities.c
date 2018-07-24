/*
 * Copyright (C) 2016 - 2018 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>
#include <string.h>
#include "ndb.h"
#include "ndb_internal.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"

/* one can use a simple Matlab script to generate these:
PRNS_PRESENT = [11 12 19  26 24 30 ...
                 8  9  1   2  7  3 ...
                 4  5 21  25 27 31];
mask = repmat('0', 1, 64);
mask(PRNS_PRESENT) = '1';
fl_mask = fliplr(mask);
fprintf(1, '0x%016Xull \n', bin2dec(fl_mask))
fprintf(1, 'E%02d\n', strfind(mask, '1'));
*/

/* clang-format off */
/* as of June 2018 */
static const gnss_capb_t gnss_capb_defaults = {
  .gps_active = (u64)0x0ffffffff,

  .gps_l2c = (u64)0x0f7814bfd,

  .gps_l5 = (u64)0x0a78003a5,

  .glo_active = (u32)0x03ffffff,
  .glo_l2of = (u32)0x03ffffff,
  .glo_l3 = (u32)0x00080100,

  .sbas_active = (u64)0x7ffff,
  .sbas_l5 = (u64)0x5b8c8,

  /* Note: BDS GEO SVs are marked as inactive,
  in order to prevent their acquisition.
  Configuration compliant with to
  http://www.csno-tarc.cn/system/basicinfo
  retrieved on May 2nd*/
  .bds_active = (u64)0x3ffffffe0,
  .bds_d2nav = (u64)0x1001f,
  .bds_b2 = (u64)0x17fff,
  .bds_b2a = (u64)0x3fffe8000,

  .qzss_active = (u32)0x3ff,

  .gal_active = (u64)0x0000000067940DDF,
  .gal_e5 = (u64)0x0000000067940DDF,
};
/* clang-format on */

static gnss_capb_t ndb_gnss_capb;
static ndb_element_metadata_t ndb_gnss_capb_md;
static sbp_msg_callbacks_node_t gnss_cabp_cb_node;
static void gnss_cabp_msg_cb(u16 sender_id, u8 len, u8 msg[], void *context);

/* clang-format off */
static ndb_file_t ndb_gnss_capb_file = {
  .name = "persistent/ndb/gnss_capb",
  .type = "gnss capb",
  .block_data = (u8 *)&ndb_gnss_capb,
  .block_md = &ndb_gnss_capb_md,
  .block_size = sizeof(ndb_gnss_capb),
  .block_count = 1
};
/* clang-format on */

void ndb_gnss_capb_init(void) {
  static bool erase = false;
  SETTING("ndb", "erase_gnss_capb", erase, TYPE_BOOL);

  ndb_load_data(&ndb_gnss_capb_file, erase || !NDB_USE_NV_GNSS_CAPB);

  /* sanity checks */
  gnss_capb_t *gc = &ndb_gnss_capb;
  u32 gps_l2c = (u32)gc->gps_l2c;
  ndb_element_metadata_t *gcmd = &ndb_gnss_capb_md;
  if ((0 == (gcmd->nv_data.state & NDB_IE_VALID)) || (0 == gps_l2c)) {
    ndb_update(&gnss_capb_defaults, NDB_DS_INIT, gcmd);
    gps_l2c = (u32)gc->gps_l2c;
    log_info("Use default gnss capb (gps_l2c=0x%08" PRIX32 " etc.)", gps_l2c);
  } else {
    log_info("Loaded gnss capb (gps_l2c=0x%08" PRIX32 " etc.)", gps_l2c);
  }

  /* register gnss capability SBP callback */
  sbp_register_cbk(SBP_MSG_GNSS_CAPB, &gnss_cabp_msg_cb, &gnss_cabp_cb_node);
}

/**
 * Store gnss capabilities
 *
 * \param[in] sid         GNSS signal identifier for the source of gnss
 *                        capabilities data in case of data source being
 *                        NDB_DS_RECEIVER, NULL for other cases.
 * \param[in] gnss_capb   gnss capabilities
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP. In other cases
 *                        set to NDB_EVENT_SENDER_ID_VOID.
 * \return NDB error code
 */
static ndb_op_code_t ndb_store_gnss_capb(const gnss_signal_t *sid,
                                         const gnss_capb_t *gc,
                                         ndb_data_source_t src,
                                         u16 sender_id) {
  assert(gc);
  ndb_op_code_t res = ndb_update(gc, src, &ndb_gnss_capb_md);

  if (NDB_ERR_NONE == res) {
    log_info("Updating gnss capabilities (gps_l2c=0x%08" PRIX32 " etc.)",
             (u32)gc->gps_l2c);
  }

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_GNSS_CAPB,
                     res,
                     src,
                     /*object_sid=*/NULL,
                     sid,
                     sender_id);

  return res;
}

/** Returns gnss capabilities */
const gnss_capb_t *ndb_get_gnss_capb(void) { return &ndb_gnss_capb; }

/**
 * Store gnss capabilities
 *
 * \param[in] sid         GNSS signal identifier for the source of gnss
 *                        capabilities data in case of data source being
 *                        NDB_DS_RECEIVER, NULL for other cases.
 * \param[in] gnss_capb   gnss capabilities
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP. In other cases
 *                        set to NDB_EVENT_SENDER_ID_VOID.
 * \return NDB error code
 */
ndb_op_code_t ndb_store_gps_l2c_capb(u64 capb, const gnss_signal_t *sid) {
  ndb_data_source_t src = NDB_DS_RECEIVER;
  u16 sender_id = NDB_EVENT_SENDER_ID_VOID;
  gnss_capb_t gc = ndb_gnss_capb;
  gc.gps_l2c = capb;
  return ndb_store_gnss_capb(sid, &gc, src, sender_id);
}

static void gnss_cabp_msg_cb(u16 sender_id, u8 len, u8 msg[], void *context) {
  (void)context;

  log_info("gnss capb received from peer");
  gnss_capb_t *gc = &((msg_gnss_capb_t *)msg)->gc;
  ndb_store_gnss_capb(/*sid=*/NULL, gc, NDB_DS_SBP, sender_id);

  sbp_send_msg_(SBP_MSG_GNSS_CAPB, len, msg, MSG_FORWARD_SENDER_ID);
}
