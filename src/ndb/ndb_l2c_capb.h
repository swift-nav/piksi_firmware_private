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

#ifndef SRC_NDB_L2C_CAPB_H_
#define SRC_NDB_L2C_CAPB_H_

#include <ndb/ndb_common.h>

/**
 * L2CM specific
 */
void ndb_l2c_capb_init(void);
ndb_op_code_t ndb_gps_l2cm_l2c_cap_read(u32 *l2c_cap) NDB_WEAK;
ndb_op_code_t ndb_gps_l2cm_l2c_cap_store(const gnss_signal_t *sid,
                                         const u32 *l2c_cap,
                                         ndb_data_source_t src,
                                         u16 sender_id) NDB_WEAK;
bool ndb_gps_l2cm_l2c_cap_pending(const gnss_signal_t *sid,
                                  const u32 *l2c_cap,
                                  ndb_data_source_t src,
                                  u16 sender_id) NDB_WEAK;

#endif /* SRC_NDB_L2C_CAPB_H_ */
