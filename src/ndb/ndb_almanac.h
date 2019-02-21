/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_NDB_ALMANAC_H_
#define SRC_NDB_ALMANAC_H_

#include <ndb/ndb_common.h>

#ifdef __cplusplus
extern "C" {
#endif

void ndb_almanac_init(void);
ndb_op_code_t ndb_almanac_read(gnss_signal_t sid, almanac_t *a);
ndb_op_code_t ndb_almanac_store(const gnss_signal_t *src_sid,
                                const almanac_t *a,
                                ndb_data_source_t ds,
                                u16 sender_id);
ndb_op_code_t ndb_almanac_erase(gnss_signal_t sid);
ndb_op_code_t ndb_almanac_erase_by_src(gnss_signal_t src_sid);
ndb_op_code_t ndb_almanac_wn_read(u32 toa, u16 *wn);
ndb_op_code_t ndb_almanac_wn_store(
    gnss_signal_t sid, u32 toa, u16 wn, ndb_data_source_t ds, u16 sender_id);
void ndb_almanac_sbp_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_NDB_ALMANAC_H_ */
