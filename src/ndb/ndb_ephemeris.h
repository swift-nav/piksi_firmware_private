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

#ifndef SRC_NDB_EPHEMERIS_H_
#define SRC_NDB_EPHEMERIS_H_

#include <ndb/ndb_common.h>

#ifdef __cplusplus
extern "C" {
#endif

void ndb_ephemeris_init(void);
ndb_op_code_t ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e) NDB_WEAK;
ndb_op_code_t ndb_ephemeris_store(const ephemeris_t *e,
                                  enum ndb_data_source,
                                  u16 sender_id) NDB_WEAK;
ndb_op_code_t ndb_ephemeris_info(gnss_signal_t sid, u8* valid,
                                 u8* health_bits, gps_time_t* toe,
                                 u32* fit_interval, float* ura) NDB_WEAK;
ndb_op_code_t ndb_ephemeris_erase(gnss_signal_t sid);
void ndb_ephemeris_sbp_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_NDB_EPHEMERIS_H_ */
