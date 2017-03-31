/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Niilo Sirola <niilo.sirola@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_NDB_UTC_H_
#define SRC_NDB_UTC_H_

#include <libswiftnav/time.h>
#include <ndb/ndb_common.h>

void ndb_utc_params_init(void);
ndb_op_code_t ndb_utc_params_read(utc_params_t *utc_params) NDB_WEAK;
ndb_op_code_t ndb_utc_params_store(const gnss_signal_t *sid,
                                   const utc_params_t *utc_params,
                                   ndb_data_source_t src,
                                   u16 sender_id) NDB_WEAK;

#endif /* SRC_NDB_UTC_H_ */
