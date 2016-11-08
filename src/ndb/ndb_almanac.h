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

#ifndef SRC_NDB_ALMANAC_H_
#define SRC_NDB_ALMANAC_H_

#include <ndb/ndb_common.h>

void ndb_almanac_init(void);
ndb_op_code_t ndb_almanac_read(gnss_signal_t sid, almanac_t *a);
ndb_op_code_t ndb_almanac_store(const almanac_t *a, ndb_data_source_t src);
void ndb_almanac_sbp_update(void);
#endif /* SRC_NDB_ALMANAC_H_ */
