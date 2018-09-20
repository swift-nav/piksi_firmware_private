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

#ifndef SRC_NDB_IONO_H_
#define SRC_NDB_IONO_H_

#include <ndb/ndb_common.h>
#include <swiftnav/ionosphere.h>

void ndb_iono_init(void);
ndb_op_code_t ndb_iono_corr_read(ionosphere_t *iono);
ndb_op_code_t ndb_iono_corr_store(const gnss_signal_t *sid,
                                  const ionosphere_t *iono,
                                  ndb_data_source_t src,
                                  u16 sender_id);

#endif /* SRC_NDB_IONO_H_ */
