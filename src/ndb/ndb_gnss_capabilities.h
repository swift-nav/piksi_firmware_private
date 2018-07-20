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

#ifndef SRC_NDB_GNSS_CAPB_H_
#define SRC_NDB_GNSS_CAPB_H_

#include <libsbp/observation.h>
#include <ndb/ndb_common.h>

void ndb_gnss_capb_init(void);
const gnss_capb_t* ndb_get_gnss_capb(void);
ndb_op_code_t ndb_store_gps_l2c_capb(u64 capb, const gnss_signal_t* src_sid);

#endif /* SRC_NDB_GNSS_CAPB_H_ */
