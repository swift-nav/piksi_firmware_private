/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_NDB_LGF_H_
#define SRC_NDB_LGF_H_

#include "ndb_common.h"
#include "position/position.h"

void ndb_lgf_init(void);
ndb_op_code_t ndb_lgf_read(last_good_fix_t *lgf);
ndb_op_code_t ndb_lgf_store(const last_good_fix_t *lgf);
ndb_op_code_t ndb_cached_lgf_read(last_good_fix_t *lgf);

#endif /* SRC_NDB_LGF_H_ */
