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

#ifndef SRC_NDB_IONO_H_
#define SRC_NDB_IONO_H_

#include <libswiftnav/ionosphere.h>
#include <ndb/ndb_common.h>

void ndb_iono_init();
enum ndb_op_code ndb_iono_corr_read(ionosphere_t *iono) NDB_WEAK;
enum ndb_op_code ndb_iono_corr_store(ionosphere_t *iono,
                                            enum ndb_data_source src) NDB_WEAK;

void ndb_sbp_iono_reg_cbk();

#endif /* SRC_NDB_IONO_H_ */
