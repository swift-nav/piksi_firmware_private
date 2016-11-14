/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri.atamaniouk@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_NDB_COMMON_H_
#define SRC_NDB_COMMON_H_

#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/almanac.h>
#include <libswiftnav/signal.h>
#include <ch.h>

typedef enum ndb_op_code
{
  NDB_ERR_NONE = 0,        /**< No error */
  NDB_ERR_MISSING_IE,      /**< DB doesn't contain value of this IE */
  NDB_ERR_UNSUPPORTED,     /**< Feature is not implemented */
  NDB_ERR_FILE_IO,         /**< File I/O error */
  NDB_ERR_INIT_DONE,       /**< New file created */
  NDB_ERR_BAD_PARAM,       /**< Bad parameter */
  NDB_ERR_UNRELIABLE_DATA, /**< Data can't be verified */
  NDB_ERR_ALGORITHM_ERROR, /**< Error */
  NDB_ERR_NO_DATA,         /**< No data to process */
  NDB_ERR_NO_CHANGE,       /**< Data has not been updated */
} ndb_op_code_t;

typedef enum ndb_data_source
{
  NDB_DS_UNDEFINED = 0,
  NDB_DS_INIT,
  NDB_DS_RECEIVER,
  NDB_DS_SBP
} ndb_data_source_t;

/** NDB Timestamp: TAI seconds since GPS epoch */
typedef u32 ndb_timestamp_t;

#ifndef NDB_WEAK
#define NDB_WEAK __attribute__ ((weak, alias ("ndb_not_implemented")))
#endif

enum ndb_op_code ndb_not_implemented(void) __attribute__ ((weak));
inline enum ndb_op_code ndb_not_implemented(void) { return NDB_ERR_UNSUPPORTED; }
#endif /* SRC_NDB_COMMON_H_ */
