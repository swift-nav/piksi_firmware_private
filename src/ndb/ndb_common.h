/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
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
  NDB_ERR_UNCONFIRMED_DATA,/**< Data can't be verified */
  NDB_ERR_ALGORITHM_ERROR, /**< Error */
  NDB_ERR_NO_DATA,         /**< No data to process */
  NDB_ERR_NO_CHANGE,       /**< Data has not been updated */
  NDB_ERR_OLDER_DATA,      /**< Data is older than existing */
  NDB_ERR_TIME_UNKNOWN,    /**< TAI time is not available */
} ndb_op_code_t;

typedef enum ndb_data_source
{
  NDB_DS_UNDEFINED = 0, /**< Undefined data source */
  NDB_DS_INIT,          /**< Initial hard-coded value */
  NDB_DS_RECEIVER,      /**< Data received from satellites */
  NDB_DS_SBP,           /**< Data received over SBP */
  NDB_DS_NV             /**< Data loaded from non-volatile memory */
} ndb_data_source_t;

/** NDB Timestamp: NAP time [s] */
typedef u32 ndb_timestamp_t;

#ifndef NDB_WEAK
#define NDB_WEAK __attribute__ ((weak, alias ("ndb_not_implemented")))
#endif

enum ndb_op_code ndb_not_implemented(void) __attribute__ ((weak));
inline enum ndb_op_code ndb_not_implemented(void) { return NDB_ERR_UNSUPPORTED; }
#endif /* SRC_NDB_COMMON_H_ */
