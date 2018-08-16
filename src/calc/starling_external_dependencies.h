/*
 * Copyright (C) 2014-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_EXTERNAL_DEPENDENCIES_H_
#define STARLING_EXTERNAL_DEPENDENCIES_H_

/* For the time being, Starling has a couple unresolved external dependencies.
 * They are enumerated here.
 *
 * As we work to the functionality provided in these functions with
 * implementations internal to Starling, this file should vanish. */

typedef enum ndb_op_code {
  NDB_ERR_NONE = 0,         /**< No error */
  NDB_ERR_MISSING_IE,       /**< DB doesn't contain value of this IE */
  NDB_ERR_UNSUPPORTED,      /**< Feature is not implemented */
  NDB_ERR_FILE_IO,          /**< File I/O error */
  NDB_ERR_INIT_DONE,        /**< New file created */
  NDB_ERR_BAD_PARAM,        /**< Bad parameter */
  NDB_ERR_UNCONFIRMED_DATA, /**< Data can't be verified */
  NDB_ERR_ALGORITHM_ERROR,  /**< Error */
  NDB_ERR_NO_DATA,          /**< No data to process */
  NDB_ERR_NO_CHANGE,        /**< Data has not been updated */
  NDB_ERR_OLDER_DATA,       /**< Data is older than existing */
  NDB_ERR_AGED_DATA,        /**< Data in NDB has aged out */
  NDB_ERR_GPS_TIME_MISSING, /**< GPS time missing, can't use NDB data */
} ndb_op_code_t;

ndb_op_code_t ndb_ephemeris_read(const gnss_signal_t sid,
                                 ephemeris_t *ephemeris);
ndb_op_code_t ndb_iono_corr_read(ionosphere_t *iono);

bool track_sid_db_elevation_degrees_get(const gnss_signal_t sid,
                                        double *elevation);

bool shm_navigation_unusable(const gnss_signal_t sid);

#endif
