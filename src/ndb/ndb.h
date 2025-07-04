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

#ifndef SWIFTNAV_NDB_H
#define SWIFTNAV_NDB_H

#include <ndb/ndb_almanac.h>
#include <ndb/ndb_ephemeris.h>
#include <ndb/ndb_gnss_capabilities.h>
#include <ndb/ndb_iono.h>
#include <ndb/ndb_lgf.h>
#include <ndb/ndb_utc.h>

void ndb_setup(void);
void ndb_sbp_updates(void);

#endif /* SWIFTNAV_NDB_H */
