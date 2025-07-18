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
#include "ndb.h"

#include "ndb/ndb_internal.h"
#include "signal_db/signal_db.h"

/** Set up the NDB module. */
void ndb_setup(void) {
  ndb_init();
  platform_ndb_init();
  ndb_start();
}

void ndb_sbp_updates(void) { platform_ndb_sbp_updates(); }
