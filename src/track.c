/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "manage.h"
#include "nap/nap_constants.h"
#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "signal.h"
#include "timing/timing.h"
#include "track.h"
#include "track/track_api.h"
#include "track/track_cn0.h"
#include "track/track_interface.h"
#include "track/track_sbp.h"
#include "track/track_sid_db.h"
#include "track/track_state.h"
#include "track/track_utils.h"

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

/** \} */
