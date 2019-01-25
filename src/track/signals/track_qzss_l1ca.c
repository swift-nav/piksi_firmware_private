/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_qzss_l1ca.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_qzss_l2c.h" /* for L1C/A to L2C tracking handover */

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <string.h>

/** QZSS L1 C/A parameter section name */
#define QZSS_L1CA_TRACK_SETTING_SECTION "qzss_l1ca_track"

/** QZSS L1 C/A configuration container */
static tp_tracker_config_t qzss_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for QZSS L1 C/A */
static tracker_interface_function_t tracker_qzss_l1ca_init;
static tracker_interface_function_t tracker_qzss_l1ca_update;

/** QZSS L1 C/A tracker interface */
static const tracker_interface_t tracker_interface_qzss_l1ca = {
    .code = CODE_QZS_L1CA,
    .init = tracker_qzss_l1ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_qzss_l1ca_update,
};

/** Register QZSS L1 C/A tracker into the the tracker interface & settings
 *  framework.
 */
void track_qzss_l1ca_register(void) {
  tracker_interface_register(&tracker_interface_qzss_l1ca);
}

static void tracker_qzss_l1ca_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &qzss_l1ca_config);
}

static void tracker_qzss_l1ca_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &qzss_l1ca_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool settled = (0 == (tracker->flags & TRACKER_FLAG_RECOVERY_MODE));
  bool inlock = tracker_has_all_locks(tracker);

  if (inlock && settled && (TOW_UNKNOWN != (tracker->TOW_ms))) {
    log_debug_mesid(tracker->mesid, "calling qzss_l1ca_to_l2c_handover()");

    /* Start L2C tracker if not running */
    qzss_l1ca_to_l2c_handover(tracker->sample_count,
                              tracker->mesid.sat,
                              tracker->code_phase_prompt,
                              tracker->doppler_freq_hz,
                              tracker->cn0,
                              tracker->TOW_ms);
  }
}
