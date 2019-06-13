/*
 * Copyright (C) 2017 Swift Navigation Inc.
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
#include "track_bds2_b11.h"

#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_bds2_b2.h"
#include "track_bds3_b5.h"

/* Non-local headers */
#include <acq/manage.h>

/* Libraries */
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <string.h>

/** BDS2 L1 parameter section name */
#define BDS2_B11_TRACK_SETTING_SECTION "bds2_b11_track"

/** BDS2 L1 configuration container */
static tp_tracker_config_t bds2_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for Beidou2 B11 */
static tracker_interface_function_t tracker_bds2_b11_init;
static tracker_interface_function_t tracker_bds2_b11_update;

/** BDS2 B11 tracker interface */
static const tracker_interface_t tracker_interface_bds2_b11 = {
    .code = CODE_BDS2_B1,
    .init = tracker_bds2_b11_init,
    .disable = tp_tracker_disable,
    .update = tracker_bds2_b11_update,
};

static void tracker_bds2_b11_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &bds2_l1ca_config);
}

static void tracker_bds2_b11_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &bds2_l1ca_config);

  /* If BDS SV is marked unhealthy from B1, also drop B2 tracker */
  if (0 != (tracker->flags & TRACKER_FLAG_UNHEALTHY)) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_BDS2_B2, tracker->mesid.sat);
    tracker_drop_unhealthy(mesid_drop);
    return;
  }

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool settled = (0 == (tracker->flags & TRACKER_FLAG_RECOVERY_MODE));
  bool inlock = tracker_has_all_locks(tracker);

  if (inlock && settled) {
#if defined CODE_BDS2_B2_SUPPORT && CODE_BDS2_B2_SUPPORT > 0
    bds_b11_to_b2_handover(tracker->sample_count,
                           tracker->mesid.sat,
                           tracker->code_phase_prompt,
                           tracker->doppler_hz,
                           tracker->cn0);
#endif

#if defined CODE_BDS3_B5_SUPPORT && CODE_BDS3_B5_SUPPORT > 0
    bds_b1_to_b5_handover(tracker->sample_count,
                          tracker->mesid.sat,
                          tracker->code_phase_prompt,
                          tracker->doppler_hz,
                          tracker->cn0);
#endif
  }
}

/** Register BDS2 B11 tracker into the the tracker interface & settings
 *  framework.
 */
void track_bds2_b11_register(void) {
  tracker_interface_register(&tracker_interface_bds2_b11);
}
