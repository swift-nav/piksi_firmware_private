/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_glo_l1of.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_utils.h"
#include "track_glo_l2of.h" /* for L1CA to L2CA tracking handover */

/* Non-local headers */
#include <manage.h>
#include <platform_track.h>

/* Libraries */
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>
#include <swiftnav/signal.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** GLO L1CA configuration section name */
#define GLO_L1CA_TRACK_SETTING_SECTION "glo_l1of_track"

static tp_tracker_config_t glo_l1of_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GLO L1CA */
static tracker_interface_function_t tracker_glo_l1of_init;
static tracker_interface_function_t tracker_glo_l1of_update;

/** GLO L1CA tracker interface */
static const tracker_interface_t tracker_interface_glo_l1of = {
    .code = CODE_GLO_L1OF,
    .init = tracker_glo_l1of_init,
    .disable = tp_tracker_disable,
    .update = tracker_glo_l1of_update,
};

/** Register GLO L1CA tracker into the the tracker interface & settings
 *  framework.
 */
void track_glo_l1of_register(void) {
  tracker_interface_register(&tracker_interface_glo_l1of);
}

static void tracker_glo_l1of_init(tracker_t *tracker) {
  tp_tracker_init(tracker, &glo_l1of_config);
}

static void tracker_glo_l1of_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &glo_l1of_config);

  /* If GLO SV is marked unhealthy from L1, also drop L2 tracker */
  if (0 != (tracker->flags & TRACKER_FLAG_UNHEALTHY)) {
    me_gnss_signal_t mesid_drop;
    mesid_drop = construct_mesid(CODE_GLO_L2OF, tracker->mesid.sat);
    tracker_drop_unhealthy(tracker, mesid_drop);
    return;
  }

  if (glo_slot_id_is_valid(tracker->glo_orbit_slot)) {
    gnss_signal_t sid = mesid2sid(tracker->mesid, tracker->glo_orbit_slot);
    if (!glo_active(sid)) {
      tracker_drop_unhealthy(tracker, tracker->mesid);
      return;
    }
  }

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool inlock = ((0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) ||
                 (0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)));
  double cn0_threshold_dbhz = TP_DEFAULT_CN0_USE_THRESHOLD_DBHZ;
  cn0_threshold_dbhz += TRACK_CN0_HYSTERESIS_THRES_DBHZ;
  bool cn0_high = (tracker->cn0 > cn0_threshold_dbhz);

  if (inlock && confirmed && cn0_high) {
    /* Start GLO L2CA tracker if not running */
    do_glo_l1of_to_l2of_handover(tracker->sample_count,
                                 tracker->mesid.sat,
                                 tracker->code_phase_prompt,
                                 tracker->carrier_freq,
                                 tracker->cn0);
  }
}
