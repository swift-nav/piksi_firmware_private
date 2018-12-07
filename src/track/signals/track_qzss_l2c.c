/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_qzss_l2c.h"
#include "signal_db/signal_db.h"
#include "track/track_api.h"
#include "track/track_common.h"
#include "track/track_interface.h"
#include "track/track_sid_db.h"
#include "track/track_utils.h"
#include "track_gps_l2c.h"

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

/** QZSS L2C configuration section name */
#define QZSS_L2C_TRACK_SETTING_SECTION "qzss_l2c_track"

/** QZSS L2C configuration container */
static tp_tracker_config_t qzss_l2c_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for QZSS L2C */
static tracker_interface_function_t tracker_qzss_l2c_init;
static tracker_interface_function_t tracker_qzss_l2c_update;

/** QZSS L2C tracker interface */
static const tracker_interface_t tracker_interface_qzss_l2c = {
    .code = CODE_QZS_L2CM,
    .init = tracker_qzss_l2c_init,
    .disable = tp_tracker_disable,
    .update = tracker_qzss_l2c_update,
};

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_qzss_l2c_register(void) {
  lp1_filter_compute_params(&qzss_l2c_config.xcorr_f_params,
                            qzss_l2c_config.xcorr_cof,
                            SECS_MS / QZS_L2C_SYMBOL_LENGTH_MS);

  tracker_interface_register(&tracker_interface_qzss_l2c);
}

static void tracker_qzss_l2c_init(tracker_t *tracker) {
  gps_l2cm_tracker_data_t *data = &tracker->gps_l2cm;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(tracker, &qzss_l2c_config);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(tracker, /* bit_phase_ref = */ 0);
}

static void tracker_qzss_l2c_update(tracker_t *tracker) {
  u32 cflags = tp_tracker_update(tracker, &qzss_l2c_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker);

  bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));
  bool in_phase_lock = (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK));

  if (in_phase_lock && confirmed) {
    /* naturally synched as we track */
    tracker->bit_polarity = BIT_POLARITY_INVERTED;
    tracker_update_bit_polarity_flags(tracker);
  }
}
