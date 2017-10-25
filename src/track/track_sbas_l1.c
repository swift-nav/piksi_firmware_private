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
#include "track_sbas_l1.h"
#include "track_cn0.h"
#include "track_sid_db.h"

/* Non-local headers */
#include <manage.h>
#include <ndb.h>
#include <platform_track.h>
#include <signal.h>
#include <track.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <string.h>

/** SBAS L1 parameter section name */
#define SBAS_L1_TRACK_SETTING_SECTION "sbas_l1_track"

/** SBAS L1 configuration container */
static tp_tracker_config_t sbas_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for SBAS L1 */
static tracker_interface_function_t tracker_sbas_l1ca_init;
static tracker_interface_function_t tracker_sbas_l1ca_update;

/** SBAS L1 tracker interface */
static const tracker_interface_t tracker_interface_sbas_l1ca = {
    .code = CODE_SBAS_L1CA,
    .init = tracker_sbas_l1ca_init,
    .disable = tp_tracker_disable,
    .update = tracker_sbas_l1ca_update,
};

/** SBAS L1 C/A tracker interface list element */
static tracker_interface_list_element_t
    tracker_interface_list_element_sbas_l1ca = {
        .interface = &tracker_interface_sbas_l1ca, .next = 0};

/** Register SBAS L1 tracker into the the tracker interface & settings
 *  framework.
 */
void track_sbas_l1_register(void) {
  tracker_interface_register(&tracker_interface_list_element_sbas_l1ca);
}

static void tracker_sbas_l1ca_init(tracker_channel_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &sbas_l1ca_config);
}

/**
 * Performs ToW caching and propagation.
 *
 * SBAS L2 tracker performs ToW update/propagation only on bit edge. This makes
 * it more robust to propagation errors.
 *
 * \param[in,out  tracker_channel Tracker channel data
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
static void update_tow_sbas_l1ca(tracker_channel_t *tracker_channel,
                                 u32 cycle_flags) {
  me_gnss_signal_t mesid = tracker_channel->mesid;

  tp_tow_entry_t tow_entry;

  gnss_signal_t sid = construct_sid(mesid.code, mesid.sat);
  track_sid_db_load_tow(sid, &tow_entry);

  u64 sample_time_tk = nap_sample_time_to_count(tracker_channel->sample_count);

  bool aligned = false;

  if (0 != (cycle_flags & TPF_BSYNC_UPD) &&
      tracker_bit_aligned(tracker_channel)) {
    /* Check current state: do we have bit/ToW alignment */
    aligned = true;
  }

  if (TOW_UNKNOWN != tracker_channel->TOW_ms && aligned) {
    /*
     * Verify ToW alignment
     * Current block assumes the bit sync has been reached and current
     * interval has closed a bit interval. ToW shall be aligned by bit
     * duration, which is 20ms for SBAS L1.
     */
    u8 tail = tracker_channel->TOW_ms % SBAS_L1CA_BIT_LENGTH_MS;
    if (0 != tail) {
      s8 error_ms = tail < (SBAS_L1CA_BIT_LENGTH_MS >> 1)
                        ? -tail
                        : SBAS_L1CA_BIT_LENGTH_MS - tail;

      log_error_mesid(mesid,
                      "[+%" PRIu32
                      "ms] TOW error detected: "
                      "error=%" PRId8 "ms old_tow=%" PRId32,
                      tracker_channel->update_count,
                      error_ms,
                      tracker_channel->TOW_ms);

      /* This is rude, but safe. Do not expect it to happen normally. */
      tracker_channel->flags |= TRACKER_FLAG_OUTLIER;
    }
  }

  if (TOW_UNKNOWN == tracker_channel->TOW_ms &&
      TOW_UNKNOWN != tow_entry.TOW_ms) {
    /* ToW is not known, but there is a cached value */
    s32 ToW_ms = TOW_UNKNOWN;
    double error_ms = 0;
    u64 time_delta_tk = sample_time_tk - tow_entry.sample_time_tk;
    u8 ms_align =
        aligned ? SBAS_L1CA_BIT_LENGTH_MS : SBAS_L1CA_PSYMBOL_LENGTH_MS;

    ToW_ms =
        tp_tow_compute(tow_entry.TOW_ms, time_delta_tk, ms_align, &error_ms);

    if (TOW_UNKNOWN != ToW_ms) {
      log_debug_mesid(mesid,
                      "[+%" PRIu32 "ms] Initializing TOW from cache [%" PRIu8
                      "ms]"
                      " delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                      tracker_channel->update_count,
                      ms_align,
                      nap_count_to_ms(time_delta_tk),
                      ToW_ms,
                      error_ms);
      tracker_channel->TOW_ms = ToW_ms;
      if (tp_tow_is_sane(tracker_channel->TOW_ms)) {
        tracker_channel->flags |= TRACKER_FLAG_TOW_VALID;
      } else {
        log_error_mesid(mesid,
                        "[+%" PRIu32 "ms] Error TOW propagation %" PRId32,
                        tracker_channel->update_count,
                        tracker_channel->TOW_ms);
        tracker_channel->TOW_ms = TOW_UNKNOWN;
        tracker_channel->flags &= ~TRACKER_FLAG_TOW_VALID;
      }
    }
  }

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  if (confirmed && aligned &&
      (tracker_channel->cn0 >= CN0_TOW_CACHE_THRESHOLD)) {
    /* Update ToW cache:
     * - bit edge is reached
     * - CN0 is OK
     * - Tracker is confirmed
     */
    tow_entry.TOW_ms = tracker_channel->TOW_ms;
    tow_entry.sample_time_tk = sample_time_tk;
    track_sid_db_update_tow(sid, &tow_entry);
  }
}

static void tracker_sbas_l1ca_update(tracker_channel_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &sbas_l1ca_config);

  /* SBAS L1-specific ToW manipulation */
  update_tow_sbas_l1ca(tracker_channel, cflags);
}
