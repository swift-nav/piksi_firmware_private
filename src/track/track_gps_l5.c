/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tommi.paakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_gps_l5.h"
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
#include <libswiftnav/gnss_capabilities.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <assert.h>
#include <string.h>

/** GPS L5 configuration section name */
#define L5_TRACK_SETTING_SECTION "l5c_track"

/** GPS L5 configuration container */
static tp_tracker_config_t gps_l5_config = TP_TRACKER_DEFAULT_CONFIG;

/* Forward declarations of interface methods for GPS L5 */
static tracker_interface_function_t tracker_gps_l5_init;
static tracker_interface_function_t tracker_gps_l5_update;

/** GPS L5 tracker interface */
static const tracker_interface_t tracker_interface_gps_l5 = {
    .code = CODE_GPS_L5Q,
    .init = tracker_gps_l5_init,
    .disable = tp_tracker_disable,
    .update = tracker_gps_l5_update,
};

/** GPS L5 tracker interface list element */
static tracker_interface_list_element_t tracker_interface_list_element_gps_l5 =
    {.interface = &tracker_interface_gps_l5, .next = 0};

/** Register L5 tracker into the the tracker interface & settings framework.
 */
void track_gps_l5_register(void) {
  tracker_interface_register(&tracker_interface_list_element_gps_l5);
}

/** Do L1CA to L5 handover.
 *
 * The condition for the handover is that TOW must be known.
 *
 * \param[in] sample_count NAP sample count
 * \param[in] sat          Satellite ID
 * \param[in] code_phase   code phase [chips]
 * \param[in] carrier_freq Doppler [Hz]
 * \param[in] cn0          CN0 estimate [dB-Hz]
 * \param[in] TOW_ms       Latest decoded TOW [ms]
 */
void do_l1ca_to_l5_handover(u32 sample_count,
                            u16 sat,
                            double code_phase,
                            double carrier_freq,
                            float cn0_init,
                            s32 TOW_ms) {
  (void)TOW_ms;
  /* compose L5 MESID: same SV, but code is L5 */
  me_gnss_signal_t mesid = construct_mesid(CODE_GPS_L5Q, sat);

  if (!tracking_startup_ready(mesid)) {
    return; /* L5 signal from the SV is already in track */
  }

  if (0 == (gnss_capab.gps_l5 & (1ULL << (sat - 1)))) {
    return;
  }

  if (32 != mesid.sat) return;

  if (!handover_valid(code_phase, GPS_L1CA_CHIPS_NUM)) {
    log_warn_mesid(
        mesid, "Unexpected L1CA to L5 hand-over code phase: %f", code_phase);
    return;
  }

  tracking_startup_params_t startup_params = {
      .mesid = mesid,
      .sample_count = sample_count,
      /* recalculate doppler freq for L5 from L1 */
      .carrier_freq = carrier_freq * GPS_L5_HZ / GPS_L1_HZ,
      .code_phase = code_phase * GPS_L5_CHIPPING_RATE / GPS_CA_CHIPPING_RATE,
      /* chips to correlate during first 1 ms of tracking */
      .chips_to_correlate = code_to_chip_rate(mesid.code) * 1e-3,
      /* get initial cn0 from parent L1CA channel */
      .cn0_init = cn0_init,
      .elevation = TRACKING_ELEVATION_UNKNOWN};

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_debug_mesid(mesid, "L5 handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_mesid(mesid, "Failed to start L5 tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_gps_l5_init(tracker_channel_t *tracker_channel) {
  tp_tracker_init(tracker_channel, &gps_l5_config);

  tracker_bit_sync_set(tracker_channel, /* bit_phase_ref = */ 0);
}

static void tracker_gps_l5_update(tracker_channel_t *tracker_channel) {
  u32 cflags = tp_tracker_update(tracker_channel, &gps_l5_config);

  bool bit_aligned =
      ((0 != (cflags & TPF_BSYNC_UPD)) && tracker_bit_aligned(tracker_channel));

  if (!bit_aligned) {
    return;
  }

  /* TOW manipulation on bit edge */
  tracker_tow_cache(tracker_channel);

  bool confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));
  bool in_phase_lock = (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK));

  if (in_phase_lock && confirmed) {
    /* naturally synched as we track */
    s8 symb_sign = SIGN(tracker_channel->corrs.corr_epl.very_late.I);
    s8 pol_sign = SIGN(tracker_channel->cp_sync.polarity);
    log_debug("G%02d L5 %+2d %+3d",
              tracker_channel->mesid.sat,
              symb_sign,
              tracker_channel->cp_sync.polarity);
    if (symb_sign != pol_sign) {
      tracker_channel->cp_sync.polarity = symb_sign;
      tracker_channel->cp_sync.synced = false;
    } else {
      tracker_channel->cp_sync.polarity += symb_sign;
      if (100 == ABS(tracker_channel->cp_sync.polarity)) {
        tracker_channel->cp_sync.synced = true;
        tracker_channel->cp_sync.polarity -= symb_sign; /* saturate */
      } else {
        tracker_channel->cp_sync.synced = false;
      }
    }
    if (tracker_channel->cp_sync.synced) {
      tracker_channel->bit_polarity = ((tracker_channel->cp_sync.polarity) < 0)
                                          ? BIT_POLARITY_NORMAL
                                          : BIT_POLARITY_INVERTED;
    } else {
      tracker_channel->bit_polarity = BIT_POLARITY_UNKNOWN;
    }
    update_bit_polarity_flags(tracker_channel);
  }
}
