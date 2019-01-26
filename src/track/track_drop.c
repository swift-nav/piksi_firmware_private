/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode/decode.h"
#include "manage.h"
#include "track/track_common.h"
#include "track/track_flags.h"
#include "track/track_state.h"
#include "track/tracker.h"

/**
 * Helper to provide channel drop reason literal.
 * \param[in] reason Channel drop reason.
 *
 * \return Literal for the given \a reason.
 */
static const char *get_ch_drop_reason_str(ch_drop_reason_t reason) {
  const char *str = "";
  switch (reason) {
    case CH_DROP_REASON_ERROR:
      str = "error occurred, dropping";
      break;
    case CH_DROP_REASON_MASKED:
      str = "channel is masked, dropping";
      break;
    case CH_DROP_REASON_NO_BIT_SYNC:
      str = "no bit sync, dropping";
      break;
    case CH_DROP_REASON_NO_PLOCK:
      str = "No pessimistic lock for too long, dropping";
      break;
    case CH_DROP_REASON_LOW_CN0:
      str = "low CN0 too long, dropping";
      break;
    case CH_DROP_REASON_XCORR:
      str = "cross-correlation confirmed, dropping";
      break;
    case CH_DROP_REASON_NO_UPDATES:
      str = "no updates, dropping";
      break;
    case CH_DROP_REASON_SV_UNHEALTHY:
      str = "SV is unhealthy, dropping";
      break;
    case CH_DROP_REASON_LEAP_SECOND:
      str = "Leap second event, dropping GLO signal";
      break;
    case CH_DROP_REASON_OUTLIER:
      str = "SV measurement outlier, dropping";
      break;
    case CH_DROP_REASON_SBAS_PROVIDER_CHANGE:
      str = "SBAS provider change, dropping";
      break;
    case CH_DROP_REASON_RAIM:
      str = "Measurement flagged by RAIM, dropping";
      break;
    case CH_DROP_REASON_NEW_MODE:
      str = "New tracker mode, dropping";
      break;
    case CH_DROP_REASON_NOISE_ESTIMATOR:
      str = "Noise estimation is over, dropping";
      break;
    default:
      assert(!"Unknown channel drop reason");
  }
  return str;
}

/**
 * Processes channel drop operation.
 *
 * The method logs channel drop reason message, actually disables tracking
 * channel components and updates ACQ hints for re-acqusition.
 *
 * \param[in,out] tracker Tracker channel data
 * \param[in] reason     Channel drop reason
 */
void tp_drop_channel(tracker_t *tracker, ch_drop_reason_t reason) {
  /* Read the required parameters from the tracking channel first to ensure
   * that the tracking channel is not restarted in the mean time.
   */
  const u32 flags = tracker->flags;
  me_gnss_signal_t mesid = tracker->mesid;
  u64 time_in_track_ms = tracker_timer_ms(&tracker->age_timer);

  /* Log message with appropriate priority. */
  if ((CH_DROP_REASON_ERROR == reason) ||
      (CH_DROP_REASON_NO_UPDATES == reason)) {
    log_error_mesid(mesid,
                    "[+%" PRIu64 "ms] nap_channel = %" PRIu8 " %s",
                    time_in_track_ms,
                    tracker->nap_channel,
                    get_ch_drop_reason_str(reason));
  } else if ((0 == (flags & TRACKER_FLAG_CONFIRMED)) ||
             (CH_DROP_REASON_NEW_MODE == reason)) {
    /* Unconfirmed tracker messages are always logged at debug level.
       Same is for new tracker mode activation. */
    log_debug_mesid(mesid,
                    "[+%" PRIu64 "ms] %s",
                    time_in_track_ms,
                    get_ch_drop_reason_str(reason));
  } else {
    /* Confirmed tracker messages are always logged at info level */
    log_info_mesid(mesid,
                   "[+%" PRIu64 "ms] %s",
                   time_in_track_ms,
                   get_ch_drop_reason_str(reason));
  }

  update_acq_hints(tracker);

  /* Disable the decoder and tracking channels */
  decoder_channel_disable(tracker->nap_channel);
  tracker_disable(tracker->nap_channel);

  noise_update_mesid_status(mesid, /*intrack=*/false);
}
