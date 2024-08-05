/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <starling/util/sbp/misc.h>
#include <swiftnav/pvt_result.h>

u8 sbp_get_time_quality_flags(u8 time_qual) {
  /* time_qual is the same as get_time_quality() return in ME API */
  u8 flags = 0;
  switch (time_qual) {
    case TIME_FINE: /* Intentionally FALLTHROUGH */
    case TIME_FINEST:
      /* Time comes from the measurement engine and if it is
       * FINE or FINEST then we call it GNSS derived (flags=1) */
      flags = GNSS_TIME;
      break;
    case TIME_PROPAGATED:
      /* Time Propagated (flags=2) */
      flags = PROPAGATED_TIME;
      break;
    case TIME_COARSE:  /* Intentionally FALLTHROUGH  */
    case TIME_UNKNOWN: /* Intentionally FALLTHROUGH  */
    default:
      /* Time COARSE or UNKNOWN ->  mark time as invalid    */
      flags = NO_TIME;
  }
  return flags;
}

u32 round_tow_ms(double tow) {
  /* week roll-over */
  u32 tow_ms = round(tow * SECS_MS);
  while (tow_ms >= WEEK_MS) {
    tow_ms -= WEEK_MS;
  }
  return tow_ms;
}

void round_time_nano(const gps_time_t *t_in, sbp_gps_time_t *t_out) {
  t_out->wn = t_in->wn;
  t_out->tow = round(t_in->tow * SECS_MS);
  t_out->ns_residual =
      lround((t_in->tow - (double)t_out->tow / SECS_MS) * SECS_NS);
  /* week roll-over */
  if (t_out->tow >= WEEK_MS) {
    t_out->wn++;
    t_out->tow -= WEEK_MS;
  }
}

sbp_gnss_signal_t sid_to_sbp(const gnss_signal_t from) {
  sbp_gnss_signal_t sbp_sid = {
      .code = from.code,
      .sat = from.sat,
  };

  return sbp_sid;
}

gnss_signal_t sid_from_sbp(const sbp_gnss_signal_t from) {
  gnss_signal_t sid = {
      .code = from.code,
      .sat = from.sat,
  };

  return sid;
}
