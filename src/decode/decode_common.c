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

#include <libswiftnav/signal.h>
#include <libswiftnav/time.h>
#include <libswiftnav/nav_msg_glo.h>

#include "timing.h"

static gps_time_t glo2gps_with_utc_params_cb(me_gnss_signal_t mesid,
                                             const glo_time_t *glo_t)
{
  return glo2gps_with_utc_params(mesid, glo_t);
}

void nav_msg_init_glo_with_cb(nav_msg_glo_t *n, me_gnss_signal_t mesid)
{
  nav_msg_init_glo(n, mesid, glo2gps_with_utc_params_cb);
}

