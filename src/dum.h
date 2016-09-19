/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_DUM_H
#define SWIFTNAV_DUM_H

#include "position.h"

#include <libswiftnav/signal.h>
#include <libswiftnav/time.h>

void dum_get_doppler_wndw(const gnss_signal_t *sid,
                          const gps_time_t *time_cur,
                          const last_good_fix_t *lgf,
                          float *doppler_min,
                          float *doppler_max);

void dum_report_reacq_result(const gnss_signal_t *sid, bool res);

#endif /* SWIFTNAV_DUM_H */
