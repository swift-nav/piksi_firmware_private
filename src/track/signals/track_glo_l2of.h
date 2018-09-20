/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Tommi Paakki <tpaakki@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_GLO_L2CA_H
#define SWIFTNAV_TRACK_GLO_L2CA_H

#include <swiftnav/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void track_glo_l2of_register(void);

void do_glo_l1of_to_l2of_handover(u32 sample_count,
                                  u16 sat,
                                  float code_phase_chips,
                                  double carrier_freq_hz,
                                  float init_cn0_dbhz);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_TRACK_GLO_L2CA_H */
