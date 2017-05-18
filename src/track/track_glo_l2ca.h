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

#include <libswiftnav/common.h>

#ifdef TRACK_GLO_L2CA_INTERNAL
# define L2CA_WEAK
#else
# define L2CA_WEAK __attribute__ ((weak, alias ("l2ca_not_implemented")))
#endif  /* TRACK_GLO_L2CA_INTERNAL */

int l2ca_not_implemented(void) __attribute__ ((weak));
inline int l2ca_not_implemented(void) { return -1; }

/* not weak as it is used in L2CA builds only */
void track_glo_l2ca_register(void);

void do_glo_l1ca_to_l2ca_handover(u32 sample_count,
                                  u16 sat,
                                  float code_phase_chips,
                                  double carrier_freq_hz,
                                  float init_cn0_dbhz) L2CA_WEAK;

#endif /* SWIFTNAV_TRACK_GLO_L2CA_H */
