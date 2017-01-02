/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 * Contact: Adel Mamin <adelm@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TRACK_GPS_L2CL_H
#define SWIFTNAV_TRACK_GPS_L2CL_H

#include <libswiftnav/common.h>

#ifdef TRACK_GPS_L2CL_INTERNAL
# define L2CL_WEAK
#else
# define L2CL_WEAK __attribute__ ((weak, alias ("l2cl_not_implemented")))
#endif  /* TRACK_GPS_L2CL_INTERNAL */

int l2cl_not_implemented(void) __attribute__ ((weak));
inline int l2cl_not_implemented(void) { return -1; }

/* not weak as it is used in L2C builds only */
void track_gps_l2cl_register(void);

void do_l2cm_to_l2cl_handover(u32 sample_count,
                              u16 sat,
                              double code_phase,
                              double carrier_freq,
                              float cn0_init,
                              s32 TOW_ms) L2CL_WEAK;

#endif /* SWIFTNAV_TRACK_GPS_L2CL_H */
