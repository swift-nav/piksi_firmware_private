/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STUBS_MANAGE_H
#define STUBS_MANAGE_H

#include <libswiftnav/signal.h>

/** Unit test input data type */
typedef struct {
  u32 now_ms;
  u32 vis_mask, known_mask, track_mask;
  u32 lgf_stamp_ms;
  u32 deep_mask,     /**< Expected results */
      fallback_mask; /**< Expected results */
} test_case_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

bool mesid_is_tracked(const me_gnss_signal_t mesid);
bool is_glo_enabled(void);
bool is_sbas_enabled(void);
bool is_bds2_enabled(void);
bool is_qzss_enabled(void);
bool is_galileo_enabled(void);
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known);
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known);
void sm_calc_all_glo_visibility_flags(void);
u64 timing_getms(void);
bool sm_lgf_stamp(u64 *lgf_stamp);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif
