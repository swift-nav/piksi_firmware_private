/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_SEARCH_MANAGER_UTILS_H
#define SWIFTNAV_SEARCH_MANAGER_UTILS_H

#include "swiftnav/common.h"
#include "swiftnav/signal.h"

bool sm_lgf_stamp(u64 *lgf_stamp);
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known);
void sm_calc_all_glo_visibility_flags(void);
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known);


#endif /* SWIFTNAV_SEARCH_MANAGER_UTILS_H */
