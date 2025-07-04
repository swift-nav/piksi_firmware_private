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
#include "utils/signal_db/signal_db.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

u16 sm_get_visibility_flags(me_gnss_signal_t mesid, bool *visible, bool *known);

u16 sm_mesid_to_sat(me_gnss_signal_t mesid);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SWIFTNAV_SEARCH_MANAGER_UTILS_H */
