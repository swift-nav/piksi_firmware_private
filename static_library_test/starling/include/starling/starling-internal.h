/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_INTERNAL_H
#define STARLING_INTERNAL_H

#include "pvt_engine/firmware_binding.h"

pvt_engine::PVT_ENGINE_INTERFACE_RC update_tm_filter(
    const obss_t *reference_meass, obss_t *rover_channel_meass,
    pvt_engine::FilterManagerRTK *filter_manager,
    pvt_engine::FilterObservationIdSet *obs_to_drop,
    bool *reset_downstream_filter);

#endif  // STARLING_INTERNAL_H
