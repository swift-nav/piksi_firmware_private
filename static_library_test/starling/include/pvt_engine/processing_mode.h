/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_PROCESSING_MODE_H
#define STARLING_PROCESSING_MODE_H
#include <starling/starling.h>
#include "pvt_types.h"

pvt_engine::PROCESSING_MODE starling_to_pvt_engine_processing_mode(
    const dgnss_solution_mode_t mode);

dgnss_solution_mode_t pvt_engine_to_starling_processing_mode(
    const pvt_engine::PROCESSING_MODE mode);

#endif  // STARLING_PROCESSING_MODE_H
