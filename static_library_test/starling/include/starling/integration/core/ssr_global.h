/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_INTEGRATION_CORE_SSR_GLOBAL_H
#define STARLING_INTEGRATION_CORE_SSR_GLOBAL_H

#include <pvt_engine/corrections_manager.h>
#include <pvt_engine/observation_generator_clas.h>

extern pvt_engine::ObservationGeneratorClas global_observation_generator;
extern pvt_engine::ReplayCorrectionsManager global_corrections_manager;
void setup_observation_generator_config(
    const pvt_engine::ConfigurationType &config_type);

#endif  // STARLING_INTEGRATION_CORE_SSR_GLOBAL_H
