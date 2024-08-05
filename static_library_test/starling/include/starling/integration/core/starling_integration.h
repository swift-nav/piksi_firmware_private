/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_INTEGRATION_CORE_STARLING_INTEGRATION_H
#define STARLING_INTEGRATION_CORE_STARLING_INTEGRATION_H

#include <starling/starling.h>

namespace starling {
namespace integration {

/**
 * Initialises and starts an instance of starling.
 *
 * Before this function is called the integration layer should
 * have been provided with callbacks so it is able to output SBP messages
 * via a call to starling_setup_sbp_out().
 *
 * Once starling is running this function returns and the integration
 * layer will be ready to accept incoming SBP messages either via
 * starling::integration::sbp_in() or direct SBP callback functions
 * which should have already been set up with the helper functions
 * starling::integration::install_*_sbp_callbacks()
 *
 * Starling can be shutdown by calling starling_stop()
 *
 * @return none
 */
void setup(starling_pvt_config_t pvt_config);

}  // namespace integration
}  // namespace starling

#endif
