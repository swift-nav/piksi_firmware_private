/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: "dev@swift-nav.com"
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_INTEGRATION_CORE_SBP_LOGGING_H
#define STARLING_INTEGRATION_CORE_SBP_LOGGING_H

namespace starling {
namespace integration {

/**
 * Here we provide a high level function for implementing the logs
 * to use the outgoing SBP connection, rather than the default
 * behavior of "printf to stdout."
 *
 * You should call this during program initialization ( i.e. main() ).
 * Subsequently, all logs will be sent over the SBP connection.
 */
void enable_sbp_logging(void);

}  // namespace integration
}  // namespace starling

#endif
