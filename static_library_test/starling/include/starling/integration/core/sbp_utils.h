/*
 * Copyright (C) 2014, 2016 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble "fergus@swift-nav.com"
 *          Pasi Miettinen "pasi.miettinen@exafore.com"
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_INTEGRATION_CORE_SBP_UTILS_H
#define STARLING_INTEGRATION_CORE_SBP_UTILS_H

#include <libsbp/sbp.h>

namespace starling {
namespace integration {

constexpr u16 MSG_ROVER_ID = 51228;
constexpr u16 MSG_BASE_ID = 0;

/** Value specifying the size of the SBP framing */
constexpr u8 SBP_FRAMING_SIZE_BYTES = SBP_HEADER_LEN + SBP_CRC_LEN;

}  // namespace integration
}  // namespace starling

#endif /* STARLING_INTEGRATION_CORE_SBP_UTILS_H */
