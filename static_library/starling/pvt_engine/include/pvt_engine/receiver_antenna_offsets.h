/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_RECEIVER_ANTENNA_OFFSETS_H
#define STARLING_RECEIVER_ANTENNA_OFFSETS_H

#include <pvt_engine/RTKLib_apriori_models/rtklib_common_antenna.h>

namespace pvt_engine {

namespace antennas {
extern const char *const null_antenna_name;
extern const char *const gps1000_name;
extern const char *const amotechl1l2a14_name;

// There are multiple acceptable names to identify a null
// antenna, this function checks against all of these
bool is_null_antenna_name(const char *antenna_name);
}  // namespace antennas

pcv_t<PCReceiver> get_hardcoded_receiver_pcv(const char *antenna_name);

}  // namespace pvt_engine

#endif  // STARLING_RECEIVER_ANTENNA_OFFSETS_H
