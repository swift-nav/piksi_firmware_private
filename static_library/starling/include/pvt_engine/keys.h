/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_KEYS_H
#define LIBSWIFTNAV_PVT_ENGINE_KEYS_H

#include <string>

namespace pvt_engine {
namespace keys {

extern const std::string NETWORK_METRIC_TYPE;
extern const std::string INSIGHT_TYPE;
extern const std::string LOG_TYPE;
extern const std::string NETWORK_FILTER_OUT_TYPE;
extern const std::string ORION_FILTER_OUT_TYPE;
extern const std::string ORION_POSITION_QUERY_TYPE;

extern const std::string GPS;
extern const std::string GLO;
extern const std::string PRN;
extern const std::string SAT;
extern const std::string CONSTELLATION;
extern const std::string AMBIGUITY;
extern const std::string IONOSPHERE;
extern const std::string IONOSPHERE_VARIANCE;
extern const std::string IONOSPHERE_MAPPING_FUNCTION;
extern const std::string SAT_ELEVATION;
extern const std::string ZENITH_TROPO_RESIDUAL;
extern const std::string NUM_IONO_OUTPUTS;

extern const std::string X;
extern const std::string Y;
extern const std::string Z;
extern const std::string VELOCITY_X;
extern const std::string VELOCITY_Y;
extern const std::string VELOCITY_Z;
extern const std::string ACCEL_X;
extern const std::string ACCEL_Y;
extern const std::string ACCEL_Z;

}  // namespace keys
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_KEYS_H
