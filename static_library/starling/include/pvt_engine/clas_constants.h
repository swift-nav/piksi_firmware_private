/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_CLAS_CONSTANTS_H
#define STARLING_CLAS_CONSTANTS_H

#include <swiftnav/common.h>

namespace pvt_engine {

extern const double STEC_C00_MULTIPLIER;
extern const double STEC_C01_MULTIPLIER;
extern const double STEC_C10_MULTIPLIER;
extern const double STEC_C11_MULTIPLIER;

extern const u8 NUM_STEC_COEFF;
extern const double stec_poly_multipliers[];

extern const double CLAS_TROPO_SCALE_FACTOR;
extern const double CLAS_TROPO_DRY_OFFSET;
extern const double CLAS_TROPO_WET_OFFSET;
extern const double CLAS_STEC_RESIDUAL_MULTIPLIER;
extern const double CLAS_TECU_TO_METERS_GPS_L1;

extern const u8 C00;
extern const u8 C01;
extern const u8 C10;
extern const u8 C11;

}  // namespace pvt_engine

#endif
