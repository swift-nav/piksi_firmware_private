/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_UNB3M_H
#define LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_UNB3M_H

#include <pvt_engine/troposphere_model_interface.h>
#include <swiftnav/troposphere.h>

namespace pvt_engine {

class TroposphereModelUNB3m : public TroposphereModelInterface {
 public:
  explicit TroposphereModelUNB3m();

  double calc_delay(const gps_time_t *t_gps, const double llh[3],
                    double azimuth, double elevation) override;

  TROPO_MODEL get_model_type() override { return UNB3M; };
};

}  // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_UNB3M_H
