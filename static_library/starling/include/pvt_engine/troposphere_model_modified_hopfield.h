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

#ifndef LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_MODIFIED_HOPFIELD_H
#define LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_MODIFIED_HOPFIELD_H

#include <pvt_engine/troposphere_model_interface.h>
#include <swiftnav/troposphere.h>

namespace pvt_engine {

class TroposphereModelModifiedHopfield : public TroposphereModelInterface {
 public:
  explicit TroposphereModelModifiedHopfield();

  double calc_delay(const gps_time_t *t_gps, const double llh[3],
                    double azimuth, double elevation) override;

  void set_atmospheric_parameters(double pressure, double temperature,
                                  double relative_humidity);

  TROPO_MODEL get_model_type() override { return MODIFIED_HOPFIELD; };

 private:
  // P and T are taken from ICAO International Standard Atmosphere
  const double kStdPressure = 1013.25;       // mbar
  const double kStdTemperature = 15.0;       // deg C
  const double kStdRelativeHumidity = 50.0;  // %

  double pressure_;
  double temperature_;
  double relative_humidity_;

  void adjust_atmosphere_for_height(double height, double *pressure,
                                    double *temperature);

  double calc_troposphere_modified_hopfield(double elevation, double pressure,
                                            double temperature,
                                            double relative_humidity);

  double modified_hopfield_mapping_function(double zenith_angle, double height);
};

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_MODIFIED_HOPFIELD_H
