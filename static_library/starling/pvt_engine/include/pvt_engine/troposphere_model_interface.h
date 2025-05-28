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

#ifndef LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_INTERFACE_H
#define LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_INTERFACE_H

#include <pvt_engine/pvt_types.h>
#include <swiftnav/gnss_time.h>

namespace pvt_engine {

/** \defgroup troposphere_model_interface TroposphereModelInterface
 * Defines the abstract interface for a troposphere model.
 * \{ */

class TroposphereModelInterface {
 public:
  virtual ~TroposphereModelInterface() = default;

  /**
   * Calculates the tropospheric delay for the given rover and satellite
   * postition.
   *
   * Note: as Piksi doesn't have a geoid the height can be approximated using
   * the ellipsoidal height.
   *
   * @param t_gps GPS time of the observation
   * @param llh the rover latitude, longitude, and height above mean sea level
   * [rad, rad, m]
   * @param azimuth the satellite azimuth [rad]
   * @param elevation the satellite elevation [rad]
   * @return PRC the tropospheric delay [m]
   */
  virtual double calc_delay(const gps_time_t *t_gps, const double llh[3],
                            double azimuth, double elevation) = 0;

  virtual TROPO_MODEL get_model_type() = 0;
};
// \}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_INTERFACE_H
