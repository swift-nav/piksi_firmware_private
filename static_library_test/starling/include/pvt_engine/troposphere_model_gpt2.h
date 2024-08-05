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

#ifndef LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_GPT2_H
#define LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_GPT2_H

#include <pvt_engine/troposphere_model_interface.h>

const u16 GRID_SIZE = 2592;
const u16 ORDER = 5;

extern const double pressure_grid[GRID_SIZE][ORDER];
extern const double temperature_grid[GRID_SIZE][ORDER];
extern const double humidity_grid[GRID_SIZE][ORDER];
extern const double delta_temp_grid[GRID_SIZE][ORDER];
extern const double GPT2_ah_grid[GRID_SIZE][ORDER];
extern const double GPT2_aw_grid[GRID_SIZE][ORDER];
extern const double water_vapour_lapse_grid[GRID_SIZE][ORDER];
extern const double water_vapour_temp_grid[GRID_SIZE][ORDER];
extern const double geoid_undulation_grid[GRID_SIZE];
extern const double height_grid[GRID_SIZE];

// coefficients
static const double k1 = 77.6040;                         // K/hPa
static const double k2 = 64.790;                          // K/hPa
static const double k2p = k2 - k1 * 18.01520 / 28.96440;  // K/hPa
static const double k3 = 377600.0;                        // KK/hPa

// molar mass of dry air in kg/mol
static const double molar_mass_dry_air = 28.965e-3;
// universal gas constant in J/K/mol
static const double uni_gas_constant = 8.31430;

namespace pvt_engine {

class TroposphereModelGPT2 : public TroposphereModelInterface {
 public:
  explicit TroposphereModelGPT2();

  double calc_delay(const gps_time_t *t_gps, const double llh[3],
                    double azimuth, double elevation) override;

  double get_dry_mapping_function(const gps_time_t *t_gps, const double llh[3],
                                  double azimuth, double elevation) const;

  double get_wet_mapping_function(const gps_time_t *t_gps, const double llh[3],
                                  double azimuth, double elevation) const;

  TROPO_MODEL get_model_type() override { return GPT2; };

 private:
  static void calc_params(const gps_time_t *t_gps, const double llh[3],
                          const double azimuth, const double elevation,
                          double *zhd, double *map_func_dry, double *zwd,
                          double *map_func_wet);
  static double asknewet(const double water_vapour_pressure,
                         const double temperature, const double lambda);
  static double saasthyd(const double pressure, const double lat,
                         const double hell);
  static void vmf1_ht(const double GPT2_ah, const double GPT2_aw,
                      const double mjd, const double lat, const double height,
                      const double zenith, double *vmf1h, double *vmf1w);
  static void gpt2_1w(const double mjd, const double lat, const double lon,
                      const double hell, const u16 it, double *pressure,
                      double *temperature, double *delta_temp,
                      double *water_vapour_temp, double *water_vapour_pressure,
                      double *GPT2_ah, double *GPT2_aw, double *lambda,
                      double *undulation);
};

};      // namespace pvt_engine
#endif  // LIBSWIFTNAV_PVT_ENGINE_TROPOSPHERE_MODEL_GPT2_H
