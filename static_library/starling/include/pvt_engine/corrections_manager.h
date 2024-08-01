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

#ifndef PVT_ENGINE_CORRECTIONS_MANAGER_H
#define PVT_ENGINE_CORRECTIONS_MANAGER_H

#include <pvt_engine/configuration.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sbas/sbas.h>
#include <pvt_engine/sbas/sbas_corrections_manager.h>
#include <pvt_engine/ssr_corrections.h>

namespace pvt_engine {

class ReplayCorrectionsManager {
 public:
  ReplayCorrectionsManager();

  void initialize();
  PRC set_configuration(
      const IgsDerivedBiasesConfiguration &config,
      const pvt_engine::OffsetBiasesConfiguration &config_offset =
          pvt_engine::OffsetBiasesConfiguration());
  bool update_config(
      const IgsDerivedBiasesConfiguration &config,
      const pvt_engine::OffsetBiasesConfiguration &config_offset);

  optional<ephemeris_t> get_ephemeris_by_iode(const gnss_signal_t &sid,
                                              const u32 &desired_iode) const;

  optional<ephemeris_t> get_ephemeris(const gnss_signal_t &sid) const;

  // todo: (martin) ORI-819, replace with StaticVector
  s16 get_all_ephemerides(const ephemeris_t *stored_ephs[]) const;

  void store_new_ephemeris(const ephemeris_t &new_ephemeris);
  void store_new_ssr_orbit(const gnss_signal_t &sid,
                           const OrbitCorrection &orbit_correction);
  void store_new_ssr_clock(const gnss_signal_t &sid,
                           const ClockCorrection &clock_correction);
  void store_new_ssr_code_biases(const gnss_signal_t &sid,
                                 const CodeBiases &code_biases);
  void store_new_ssr_phase_biases(const gnss_signal_t &sid,
                                  const PhaseBiases &phase_biases);
  void store_new_ssr_stec_corrections(
      const StecPolynomialCorrections &stec_corrections);

  void store_new_ssr_gridded_atmo(const GriddedAtmo &gridded_atmo);

  void store_new_ssr_grid_definition(const SsrGrid &grid_def);

  SatelliteCorrectionsHandler get_all_corrections();

  AtmosphericCorrectionsHandler get_all_atmospheric_corrections() const;

  PRC process_sbas_message(const SBASRawData &message);

  const SBASCorrectionsManager &get_sbas_corrections_manager() const;

 private:
  SatelliteCorrectionsHandler satellite_corrections_handler_;
  AtmosphericCorrectionsHandler atmospheric_corrections_handler_;
  SBASCorrectionsManager sbas_corrections_manager_;
  bool enable_IGS_derived_biases_;
  bool enable_offset_code_biases_;
};

}  // namespace pvt_engine
#endif
