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
#include <pvt_engine/ephemeris_handler.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/sbas/sbas.h>
#include <pvt_engine/sbas/sbas_corrections_manager.h>
#include <pvt_engine/ssr_corrections.h>

namespace pvt_engine {

/******************************************************************
 *               ReplayCorrectionsManager Interface               *
 ******************************************************************/

class ReplayCorrectionsManagerInterface {
 public:
  virtual ~ReplayCorrectionsManagerInterface() = default;

  virtual void initialize() = 0;

  virtual PRC set_configuration(
      const IgsDerivedBiasesConfiguration &config,
      const pvt_engine::OffsetBiasesConfiguration &config_offset) = 0;

  virtual bool update_config(
      const IgsDerivedBiasesConfiguration &config,
      const pvt_engine::OffsetBiasesConfiguration &config_offset) = 0;

  virtual optional<ephemeris_t> get_ephemeris_by_iode(
      const gnss_signal_t &sid, const SsrIodeKey &desired_iode) const = 0;

  virtual optional<const ephemeris_t &> get_most_recent_ephemeris(
      const pvt_engine::SatIdentifier &sat) const = 0;

  virtual s16 get_all_ephemerides(const ephemeris_t *stored_ephs[]) const = 0;

  virtual void store_new_ephemeris(const ephemeris_t &new_ephemeris) = 0;

  virtual void store_new_ssr_orbit(const gnss_signal_t &sid,
                                   const OrbitCorrection &orbit_correction) = 0;

  virtual void store_new_ssr_clock(const gnss_signal_t &sid,
                                   const ClockCorrection &clock_correction) = 0;

  virtual void store_new_ssr_code_biases(const gnss_signal_t &sid,
                                         const CodeBiases &code_biases) = 0;

  virtual void store_new_ssr_phase_biases(const gnss_signal_t &sid,
                                          const PhaseBiases &phase_biases) = 0;

  virtual void store_new_ssr_gridded_atmo(const GriddedAtmo &gridded_atmo) = 0;

  virtual void store_new_ssr_grid_definition(const SsrGrid &grid_def) = 0;

  virtual SatelliteCorrectionsHandler get_sat_corrections_handler() const = 0;

  virtual SatelliteCorrectionsHandler get_all_corrections() = 0;

  virtual const AtmosphericCorrectionsHandler &get_all_atmospheric_corrections()
      const = 0;

  virtual PRC process_sbas_message(const SBASRawData &message) = 0;

  virtual const SBASCorrectionsManager &get_sbas_corrections_manager()
      const = 0;

  virtual optional<pvt_engine::SatPVA> calc_sat_pva(
      const gnss_signal_t &sid, const gps_time_t &tot) const = 0;

  virtual EphemerisHandlerInterface &get_ephemeris_handler() = 0;

  virtual const EphemerisHandlerInterface &get_ephemeris_handler() const = 0;
};

/******************************************************************
 *              ReplayCorrectionsManager Implementation           *
 ******************************************************************/

class ReplayCorrectionsManager final
    : public ReplayCorrectionsManagerInterface {
 public:
  ReplayCorrectionsManager();

  void initialize() override;

  PRC set_configuration(
      const IgsDerivedBiasesConfiguration &config,
      const pvt_engine::OffsetBiasesConfiguration &config_offset) override;

  bool update_config(
      const IgsDerivedBiasesConfiguration &config,
      const pvt_engine::OffsetBiasesConfiguration &config_offset) override;

  optional<ephemeris_t> get_ephemeris_by_iode(
      const gnss_signal_t &sid, const SsrIodeKey &desired_iode) const override;

  optional<const ephemeris_t &> get_most_recent_ephemeris(
      const pvt_engine::SatIdentifier &sat) const override;

  // todo: (martin) ORI-819, replace with StaticVector
  void store_new_ssr_orbit(const gnss_signal_t &sid,
                           const OrbitCorrection &orbit_correction) override;

  void store_new_ephemeris(const ephemeris_t &new_ephemeris) override;

  void store_new_ssr_clock(const gnss_signal_t &sid,
                           const ClockCorrection &clock_correction) override;

  void store_new_ssr_code_biases(const gnss_signal_t &sid,
                                 const CodeBiases &code_biases) override;

  void store_new_ssr_phase_biases(const gnss_signal_t &sid,
                                  const PhaseBiases &phase_biases) override;

  void store_new_ssr_gridded_atmo(const GriddedAtmo &gridded_atmo) override;

  void store_new_ssr_grid_definition(const SsrGrid &grid_def) override;

  SatelliteCorrectionsHandler get_sat_corrections_handler() const override;

  SatelliteCorrectionsHandler get_all_corrections() override;

  const AtmosphericCorrectionsHandler &get_all_atmospheric_corrections()
      const override;

  s16 get_all_ephemerides(const ephemeris_t *stored_ephs[]) const override;

  PRC process_sbas_message(const SBASRawData &message) override;

  const SBASCorrectionsManager &get_sbas_corrections_manager() const override;

  optional<pvt_engine::SatPVA> calc_sat_pva(
      const gnss_signal_t &sid, const gps_time_t &tot) const override;

  EphemerisHandlerInterface &get_ephemeris_handler() override;

  const EphemerisHandlerInterface &get_ephemeris_handler() const override;

 private:
  SatelliteCorrectionsHandler satellite_corrections_handler_;
  EphemerisHandler ephemeris_handler_;
  AtmosphericCorrectionsHandler atmospheric_corrections_handler_;
  SBASCorrectionsManager sbas_corrections_manager_;
  bool enable_IGS_derived_biases_;
  bool enable_offset_code_biases_;
};

}  // namespace pvt_engine
#endif
