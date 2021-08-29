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

#include <legacy_starling_util/sbp/ssr_unpackers.h>

#include <legacy_starling_util/sbp/misc.h>
#include <pvt_common/containers/lookup_table.h>
#include <pvt_engine/clas_constants.h>
#include <pvt_engine/satellite_attitude.h>

namespace starling {
namespace util {
namespace sbp {
namespace legacy {

static const pvt_engine::BiasReference bias_reference;

static constexpr size_t cRtcmBdsLsnSize = 15;
static constexpr size_t cRtcmGalLsnSize = 11;
static constexpr size_t cRtcmGloLsnSize = 4;
static constexpr size_t cRtcmGpsLsnSize = 16;

// See field DF467 RTCM SSR phase biases document
static const pvt_common::containers::LookupTable<u8, code_t, cRtcmBdsLsnSize>
    rtcm_to_lsn_bds{// NOLINT
                    {0, CODE_BDS2_B1},
                    {6, CODE_BDS2_B2},
                    {12, CODE_BDS3_B5I},
                    {13, CODE_BDS3_B5Q},
                    {14, CODE_BDS3_B5X}};

// See Field DF382 from RTCM 3.3 document
static const pvt_common::containers::LookupTable<u8, code_t, cRtcmGalLsnSize>
    rtcm_to_lsn_gal{// NOLINT
                    {1, CODE_GAL_E1B}, {2, CODE_GAL_E1C}, {3, CODE_GAL_E1X},
                    {5, CODE_GAL_E5I}, {6, CODE_GAL_E5Q}, {7, CODE_GAL_E5X},
                    {8, CODE_GAL_E7I}, {9, CODE_GAL_E7Q}, {10, CODE_GAL_E7X}};

// See Field DF381 from RTCM 3.3 document
static const pvt_common::containers::LookupTable<u8, code_t, cRtcmGloLsnSize>
    rtcm_to_lsn_glo{// NOLINT
                    {0, CODE_GLO_L1OF},
                    {3, CODE_GLO_L2OF}};

// See Field DF380 from RTCM 3.3 document
static const pvt_common::containers::LookupTable<u8, code_t, cRtcmGpsLsnSize>
    rtcm_to_lsn_gps{// NOLINT
                    {0, CODE_GPS_L1CA}, {1, CODE_GPS_L1P},  {7, CODE_GPS_L2CM},
                    {8, CODE_GPS_L2CL}, {9, CODE_GPS_L2CX}, {11, CODE_GPS_L2P},
                    {14, CODE_GPS_L5I}, {15, CODE_GPS_L5Q}};

static pvt_engine::TropoPoint convert_clas_to_tropo(
    const tropospheric_delay_correction_t &tropo) {
  pvt_engine::TropoPoint converted;
  converted.dry_delay.set_mean(tropo.hydro *
                                   pvt_engine::CLAS_TROPO_SCALE_FACTOR +
                               pvt_engine::CLAS_TROPO_DRY_OFFSET);
  // all the stddev is in the wet
  converted.dry_delay.set_standard_deviation(0);
  converted.wet_delay.set_mean(tropo.wet * pvt_engine::CLAS_TROPO_SCALE_FACTOR +
                               pvt_engine::CLAS_TROPO_WET_OFFSET);
  pvt_engine::SnavUra ura(tropo.stddev);
  // tropo comes down in mm, convert to meters
  converted.wet_delay.set_standard_deviation(ura.stddev() / MM_PER_METER);
  return converted;
}

// See fields DF380 and DF381 of RTCM standard
static code_t rtcm_to_lsn_code(const gnss_signal_t &sid, const u8 rtcm_code) {
  if (code_to_constellation(sid.code) == CONSTELLATION_GPS) {
    if (rtcm_code < rtcm_to_lsn_gps.max_size() &&
        rtcm_to_lsn_gps.contains(rtcm_code)) {
      return rtcm_to_lsn_gps.at(rtcm_code);
    }
  }
  if (code_to_constellation(sid.code) == CONSTELLATION_GLO) {
    if (rtcm_code < rtcm_to_lsn_glo.max_size() &&
        rtcm_to_lsn_glo.contains(rtcm_code)) {
      return rtcm_to_lsn_glo.at(rtcm_code);
    }
  }
  if (code_to_constellation(sid.code) == CONSTELLATION_GAL) {
    if (rtcm_code < rtcm_to_lsn_gal.max_size() &&
        rtcm_to_lsn_gal.contains(rtcm_code)) {
      return rtcm_to_lsn_gal.at(rtcm_code);
    }
  }
  if (code_to_constellation(sid.code) == CONSTELLATION_BDS) {
    if (rtcm_code < rtcm_to_lsn_bds.max_size() &&
        rtcm_to_lsn_bds.contains(rtcm_code)) {
      return rtcm_to_lsn_bds.at(rtcm_code);
    }
  }
  return CODE_INVALID;
}

u16 get_ssr_interval_seconds(const u8 &index) {
  if (index >= NUM_SSR_UPDATE_INTERVALS) {
    detailed_log_warn("Invalid SSR Update Interval %u", index);
    return INVALID_SSR_UPDATE_INTERVAL;
  }

  return ssr_update_interval_values[index];
}

u8 get_rtcm_df391_index(const u16 &seconds) {
  for (u8 i = 0; i < NUM_SSR_UPDATE_INTERVALS; i++) {
    if (ssr_update_interval_values[i] == seconds) {
      return i;
    }
  }

  detailed_log_warn("No SSR Update Interval for specified period %u", seconds);
  return 0;
}

pvt_engine::PRC is_valid_ssr_update_interval(const u16 &seconds) {
  for (auto ssr_update_interval_value : ssr_update_interval_values) {
    if (ssr_update_interval_value == seconds) {
      return pvt_engine::RC_S_OK;
    }
  }

  return pvt_engine::RC_E_INVALID_VALUE;
}

template <typename StecMsgTrait>
static void unpack_ssr_stec_polynomial(
    const SsrMessageAggregator<StecMsgTrait> &stec_messages,
    pvt_engine::StecPolynomialCorrections *polynomial_corrections) {
  polynomial_corrections->iod_atmo = stec_messages.get_header().iod_atmo;

  polynomial_corrections->update_interval =
      get_ssr_interval_seconds(stec_messages.get_header().update_interval);
  polynomial_corrections->correction_time.wn =
      static_cast<s16>(stec_messages.get_header().time.wn);
  polynomial_corrections->correction_time.tow =
      static_cast<double>(stec_messages.get_header().time.tow);
  if (stec_messages.get_header().update_interval > 0) {
    polynomial_corrections->correction_time.tow +=
        static_cast<double>(polynomial_corrections->update_interval) / 2.0;
  }
  normalize_gps_time(&polynomial_corrections->correction_time);

  for (const auto &message : stec_messages.get_message_bodies()) {
    const size_t sat_count = message.sat_count;

    for (size_t i = 0; i < sat_count; ++i) {
      const stec_sat_element_t &sat_elements = message.sat_elements[i];
      const u16 sat = sat_elements.sv_id.satId;
      constellation_t constellation =
          static_cast<constellation_t>(sat_elements.sv_id.constellation);
      pvt_engine::SatIdentifier sat_id(sat, constellation);

      pvt_engine::StecPolynomial stec_polynomial;
      for (int j = 0; j < pvt_engine::NUM_STEC_COEFF; ++j) {
        stec_polynomial.coefficients[j] = static_cast<float>(
            sat_elements.stec_coeff[j] * pvt_engine::stec_poly_multipliers[j]);
      }
      stec_polynomial.iono_stddev = sat_elements.stec_quality_indicator;
      polynomial_corrections->satellite_map.insert(sat_id, stec_polynomial);
    }
  }
}

template <typename StecMsgTrait, typename GriddedAtmoMsgTrait>
static void generic_unpack_ssr_atmo(
    const SsrMessageAggregator<StecMsgTrait> &stec_messages,
    const SsrMessageAggregator<GriddedAtmoMsgTrait> &gridded_atmo_messages,
    const pvt_engine::SsrGrid &grid, pvt_engine::GriddedAtmo *gridded_atmo) {
  pvt_engine::StecPolynomialCorrections stec_polynomials;
  unpack_ssr_stec_polynomial(stec_messages, &stec_polynomials);

  const int32_t grid_point_count =
      gridded_atmo_messages.get_message_bodies().size();
  gridded_atmo->tropo.resize(grid_point_count);
  gridded_atmo->stec.resize(grid_point_count);

  for (const auto &message : gridded_atmo_messages.get_message_bodies()) {
    const uint16_t grid_index = message.index;
    assert(grid_index < grid_point_count);

    gridded_atmo->tropo[grid_index] =
        convert_clas_to_tropo(message.tropo_delay_correction);
    pvt_engine::StecResidualList stec_residuals;
    const size_t residual_count = message.residual_count;

    for (size_t i = 0; i < residual_count; ++i) {
      const stec_residual_t &sbp_residual = message.stec_residuals[i];

      const u16 sat = sbp_residual.sv_id.satId;
      constellation_t constellation =
          static_cast<constellation_t>(sbp_residual.sv_id.constellation);
      pvt_engine::SatIdentifier sat_id(sat, constellation);
      const double tecu =
          sbp_residual.residual * pvt_engine::CLAS_STEC_RESIDUAL_MULTIPLIER;
      const pvt_engine::SnavUra ura(sbp_residual.stddev);
      stec_residuals.satid_tecu_map.insert(
          sat_id,
          // stddev (only) is sent down in deci-tecu
          pvt_engine::MeanAndVariance(
              tecu, (ura.stddev() / DECI) * (ura.stddev() / DECI)));
    }

    gridded_atmo->stec[grid_index] = evaluate_stec_polynomials_and_residuals(
        grid, grid.index_to_lat_lon(grid_index), stec_polynomials.satellite_map,
        stec_residuals);
  }
}

void unpack_ssr_gridded_atmo_content(
    const SsrStecMessageAggregator &stec_messages,
    const SsrGriddedCorrectionMessageAggregator &gridded_atmo_messages,
    const pvt_engine::SsrGrid &grid, pvt_engine::GriddedAtmo *gridded_atmo) {
  assert(stec_messages.is_complete());
  assert(gridded_atmo_messages.is_complete());

  assert(stec_messages.get_header().iod_atmo ==
         gridded_atmo_messages.get_header().iod_atmo);
  assert(stec_messages.get_header().time.wn ==
         gridded_atmo_messages.get_header().time.wn);
  assert(stec_messages.get_header().time.tow ==
         gridded_atmo_messages.get_header().time.tow);
  assert(stec_messages.get_header().tile_id == *grid.tile_id());
  assert(stec_messages.get_header().tile_set_id == *grid.tile_set_id());
  assert(gridded_atmo_messages.get_header().tile_id == *grid.tile_id());
  assert(gridded_atmo_messages.get_header().tile_set_id == *grid.tile_set_id());

  // We expect one gridded atmo message for each grid point
  assert(gridded_atmo_messages.get_message_bodies().size() ==
         gridded_atmo_messages.get_received_message_count());

  gridded_atmo->has_std = true;
  gridded_atmo->iod_atmo = stec_messages.get_header().iod_atmo;
  gridded_atmo->correction_time.wn =
      static_cast<s16>(stec_messages.get_header().time.wn);
  gridded_atmo->correction_time.tow =
      static_cast<double>(stec_messages.get_header().time.tow);
  normalize_gps_time(&gridded_atmo->correction_time);
  gridded_atmo->tile_id = stec_messages.get_header().tile_id;
  gridded_atmo->tile_set_id = stec_messages.get_header().tile_set_id;

  generic_unpack_ssr_atmo(stec_messages, gridded_atmo_messages, grid,
                          gridded_atmo);
}

pvt_engine::SsrGrid unpack_ssr_tile_definition_content(
    const msg_ssr_tile_definition_t &msg) {
  const double latitude_coefficient = 90.0 / std::pow(2, 14);
  const double latitude = msg.corner_nw_lat * latitude_coefficient;
  const double longitude_coefficient = 180.0 / std::pow(2, 15);
  const double longitude = msg.corner_nw_lon * longitude_coefficient;
  const double latitude_spacing = msg.spacing_lat * 0.01;
  const double longitude_spacing = msg.spacing_lon * 0.01;
  const uint16_t rows = msg.rows;
  const uint16_t cols = msg.cols;
  const uint32_t tile_id = msg.tile_id;
  const uint32_t tile_set_id = msg.tile_set_id;

  return pvt_engine::SsrGrid({latitude, longitude}, longitude_spacing,
                             latitude_spacing, rows, cols, tile_set_id,
                             tile_id);
}

void unpack_ssr_orbit_clock_content(
    const msg_ssr_orbit_clock_t &msg, gnss_signal_t *sid,
    pvt_engine::OrbitCorrection *orbit_correction,
    pvt_engine::ClockCorrection *clock_correction) {
  *sid = sid_from_sbp(msg.sid);

  // Reference time is computed from the GNSS epoch time plus half the SSR
  // Update Interval
  orbit_correction->update_interval =
      get_ssr_interval_seconds(msg.update_interval);
  orbit_correction->correction_time.wn = static_cast<s16>(msg.time.wn);
  orbit_correction->correction_time.tow = static_cast<double>(msg.time.tow);
  // The reference time for the polynomial terms is computed from the GNSS epoch
  // time (GPS: DF385, GLONASS: DF386) plus half the SSR Update Interval
  // (DF391). Exception is SSR Update Interval "0", which uses the Epoch time as
  // reference time.
  if (msg.update_interval > 0) {
    orbit_correction->correction_time.tow +=
        static_cast<double>(orbit_correction->update_interval) / 2.0;
  }
  normalize_gps_time(&orbit_correction->correction_time);

  orbit_correction->iode = msg.iod;
  orbit_correction->iod_ssr = msg.iod_ssr;

  orbit_correction->rao << (static_cast<double>(msg.radial) *
                            ORBIT_RADIAL_MULTIPLIER),
      (static_cast<double>(msg.along) * ORBIT_ALONG_MULTIPLIER),
      (static_cast<double>(msg.cross) * ORBIT_CROSS_MULTIPLIER);
  orbit_correction->dot_rao
      << static_cast<double>(msg.dot_radial) * ORBIT_DOT_RADIAL_MULTIPLIER,
      static_cast<double>(msg.dot_along) * ORBIT_DOT_ALONG_MULTIPLIER,
      static_cast<double>(msg.dot_cross) * ORBIT_DOT_CROSS_MULTIPLIER;

  clock_correction->iode = msg.iod;
  clock_correction->iod_ssr = msg.iod_ssr;
  clock_correction->update_interval = orbit_correction->update_interval;
  clock_correction->correction_time = orbit_correction->correction_time;
  clock_correction->c0 = static_cast<double>(msg.c0) * CLOCK_C0_MULTIPLIER;
  clock_correction->c1 = static_cast<double>(msg.c1) * CLOCK_C1_MULTIPLIER;
  clock_correction->c2 = static_cast<double>(msg.c2) * CLOCK_C2_MULTIPLIER;
}

void unpack_ssr_code_biases_content(const msg_ssr_code_biases_t &msg,
                                    const u8 &len, gnss_signal_t *sid,
                                    pvt_engine::CodeBiases *code_biases) {
  *sid = sid_from_sbp(msg.sid);

  code_biases->iod_ssr = msg.iod_ssr;

  code_biases->update_interval = get_ssr_interval_seconds(msg.update_interval);
  code_biases->correction_time.wn = static_cast<s16>(msg.time.wn);
  code_biases->correction_time.tow = static_cast<double>(msg.time.tow);
  if (msg.update_interval > 0) {
    code_biases->correction_time.tow +=
        static_cast<double>(code_biases->update_interval) / 2.0;
  }
  normalize_gps_time(&code_biases->correction_time);

  const u8 code_header_size = sizeof(msg_ssr_code_biases_t);
/* Calculate the number of code biases in this message by looking at the SBP
 * `len` field. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  const u8 biases_in_msg =
      (len - code_header_size) / sizeof(code_biases_content_t);
#pragma GCC diagnostic pop
  for (u8 i = 0; i < biases_in_msg; i++) {
    const code_t bias_code = rtcm_to_lsn_code(*sid, msg.biases[i].code);
    if (bias_code != CODE_INVALID) {
      code_biases->biases.insert(
          bias_code,
          static_cast<float>(msg.biases[i].value * CODE_BIASES_MULTIPLIER));
    }
  }
  code_biases->collapse_code_biases();
}

void unpack_ssr_phase_biases_content(const msg_ssr_phase_biases_t &msg,
                                     const u8 &len, gnss_signal_t *sid,
                                     pvt_engine::PhaseBiases *phase_biases) {
  *sid = sid_from_sbp(msg.sid);

  phase_biases->iod_ssr = msg.iod_ssr;

  phase_biases->update_interval = get_ssr_interval_seconds(msg.update_interval);
  phase_biases->correction_time.wn = static_cast<s16>(msg.time.wn);
  phase_biases->correction_time.tow = static_cast<double>(msg.time.tow);
  if (msg.update_interval > 0) {
    phase_biases->correction_time.tow +=
        static_cast<double>(phase_biases->update_interval) / 2.0;
  }
  normalize_gps_time(&phase_biases->correction_time);

  phase_biases->dispersive = static_cast<bool>(msg.dispersive_bias);
  phase_biases->mw_consistency = static_cast<bool>(msg.mw_consistency);
  // RTCM yaw is in semi-circles
  phase_biases->yaw_angle_radians =
      static_cast<double>(msg.yaw / YAW_MULTIPLIER / 2) * M_PI * 2;

  const u8 phase_header_size = sizeof(msg_ssr_phase_biases_t);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  const u8 biases_in_msg =
      (len - phase_header_size) / sizeof(phase_biases_content_t);
#pragma GCC diagnostic pop

  for (u8 i = 0; i < biases_in_msg; i++) {
    const code_t bias_code =
        bias_reference.rtcm_signal_to_lsn_code_t(*sid, msg.biases[i].code);
    if (bias_code != CODE_INVALID) {
      pvt_engine::PhaseBias phase_bias;
      phase_bias.value =
          static_cast<float>(msg.biases[i].bias * PHASE_BIASES_MULTIPLIER);
      phase_bias.discontinuity_value = msg.biases[i].discontinuity_counter;
      phase_bias.wl_int_indicator = msg.biases[i].widelane_integer_indicator;
      phase_bias.int_indicator =
          static_cast<bool>(msg.biases[i].integer_indicator);
      phase_biases->biases.insert(bias_code, phase_bias);
    }
  }
}

void unpack_ssr_stec_correction_content(
    const msg_ssr_stec_correction_t &msg, const u8 &len,
    pvt_engine::StecPolynomialCorrections *stec_corrections) {
  stec_corrections->iod_atmo = msg.header.iod_atmo;

  stec_corrections->update_interval =
      get_ssr_interval_seconds(msg.header.update_interval);
  stec_corrections->correction_time.wn = static_cast<s16>(msg.header.time.wn);
  stec_corrections->correction_time.tow =
      static_cast<double>(msg.header.time.tow);
  if (msg.header.update_interval > 0) {
    stec_corrections->correction_time.tow +=
        static_cast<double>(stec_corrections->update_interval) / 2.0;
  }
  normalize_gps_time(&stec_corrections->correction_time);

  const u8 stec_header_size = sizeof(stec_header_t);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  const u8 elements_in_msg =
      (len - stec_header_size) / sizeof(stec_sat_element_t);
#pragma GCC diagnostic pop

  for (u8 i = 0; i < elements_in_msg; i++) {
    const u16 sat = msg.stec_sat_list[i].sv_id.satId;
    constellation_t constellation =
        static_cast<constellation_t>(msg.stec_sat_list[i].sv_id.constellation);
    pvt_engine::SatIdentifier sat_id(sat, constellation);

    pvt_engine::StecPolynomial stec_polynomial;
    for (int j = 0; j < pvt_engine::NUM_STEC_COEFF; ++j) {
      stec_polynomial.coefficients[j] =
          static_cast<float>(msg.stec_sat_list[i].stec_coeff[j] *
                             pvt_engine::stec_poly_multipliers[j]);
    }
    stec_polynomial.iono_stddev = msg.stec_sat_list[i].stec_quality_indicator;
    stec_corrections->satellite_map.insert(sat_id, stec_polynomial);
  }
}

bool ssr_headers_match(const stec_header_t &stec_header,
                       const gridded_correction_header_t &gridded_header,
                       const pvt_engine::SsrGrid &tile) {
  const bool iod_matches = stec_header.iod_atmo == gridded_header.iod_atmo;

  const bool time_matches = stec_header.time.wn == gridded_header.time.wn &&
                            stec_header.time.tow == gridded_header.time.tow;

  const bool tile_id_matches = tile.tile_id().has_value() &&
                               stec_header.tile_id == *tile.tile_id() &&
                               gridded_header.tile_id == *tile.tile_id();

  const bool tile_set_id_matches =
      tile.tile_set_id().has_value() &&
      stec_header.tile_set_id == *tile.tile_set_id() &&
      gridded_header.tile_set_id == *tile.tile_set_id();

  return iod_matches && time_matches && tile_id_matches && tile_set_id_matches;
}

static const u8 SATELLITE_TYPE_MASK = 0x1F;

static char get_constellation_char(constellation_t constellation) {
  switch (constellation) {
    case CONSTELLATION_GPS:
      return 'G';
    case CONSTELLATION_GLO:
      return 'R';
    case CONSTELLATION_BDS:
      return 'C';
    case CONSTELLATION_QZS:
      return 'J';
    case CONSTELLATION_GAL:
      return 'E';
    case CONSTELLATION_SBAS:
    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      return '\0';
  }
}

static char single_char_int(int value) {
  assert(value >= 0 && value < 10);
  switch (value) {
    case 0:
      return '0';
    case 1:
      return '1';
    case 2:
      return '2';
    case 3:
      return '3';
    case 4:
      return '4';
    case 5:
      return '5';
    case 6:
      return '6';
    case 7:
      return '7';
    case 8:
      return '8';
    case 9:
      return '9';
    default:
      return 'X';
  }
}

// This function is an unfortunate neccesity. The SVN value is a 16-bit integer
// and some compilers see a potential for a stirng overflow when packing it
// into the 5 byte long buffer. This function sidesteps that concern.
static void format_svn(constellation_t constellation, uint16_t vehicle_number,
                       char (*svn)[5]) {
  const std::div_t ones_result = std::div(vehicle_number, 10);
  const std::div_t tens_result = std::div(ones_result.quot, 10);
  const std::div_t hundreds_result = std::div(tens_result.quot, 10);

  snprintf(*svn, sizeof(*svn), "%c%c%c%c",
           get_constellation_char(constellation),
           single_char_int(ones_result.rem), single_char_int(tens_result.rem),
           single_char_int(hundreds_result.rem));
}

static pvt_engine::PhaseCenter<pvt_engine::MAX_AZIMUTH_SATELLITE>
make_satellite_phase_center(const satellite_apc_t &apc) {
  pvt_engine::PhaseCenter<pvt_engine::MAX_AZIMUTH_SATELLITE> phase_center;

  double offset[3];
  for (int i = 0; i < 3; ++i) {
    offset[i] = static_cast<double>(apc.pco[i]) / 1000.0;
  }
  phase_center.set_offset(offset);

  double var[pvt_engine::MAX_ELEVATION_BINS];
  for (int i = 0; i < pvt_engine::MAX_ELEVATION_BINS; ++i) {
    var[i] = static_cast<double>(apc.pcv[i]) / 1000.0;
  }
  phase_center.set_variation_elevation(0, var);

  return phase_center;
}

void unpack_ssr_satellite_apc_content(const msg_ssr_satellite_apc_t &msg,
                                      u8 len,
                                      pvt_engine::SatellitePCVMap *sat_apc) {
  assert(sat_apc != nullptr);
  const u8 number_of_elements = len / sizeof(satellite_apc_t);

  for (u8 i = 0; i < number_of_elements; ++i) {
    const satellite_apc_t &apc = msg.apc[i];

    const gnss_signal_t sid = sid_from_sbp(apc.sid);
    const int32_t sat_number = pvt_engine::sid2no(sid);
    const pvt_engine::PCVFrequencyType freq =
        pvt_engine::code2pcvfreq(sid.code);
    const pvt_engine::SatelliteType satellite_type =
        pvt_engine::get_satellite_type(apc.sat_info & SATELLITE_TYPE_MASK);

    optional<pvt_engine::pcv_t<pvt_engine::PCSatellite> &> sat_pcv_lookup =
        sat_apc->lookup(sat_number);

    if (!sat_pcv_lookup.has_value()) {
      pvt_engine::pcv_t<pvt_engine::PCSatellite> new_pcv;
      new_pcv.sat = sat_number;
      new_pcv.nb_azi = pvt_engine::MAX_AZIMUTH_SATELLITE;
      strncpy(new_pcv.type, satellite_type_to_str(satellite_type),
              sizeof(new_pcv.type) - 1);
      new_pcv.code[0] = '\0';
      format_svn(sid_to_constellation(sid), apc.svn, &new_pcv.svn);
      new_pcv.start_time = GPS_TIME_UNKNOWN;
      new_pcv.end_time = GPS_TIME_UNKNOWN;
      new_pcv.phase_center.insert(freq, make_satellite_phase_center(apc));
      sat_apc->insert(sat_number, new_pcv);
    } else {
      assert(!sat_pcv_lookup->phase_center.lookup(freq));
      sat_pcv_lookup->phase_center.insert(freq,
                                          make_satellite_phase_center(apc));
    }
  }
}

}  // namespace legacy
}  // namespace sbp
}  // namespace util
}  // namespace starling
