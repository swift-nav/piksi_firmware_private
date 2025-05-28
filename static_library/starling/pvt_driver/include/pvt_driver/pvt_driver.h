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

#ifndef STARLING_PVT_DRIVER_H
#define STARLING_PVT_DRIVER_H

#include <libpal++/unique_pointer.h>
#include <pvt_driver/debug.h>
#include <pvt_driver/external_functions.h>
#include <pvt_driver/solutions.h>
#include <pvt_driver/types.h>
#include <pvt_engine/process_noise_motion.h>
#include <pvt_engine/solution_frequency.h>
#include <swiftnav/glonass_phase_biases.h>
#include <swiftnav/sbas_raw_data.h>

namespace pvt_driver {

class PvtDriver {
  class PvtDriverImpl;

  pal::UniquePointer<PvtDriverImpl> impl_;

 public:
  explicit PvtDriver();
  PvtDriver(const PvtDriver &) = delete;
  PvtDriver(PvtDriver &&other) = default;

  ~PvtDriver();

  PvtDriver &operator=(const PvtDriver &) = delete;
  PvtDriver &operator=(PvtDriver &&other) = default;

  // Initializes from only ME type and frequency bands,
  // use other init() function to customize other config parameters
  pal_error init(pvt_driver_config_me_t me_config,
                 pvt_driver_insights_config_t insights_config) noexcept;

  // Initializes from a complete config structure.
  // Requires ME type and frequency bands to be set,
  // set size of array parameters to zero to use default values for them.
  pal_error init(pvt_driver_config_t config,
                 pvt_driver_insights_config_t insights_config) noexcept;

  bool start_engine() noexcept;
  void stop_engine() noexcept;

  // Data Input Functions
  obs_array_t *alloc_rover_obs() noexcept;
  obs_array_t *alloc_base_obs() noexcept;
  void free_rover_obs(obs_array_t *obs_array) noexcept;
  void free_base_obs(obs_array_t *obs_array) noexcept;
  /**
   * This function takes ownership of `obs_array` in all cases, even when
   * an error occurs.
   */
  bool send_rover_obs(obs_array_t *obs_array) noexcept;
  /**
   * This function takes ownership of `obs_array` in all cases, even when
   * an error occurs.
   */
  bool send_base_obs(obs_array_t *obs_array) noexcept;
  bool send_ephemerides(const ephemeris_t *ephemerides, size_t n) noexcept;
  bool send_sbas_data(const sbas_raw_data_t *sbas_data) noexcept;
  bool send_prior_position(
      const pvt_driver_prior_position_t *prior_position) noexcept;

  // Management functions
  void reset_all_filters() noexcept;
  void reset_time_matched_filter() noexcept;
  void reset_low_latency_filter() noexcept;
  int add_solution_handler(pvt_driver_solution_callback_t callbacks) noexcept;
  void remove_solution_handler(int id) noexcept;

  // Settings functions
  void set_is_glonass_enabled(bool is_enabled) noexcept;
  void set_is_galileo_enabled(bool is_enabled) noexcept;
  void set_is_beidou_enabled(bool is_enabled) noexcept;
  void set_is_rtk_fixed_enabled(bool is_enabled) noexcept;
  void set_is_time_matched_klobuchar_enabled(bool is_enabled) noexcept;
  void set_max_correction_age(unsigned int max_age) noexcept;
  void set_process_noise_motion(
      PROCESS_NOISE_MOTION_TYPE process_noise_settings) noexcept;
  /* Modify the relative weighting observations. */
  void set_downweight_factor(code_t code, float factor) noexcept;
  /* Set the elevation mask used to filter satellites from the solution. */
  void set_elevation_mask(float elevation_mask) noexcept;

  void set_min_modelled_baseline_len_km(double value) noexcept;
  void set_enable_gnss_only_output(bool is_enabled) noexcept;

  /* When GNSS-only output is enabled, only the GNSS engine is run. Instruct it
   * to send messages with the _GNSS decorator and its associated metadata.
   *
   * When GNSS-only output is disabled, the fusion engine is also run. The
   * fusion engine will send the metadata, so the GNSS engine only needs to send
   * messages with the _GNSS decorator. */
  bool get_enable_gnss_only_output() const noexcept;

  /* Get the max sats for the given requested frequency. */
  bool get_max_sats_from_requested_freq(double requested_frequency_hz,
                                        s32 *max_sats) noexcept;
  bool set_max_sats(s32 max_sats) noexcept;
  void set_known_ref_pos(const double base_pos[3]) noexcept;
  void set_known_glonass_biases(const glo_biases_t &biases) noexcept;
  pvt_driver_solution_mode_t get_solution_mode() const noexcept;
  void set_solution_mode(pvt_driver_solution_mode_t new_mode) noexcept;
  /* Set the antenna offset to compensate for */
  void set_antenna_offset(pvt_driver_antenna_t antenna) noexcept;
  void set_is_pl_enabled(bool is_enabled) noexcept;
  void set_solution_output_mode(const pvt_driver_solution_mode_t mode) noexcept;

  /* Set PVT solution output gating flag */
  void set_enable_output_gating(bool is_enabled) noexcept;

  void set_input_bridge_mode(pvt_driver_bridge_mode_t mode) noexcept;
  void set_external_functions(
      pvt_driver_external_functions_t functions) noexcept;
  void set_debug_functions(
      pvt_driver_debug_functions_t debug_functions) noexcept;
  void set_thread_synchronization(bool enabled) noexcept;
  void set_external_data_path(const char *path) noexcept;
};

}  // namespace pvt_driver

#endif  // STARLING_PVT_DRIVER_H
