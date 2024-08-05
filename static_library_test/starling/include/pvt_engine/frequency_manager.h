/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_FREQUENCY_MANAGER_H
#define LIBSWIFTNAV_PVT_ENGINE_FREQUENCY_MANAGER_H

#include <pvt_engine/gnss_constants.h>
#include <pvt_engine/pvt_return_codes.h>
#include <pvt_engine/pvt_types.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

class FrequencyManager {
 public:
  static FrequencyManager &instance();

  PRC get_center_frequency(const constellation_t &constel,
                           const FREQUENCY &freq_enum, double *frequency) const;
  PRC get_frequency(const gnss_signal_t &sid, double *frequency) const;
  PRC get_wavelength(const gnss_signal_t &sid, double *wavelength) const;
  PRC get_frequency_enum(const gnss_signal_t &sid, FREQUENCY *frequency) const;
  PRC get_l1l2_ionofree_coeffs(const gnss_signal_t &sid, double *alpha,
                               double *beta) const;
  PRC compute_l1l2_ionofree_combination(const gnss_signal_t &sid,
                                        const double &L1_obs_meters,
                                        const double &L2_obs_meters,
                                        double *iono_free_obs) const;
  PRC get_l1l2_widelane_coeffs(const gnss_signal_t &sid, double *alpha,
                               double *beta) const;
  PRC compute_l1l2_widelane_combination(const gnss_signal_t &sid,
                                        const double &L1_obs_meters,
                                        const double &L2_obs_meters,
                                        double *widelane_obs) const;

 private:
  FrequencyManager() = default;
  FrequencyManager(const FrequencyManager &) = delete;
  FrequencyManager &operator=(const FrequencyManager &) = delete;

  static FrequencyManager instance_;
};

}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_FREQUENCY_MANAGER_H
