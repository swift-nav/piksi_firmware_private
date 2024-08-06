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

#ifndef ORION_DUAL_FREQUENCY_GNSS_H
#define ORION_DUAL_FREQUENCY_GNSS_H

namespace orion {
namespace dual_frequency_gnss {
/**
 * Basic methods for manipulating and combining dual frequency GNSS signals
 * using types that support basic arithmetic operations
 */

template <typename T>
T compute_widelane_wavelength(const T &lambda1, const T &lambda2) {
  return 1 / (1 / lambda1 - 1 / lambda2);
}

template <typename T>
T compute_narrowlane_wavelength(const T &lambda1, const T &lambda2) {
  return 1 / (1 / lambda1 + 1 / lambda2);
}

template <typename T, typename V>
V compute_widelane_signal(const V &signal1, const V &signal2, const T &lambda1,
                          const T &lambda2) {
  T lambda_wl = compute_widelane_wavelength(lambda1, lambda2);
  V phi_wl = (signal1 / lambda1) - (signal2 / lambda2);

  return phi_wl * lambda_wl;
}

template <typename T, typename V>
V compute_narrowlane_signal(const V &signal1, const V &signal2,
                            const T &lambda1, const T &lambda2) {
  T lambda_nl = compute_narrowlane_wavelength(lambda1, lambda2);
  V rho_nl = (signal1 / lambda1) + (signal2 / lambda2);

  return rho_nl * lambda_nl;
}

template <typename T, typename V>
V compute_mw_combination(const V &carrier1, const V &carrier2, const V &code1,
                         const V &code2, const T &lambda1, const T &lambda2) {
  V rho_nl = compute_narrowlane_signal(code1, code2, lambda1, lambda2);
  V phi_wl = compute_widelane_signal(carrier1, carrier2, lambda1, lambda2);

  return phi_wl - rho_nl;
}

template <typename T, typename V>
V invert_mw_combination(const V &carrier1, const V &code1, const V &bw,
                        const V &delta_code, const T &lambda1,
                        const T &lambda2) {
  T lambda_nl = compute_narrowlane_wavelength(lambda1, lambda2);
  T lambda_wl = compute_widelane_wavelength(lambda1, lambda2);

  V a = static_cast<V>(lambda_wl / lambda1) * carrier1;
  V b = static_cast<V>(lambda_nl / lambda1) * code1;
  V c = static_cast<V>(lambda_nl / lambda2) * delta_code;
  V d = static_cast<V>(lambda_nl / lambda2) * code1;
  V carrier2 = (a - b - c - d - bw) * static_cast<V>(lambda2 / lambda_wl);

  return carrier2;
}
}  // namespace dual_frequency_gnss
}  // namespace orion

#endif  // ORION_DUAL_FREQUENCY_GNSS_H
