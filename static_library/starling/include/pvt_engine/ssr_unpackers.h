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

#ifndef STARLING_SSR_UNPACKERS_H
#define STARLING_SSR_UNPACKERS_H

#include <libsbp/ssr.h>
#include <pvt_engine/ssr_corrections.h>
#include <swiftnav/signal.h>

namespace pvt_engine {

void unpack_ssr_orbit_clock_content(const msg_ssr_orbit_clock_t &msg,
                                    gnss_signal_t *sid,
                                    OrbitCorrection *orbit_correction,
                                    ClockCorrection *clock_correction);

void unpack_ssr_code_biases_content(const msg_ssr_code_biases_t &msg,
                                    const u8 &len, gnss_signal_t *sid,
                                    CodeBiases *code_biases);

void unpack_ssr_phase_biases_content(const msg_ssr_phase_biases_t &msg,
                                     const u8 &len, gnss_signal_t *sid,
                                     PhaseBiases *phase_biases);

void unpack_ssr_stec_correction_content(
    const msg_ssr_stec_correction_t &msg, const u8 &len,
    StecPolynomialCorrections *stec_corrections);

void unpack_ssr_gridded_atmo_content(const msg_ssr_gridded_correction_t &msg,
                                     const u8 &len, GriddedAtmo *gridded_atmo);

void unpack_ssr_gridded_atmo_content(
    const msg_ssr_gridded_correction_no_std_t &msg, const u8 &len,
    GriddedAtmo *gridded_atmo);

SsrGrid unpack_ssr_grid_definition_content(const msg_ssr_grid_definition_t &msg,
                                           const u8 &len);

}  // namespace pvt_engine

#endif
