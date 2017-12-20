/*
 * Copyright (C) 2013, 2016-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <math.h>
#include <stdlib.h>

#ifdef __SSSE3__
#include <tmmintrin.h>
#endif

#include <libswiftnav/constants.h>
#include <libswiftnav/signal.h>

#include "correlate.h"

/** \defgroup corr Correlation
 * Correlators used for tracking.
 * \{ */

static void track_correlate(code_t code_type,
                            const s8* restrict samples,
                            const s8* restrict code,
                            double* restrict init_code_phase,
                            double code_step,
                            double* restrict init_carr_phase,
                            double carr_step,
                            double* restrict I_E,
                            double* restrict Q_E,
                            double* restrict I_P,
                            double* restrict Q_P,
                            double* restrict I_L,
                            double* restrict Q_L,
                            u32 num_samples);

/**
 * Helper to compute number of samples for correlation
 *
 * \param[in]  samples_len        Number of input samples.
 * \param[in]  chips_to_correlate Number of chips to correlate.
 * \param[in]  init_code_phase    Initial code phase.
 * \param[in]  code_step          Code step per sample.
 * \param[out] num_samples        Resulting number of samples.
 *
 * \return Resulting number of samples to process.
 */
static u32 compute_num_samples(size_t samples_len,
                               u32 chips_to_correlate,
                               double init_code_phase,
                               double code_step,
                               u32* restrict num_samples) {
  u32 result = 0;

  result = (u32)ceil((chips_to_correlate - init_code_phase) / code_step);

  if (0 == result) {
    result = (int)ceil(chips_to_correlate / code_step);
  }

  if (result > samples_len) {
    result = samples_len;
  }

  *num_samples = result;

  return result;
}

/** Perform L1C/A correlation.
 *
 * \param samples          Samples array. One byte per sample.
 * \param samples_len      Samples array size.
 * \param code             L1C/A PRN code. One byte per chip: 1023 bytes long.
 * \param chips_to_correlate Number of chips to correlate [chips].
 * \param[in,out] init_code_phase  Initial code phase [chips].
 *                         The function returns the
 *                         the last unprocessed code phase here.
 * \param code_step        Code phase increment step [chips].
 * \param[in,out] init_carr_phase  Initial carrier phase [radians].
 *                         The function returns the the last unprocessed carrier
 *                         phase here.
 * \param carr_step        Carrier phase increment step [radians].
 * \param[out] I_E         Early replica in-phase correlation component.
 * \param[out] Q_E         Early replica quadrature correlation component.
 * \param[out] I_P         Prompt replica in-phase correlation component.
 * \param[out] Q_P         Prompt replica quadrature correlation component.
 * \param[out] I_L         Late replica in-phase correlation component.
 * \param[out] Q_L         Late replica quadrature correlation component.
 * \param[out] num_samples The number of processed samples from \e samples
 *                         array.
 */
void l1_ca_track_correlate(const s8* samples,
                           size_t samples_len,
                           const s8* code,
                           u32 chips_to_correlate,
                           double* init_code_phase,
                           double code_step,
                           double* init_carr_phase,
                           double carr_step,
                           double* I_E,
                           double* Q_E,
                           double* I_P,
                           double* Q_P,
                           double* I_L,
                           double* Q_L,
                           u32* num_samples) {
  if (0 == compute_num_samples(samples_len,
                               chips_to_correlate,
                               *init_code_phase,
                               code_step,
                               num_samples)) {
    return;
  }

  track_correlate(CODE_GPS_L1CA,
                  samples,
                  code,
                  init_code_phase,
                  code_step,
                  init_carr_phase,
                  carr_step,
                  I_E,
                  Q_E,
                  I_P,
                  Q_P,
                  I_L,
                  Q_L,
                  *num_samples);
}

/** Perform L2C CM correlation.
 *
 * \param samples          Samples array. One byte per sample.
 * \param samples_len      Samples array size.
 * \param code             L2C CM PRN code. One byte per chip: 10230 bytes long.
 * \param chips_to_correlate Number of chips to correlate [chips].
 * \param[in,out] init_code_phase  Initial code phase [chips].
 *                         The function returns the
 *                         the last unprocessed code phase here.
 * \param code_step        Code phase increment step [chips].
 * \param[in,out] init_carr_phase  Initial carrier phase [radians].
 *                         The function returns the the last unprocessed carrier
 *                         phase here.
 * \param carr_step        Carrier phase increment step [radians].
 * \param[out] I_E         Early replica in-phase correlation component.
 * \param[out] Q_E         Early replica quadrature correlation component.
 * \param[out] I_P         Prompt replica in-phase correlation component.
 * \param[out] Q_P         Prompt replica quadrature correlation component.
 * \param[out] I_L         Late replica in-phase correlation component.
 * \param[out] Q_L         Late replica quadrature correlation component.
 * \param[out] num_samples The number of processed samples from \e samples
 *                         array.
 */
void l2c_cm_track_correlate(const s8* samples,
                            size_t samples_len,
                            const s8* code,
                            u32 chips_to_correlate,
                            double* init_code_phase,
                            double code_step,
                            double* init_carr_phase,
                            double carr_step,
                            double* I_E,
                            double* Q_E,
                            double* I_P,
                            double* Q_P,
                            double* I_L,
                            double* Q_L,
                            u32* num_samples) {
  if (0 == compute_num_samples(samples_len,
                               chips_to_correlate,
                               *init_code_phase,
                               code_step,
                               num_samples)) {
    return;
  }

  track_correlate(CODE_GPS_L2CM,
                  samples,
                  code,
                  init_code_phase,
                  code_step,
                  init_carr_phase,
                  carr_step,
                  I_E,
                  Q_E,
                  I_P,
                  Q_P,
                  I_L,
                  Q_L,
                  *num_samples);
}

/** Perform GLO CA correlation.
 *
 * \param samples          Samples array. One byte per sample.
 * \param samples_len      Samples array size.
 * \param code             GLO CA PRN code. One byte per chip: 511 bytes long.
 * \param chips_to_correlate Number of chips to correlate [chips].
 * \param[in,out] init_code_phase  Initial code phase [chips].
 *                         The function returns the
 *                         the last unprocessed code phase here.
 * \param code_step        Code phase increment step [chips].
 * \param[in,out] init_carr_phase  Initial carrier phase [radians].
 *                         The function returns the the last unprocessed carrier
 *                         phase here.
 * \param carr_step        Carrier phase increment step [radians].
 * \param[out] I_E         Early replica in-phase correlation component.
 * \param[out] Q_E         Early replica quadrature correlation component.
 * \param[out] I_P         Prompt replica in-phase correlation component.
 * \param[out] Q_P         Prompt replica quadrature correlation component.
 * \param[out] I_L         Late replica in-phase correlation component.
 * \param[out] Q_L         Late replica quadrature correlation component.
 * \param[out] num_samples The number of processed samples from \e samples
 *                         array.
 */
void glo_ca_track_correlate(const s8* samples,
                            size_t samples_len,
                            const s8* code,
                            u32 chips_to_correlate,
                            double* init_code_phase,
                            double code_step,
                            double* init_carr_phase,
                            double carr_step,
                            double* I_E,
                            double* Q_E,
                            double* I_P,
                            double* Q_P,
                            double* I_L,
                            double* Q_L,
                            u32* num_samples) {
  if (0 == compute_num_samples(samples_len,
                               chips_to_correlate,
                               *init_code_phase,
                               code_step,
                               num_samples)) {
    return;
  }

  track_correlate(CODE_GLO_L1OF,
                  samples,
                  code,
                  init_code_phase,
                  code_step,
                  init_carr_phase,
                  carr_step,
                  I_E,
                  Q_E,
                  I_P,
                  Q_P,
                  I_L,
                  Q_L,
                  *num_samples);
}

/** Produce L1 C/A chip for the given code phase
 *
 * \param code       L1 C/A PRN code array. One byte per sample: 1023 bytes
 * long.
 * \param code_phase Code phase of the chip to return.
 * \return Code chip.
 */
static inline s8 l1_ca_get_chip(const s8* code, double code_phase) {
  int i;
  if (code_phase < 0) {
    i = (int)(code_phase + GPS_L1CA_CHIPS_NUM);
  } else if (code_phase >= GPS_L1CA_CHIPS_NUM) {
    i = (int)(code_phase - GPS_L1CA_CHIPS_NUM);
  } else {
    i = (int)code_phase;
  }
  return code[i];
}

/** Produce a new L1 C/A code phase given current code phase and the step.
 *
 * \param code_phase Current code phase [chips].
 * \param code_step  Code phase update step [chips].
 * \return New code phase.
 */
static inline double l1_ca_get_code_phase(double code_phase, double code_step) {
  code_phase += code_step;
  if (code_phase >= GPS_L1CA_CHIPS_NUM) {
    code_phase -= GPS_L1CA_CHIPS_NUM;
  }
  return code_phase;
}

/** Produce L2C CM chip for the given code phase
 *
 * \param code       L2C CM PRN code array. One byte per sample: 10230 bytes
 * long.
 * \param code_phase Code phase of the chip to return.
 * \return Code chip.
 */
static inline s8 l2c_cm_get_chip(const s8* code, double code_phase) {
  if (code_phase < 0) {
    code_phase += GPS_L2CM_CHIPS_NUM;
  } else if (code_phase >= GPS_L2CM_CHIPS_NUM) {
    code_phase -= GPS_L2CM_CHIPS_NUM;
  }
  if ((int)code_phase & 1) {
    return 0;  // hit CL code, which is neglected by design
  }

  return -code[(int)code_phase / 2];  // otherwise return CM code
}

/** Produce L2C CL chip for the given code phase
 *
 * \param code       L2C CL PRN code array. One byte per sample: 767250 bytes
 * long.
 * \param code_phase Code phase of the chip to return.
 * \return Code chip.
 */
static inline s8 l2c_cl_get_chip(const s8* code, double code_phase) {
  if (code_phase < 0) {
    code_phase += GPS_L2CL_CHIPS_NUM;
  } else if (code_phase >= GPS_L2CL_CHIPS_NUM) {
    code_phase -= GPS_L2CL_CHIPS_NUM;
  }
  if (!((int)code_phase & 1)) {
    return 0;  // hit CM code, which is neglected by design
  }

  return -code[(int)code_phase / 2];  // otherwise return CL code
}

/** Produce a new L2C CM code phase given current code phase and the step.
 *
 * \param code_phase Current code phase [chips].
 * \param code_step  Code phase update step [chips].
 * \return New code phase.
 */
static inline double l2c_cm_get_code_phase(double code_phase,
                                           double code_step) {
  code_phase += code_step;
  if (code_phase >= GPS_L2CM_CHIPS_NUM) {
    code_phase -= GPS_L2CM_CHIPS_NUM;
  }
  return code_phase;
}

/** Produce a new L2C CL code phase given current code phase and the step.
 *
 * \param code_phase Current code phase [chips].
 * \param code_step  Code phase update step [chips].
 * \return New code phase.
 */
static inline double l2c_cl_get_code_phase(double code_phase,
                                           double code_step) {
  code_phase += code_step;
  if (code_phase >= GPS_L2CL_CHIPS_NUM) {
    code_phase -= GPS_L2CL_CHIPS_NUM;
  }
  return code_phase;
}

/** Produce GLO C/A chip for the given code phase
 *
 * \param code       GLO C/A PRN code array.
 *                   One byte per sample: 511 bytes long.
 * \param code_phase Code phase of the chip to return.
 * \return Code chip.
 */
static inline s8 glo_ca_get_chip(const s8* code, double code_phase) {
  int i;
  if (code_phase < 0)
    i = (int)(code_phase + GLO_CA_CHIPS_NUM);
  else if (code_phase >= GLO_CA_CHIPS_NUM)
    i = (int)(code_phase - GLO_CA_CHIPS_NUM);
  else
    i = (int)code_phase;
  return code[i];
}

/** Produce a new GLO C/A code phase given current code phase
 *  and the step.
 *
 * \param code_phase Current code phase [chips].
 * \param code_step  Code phase update step [chips].
 * \return New code phase.
 */
static inline double glo_ca_get_code_phase(double code_phase,
                                           double code_step) {
  code_phase += code_step;
  if (code_phase >= GLO_CA_CHIPS_NUM) code_phase -= GLO_CA_CHIPS_NUM;
  return code_phase;
}

#ifndef __SSSE3__

/** Perform correlation.
 *
 * \param code_type        Code identifier. Supported GPS L1 C/A, GPS L2 C,
 *                         GLONASS L1 C/A and GLONASS L2 C/A.
 * \param samples          Samples array. One byte per sample.
 * \param code             PRN code. One byte per chip.
 * \param[in,out] init_code_phase  Initial code phase [chips].
 *                         The function returns the last unprocessed code
 *                         phase here.
 * \param code_step        Code phase increment step [chips].
 * \param[in,out] init_carr_phase  Initial carrier phase [radians].
 *                         The function returns the the last unprocessed carrier
 *                         phase here.
 * \param carr_step        Carrier phase increment step [radians].
 * \param[out] I_E         Early replica in-phase correlation component.
 * \param[out] Q_E         Early replica quadrature correlation component.
 * \param[out] I_P         Prompt replica in-phase correlation component.
 * \param[out] Q_P         Prompt replica quadrature correlation component.
 * \param[out] I_L         Late replica in-phase correlation component.
 * \param[out] Q_L         Late replica quadrature correlation component.
 * \param num_samples      The number of samples to correlate from \e samples
 *                         array.
 */

static void track_correlate(code_t code_type,
                            const s8* restrict samples,
                            const s8* restrict code,
                            double* restrict init_code_phase,
                            double code_step,
                            double* restrict init_carr_phase,
                            double carr_step,
                            double* restrict I_E,
                            double* restrict Q_E,
                            double* restrict I_P,
                            double* restrict Q_P,
                            double* restrict I_L,
                            double* restrict Q_L,
                            u32 num_samples) {
  double code_phase = *init_code_phase;
  double carr_phase = *init_carr_phase;

  double carr_sin = sin(carr_phase);
  double carr_cos = cos(carr_phase);
  double sin_delta = sin(carr_step);
  double cos_delta = cos(carr_step);

  *I_E = *Q_E = *I_P = *Q_P = *I_L = *Q_L = 0;

  s8 code_E, code_P, code_L;
  double baseband_Q, baseband_I;
  code_E = code_P = code_L = 0;

  for (u32 i = 0; i < num_samples; i++) {
    double code_phase_new = 0;

    /* Note, l1ca_* and l2c_* functions are inline and should not
       impose much execution overhead */
    switch (code_type) {
      case CODE_GPS_L1CA:
        code_E = l1_ca_get_chip(code, code_phase - 0.5);
        code_P = l1_ca_get_chip(code, code_phase);
        code_L = l1_ca_get_chip(code, code_phase + 0.5);
        code_phase_new = l1_ca_get_code_phase(code_phase, code_step);
        break;
      case CODE_GPS_L2CM:
        code_E = l2c_cm_get_chip(code, code_phase - 0.5);
        code_P = l2c_cm_get_chip(code, code_phase);
        code_L = l2c_cm_get_chip(code, code_phase + 0.5);
        code_phase_new = l2c_cm_get_code_phase(code_phase, code_step);
        break;
      case CODE_GPS_L2CL:
        code_E = l2c_cl_get_chip(code, code_phase - 0.5);
        code_P = l2c_cl_get_chip(code, code_phase);
        code_L = l2c_cl_get_chip(code, code_phase + 0.5);
        code_phase_new = l2c_cl_get_code_phase(code_phase, code_step);
        break;
      case CODE_GLO_L1OF:
      case CODE_GLO_L2OF:
        code_E = glo_ca_get_chip(code, code_phase - 0.5);
        code_P = glo_ca_get_chip(code, code_phase);
        code_L = glo_ca_get_chip(code, code_phase + 0.5);
        code_phase_new = glo_ca_get_code_phase(code_phase, code_step);
        break;
      case CODE_INVALID:
      case CODE_COUNT:
      case CODE_SBAS_L1CA:
      case CODE_GPS_L1P:
      case CODE_GPS_L2P:
      case CODE_GPS_L2CX:
      case CODE_GPS_L5I:
      case CODE_GPS_L5Q:
      case CODE_GPS_L5X:
      case CODE_BDS2_B11:
      case CODE_BDS2_B2:
      case CODE_GAL_E1B:
      case CODE_GAL_E1C:
      case CODE_GAL_E1X:
      case CODE_GAL_E6B:
      case CODE_GAL_E6C:
      case CODE_GAL_E6X:
      case CODE_GAL_E7I:
      case CODE_GAL_E7Q:
      case CODE_GAL_E7X:
      case CODE_GAL_E8:
      case CODE_GAL_E5I:
      case CODE_GAL_E5Q:
      case CODE_GAL_E5X:
      case CODE_QZS_L1CA:
      case CODE_QZS_L2CM:
      case CODE_QZS_L2CL:
      case CODE_QZS_L2CX:
      case CODE_QZS_L5I:
      case CODE_QZS_L5Q:
      case CODE_QZS_L5X:
      default:
        assert(!"Unsupported correlator type");
        break;
    }

    baseband_Q = -carr_sin * samples[i];
    baseband_I = carr_cos * samples[i];

    /* generate a new set of sine and cosine samples using
       rotation matrix */
    double carr_sin_ = carr_sin * cos_delta + carr_cos * sin_delta;
    double carr_cos_ = carr_cos * cos_delta - carr_sin * sin_delta;
    double i_mag = (3.0 - carr_sin_ * carr_sin_ - carr_cos_ * carr_cos_) / 2.0;
    carr_sin = carr_sin_ * i_mag;
    carr_cos = carr_cos_ * i_mag;

    *I_E += code_E * baseband_I;
    *Q_E += code_E * baseband_Q;
    *I_P += code_P * baseband_I;
    *Q_P += code_P * baseband_Q;
    *I_L += code_L * baseband_I;
    *Q_L += code_L * baseband_Q;

    code_phase = code_phase_new;
  }
  *init_code_phase = code_phase;
  *init_carr_phase = fmod(*init_carr_phase + num_samples * carr_step, 2 * M_PI);
}

#else

static void track_correlate(code_t code_type,
                            const s8* restrict samples,
                            const s8* restrict code,
                            double* restrict init_code_phase,
                            double code_step,
                            double* restrict init_carr_phase,
                            double carr_step,
                            double* restrict I_E,
                            double* restrict Q_E,
                            double* restrict I_P,
                            double* restrict Q_P,
                            double* restrict I_L,
                            double* restrict Q_L,
                            u32 num_samples) {
  double code_phase = *init_code_phase;

  float carr_sin = sin(*init_carr_phase);
  float carr_cos = cos(*init_carr_phase);
  float sin_delta = sin(carr_step);
  float cos_delta = cos(carr_step);

  __m128 IE_QE_IP_QP;
  __m128 CE_CE_CP_CP;
  __m128 IL_QL_X_X;
  __m128 CL_CL_X_X;
  __m128 S_C_S_C;
  __m128 C_S_C_S;
  __m128 BI_BQ_BI_BQ;
  __m128 dC_dS_dS_dC;
  __m128 a1, a2, a3;
  __m128 one_minus_one;

  s8 code_E = 0, code_P = 0, code_L = 0;

  IE_QE_IP_QP = _mm_set_ps(0, 0, 0, 0);
  IL_QL_X_X = _mm_set_ps(0, 0, 0, 0);
  one_minus_one = _mm_set_ps(1, -1, 1, -1);
  S_C_S_C = _mm_set_ps(carr_sin, carr_cos, carr_sin, carr_cos);
  C_S_C_S = _mm_set_ps(carr_cos, -carr_sin, carr_cos, -carr_sin);
  dC_dS_dS_dC = _mm_set_ps(cos_delta, sin_delta, sin_delta, cos_delta);

  for (u32 i = 0; i < num_samples; i++) {
    double code_phase_new = 0;

    /* Note, l1ca_ and l2c_ functions are inline and should not
       impose much execution overhead */
    switch (code_type) {
      case CODE_GPS_L1CA:
        code_E = l1_ca_get_chip(code, code_phase - 0.5);
        code_P = l1_ca_get_chip(code, code_phase);
        code_L = l1_ca_get_chip(code, code_phase + 0.5);
        code_phase_new = l1_ca_get_code_phase(code_phase, code_step);
        break;
      case CODE_GPS_L2CM:
        code_E = l2c_cm_get_chip(code, code_phase - 0.5);
        code_P = l2c_cm_get_chip(code, code_phase);
        code_L = l2c_cm_get_chip(code, code_phase + 0.5);
        code_phase_new = l2c_cm_get_code_phase(code_phase, code_step);
        break;
      case CODE_GPS_L2CL:
        code_E = l2c_cl_get_chip(code, code_phase - 0.5);
        code_P = l2c_cl_get_chip(code, code_phase);
        code_L = l2c_cl_get_chip(code, code_phase + 0.5);
        code_phase_new = l2c_cl_get_code_phase(code_phase, code_step);
        break;
      case CODE_GLO_L1OF:
      case CODE_GLO_L2OF:
        code_E = glo_ca_get_chip(code, code_phase - 0.5);
        code_P = glo_ca_get_chip(code, code_phase);
        code_L = glo_ca_get_chip(code, code_phase + 0.5);
        code_phase_new = glo_ca_get_code_phase(code_phase, code_step);
        break;
      case CODE_INVALID:
      case CODE_COUNT:
      case CODE_SBAS_L1CA:
      case CODE_GPS_L1P:
      case CODE_GPS_L2P:
      case CODE_GPS_L2CX:
      case CODE_GPS_L5I:
      case CODE_GPS_L5Q:
      case CODE_GPS_L5X:
      case CODE_BDS2_B11:
      case CODE_BDS2_B2:
      case CODE_GAL_E1B:
      case CODE_GAL_E1C:
      case CODE_GAL_E1X:
      case CODE_GAL_E6B:
      case CODE_GAL_E6C:
      case CODE_GAL_E6X:
      case CODE_GAL_E7I:
      case CODE_GAL_E7Q:
      case CODE_GAL_E7X:
      case CODE_GAL_E8:
      case CODE_GAL_E5I:
      case CODE_GAL_E5Q:
      case CODE_GAL_E5X:
      case CODE_QZS_L1CA:
      case CODE_QZS_L2CM:
      case CODE_QZS_L2CL:
      case CODE_QZS_L2CX:
      case CODE_QZS_L5I:
      case CODE_QZS_L5Q:
      case CODE_QZS_L5X:
      default:
        assert(!"Unsupported correlator type");
        break;
    }

    CE_CE_CP_CP = _mm_set_ps(code_E, code_E, code_P, code_P);
    CL_CL_X_X = _mm_set_ps(code_L, code_L, 0, 0);

    /* Load sample and multiply by sin/cos carrier to mix down to baseband. */
    a1 = _mm_set1_ps((float)samples[i]);  // S, S, S, S
    BI_BQ_BI_BQ = _mm_mul_ps(a1, C_S_C_S);

    /* Update carrier sin/cos values by multiplying by the constant rotation
     * matrix corresponding to carr_step. */
    a1 = _mm_mul_ps(S_C_S_C, dC_dS_dS_dC);                 // SdC, CdS, SdS, CdC
    a2 = _mm_shuffle_ps(a1, a1, _MM_SHUFFLE(3, 0, 3, 0));  // SdC_CdC_SdC_CdC
    a3 = _mm_shuffle_ps(a1, a1, _MM_SHUFFLE(2, 1, 2, 1));  // CdS_SdS_CdS_SdS
    // S = SdC + CdS, C = CdC - SdS
    S_C_S_C = _mm_addsub_ps(a2, a3);                                 // C_S_C_S
    a1 = _mm_shuffle_ps(S_C_S_C, S_C_S_C, _MM_SHUFFLE(2, 3, 0, 1));  // C_S_C_S
    C_S_C_S = _mm_mul_ps(a1, one_minus_one);                         // C_S_C_S

    /* Multiply code and baseband signal. */
    a1 = _mm_mul_ps(CE_CE_CP_CP, BI_BQ_BI_BQ);
    a2 = _mm_mul_ps(CL_CL_X_X, BI_BQ_BI_BQ);

    /* Increment accumulators. */
    IE_QE_IP_QP = _mm_add_ps(IE_QE_IP_QP, a1);
    IL_QL_X_X = _mm_add_ps(IL_QL_X_X, a2);

    code_phase = code_phase_new;
  }
  *init_code_phase = code_phase;
  *init_carr_phase = fmod(*init_carr_phase + num_samples * carr_step, 2 * M_PI);

  float res[8];
  _mm_storeu_ps(res, IE_QE_IP_QP);
  _mm_storeu_ps(res + 4, IL_QL_X_X);

  *I_E = res[3];
  *Q_E = res[2];
  *I_P = res[1];
  *Q_P = res[0];
  *I_L = res[7];
  *Q_L = res[6];
}

#endif /* !__SSSE3__ */

/** \} */
