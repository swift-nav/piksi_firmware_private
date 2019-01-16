/*
 * Copyright (C) 2016-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <string.h>

#include "cn0_est_common.h"

/** \defgroup track Tracking
 * Functions used in tracking.
 * \{ */

/** Filter coefficient for M2 an M4. */
#define CN0_MM_ALPHA (0.5f)
/** Filter coefficient for Pn. */
#define CN0_MM_PN_ALPHA (0.0001f)
/** Estimate of noise power Pn. For smoother initial CN0 output. */
#define CN0_MM_PN_INIT (100000.0f)

static float Pn_gps_l1 = CN0_MM_PN_INIT;
static float Pn_gps_l2 = CN0_MM_PN_INIT;
static float Pn_glo_l1 = CN0_MM_PN_INIT;
static float Pn_glo_l2 = CN0_MM_PN_INIT;
static float Pn_gal_l1 = CN0_MM_PN_INIT;
static float Pn_gal_l2 = CN0_MM_PN_INIT;
static float Pn_bds_l1 = CN0_MM_PN_INIT;
static float Pn_bds_l2 = CN0_MM_PN_INIT;
static float Pn_sbas_l1 = CN0_MM_PN_INIT;

/** Initialize the \f$ C / N_0 \f$ estimator state.
 *
 * Initializes Moment method \f$ C / N_0 \f$ estimator.
 *
 * The method uses the function for C/N0 computation:
 *
 * \f[
 *    \frac{C}{N_0}(n) = \frac{Pd}{Pn}
 * \f]
 * where
 * \f[
 *    Pn(n) = M2(n) - Pd(n)
 * \f]
 * where
 * \f[
 *    Pd(n) = \sqrt{2 * M2(n)^2 - M4(n)}
 * \f]
 * where
 * \f[
 *    M2(n) = \frac{1}{2}(I(n)^2 + I(n-1)^2 + Q(n)^2 + Q(n-1)^2)
 * \f]
 * \f[
 *    M4(n) = \frac{1}{2}(I(n)^4 + I(n-1)^4 + Q(n)^4 + Q(n-1)^4)
 * \f]
 *
 * \param s     The estimator state struct to initialize.
 * \param cn0_0 The initial value of \f$ C / N_0 \f$ in dBHz.
 *
 * \return None
 */
void cn0_est_mm_init(cn0_est_mm_state_t *s, float cn0_0) {
  memset(s, 0, sizeof(*s));

  s->M2 = -1.0f; /* Set negative for first iteration */
  s->M4 = -1.0f;
  s->cn0_dbhz = cn0_0;
}

/**
 * Computes \f$ C / N_0 \f$ with Moment method.
 *
 * \param s Initialized estimator object.
 * \param p Common C/N0 estimator parameters.
 * \param I In-phase signal component
 * \param Q Quadrature phase signal component.
 *
 * \return Computed \f$ C / N_0 \f$ value
 */
float cn0_est_mm_update(cn0_est_mm_state_t *s,
                        const cn0_est_params_t *p,
                        float I,
                        float Q,
                        me_gnss_signal_t mesid,
                        bool confirmed) {
  float m2 = I * I + Q * Q;
  float m4 = m2 * m2;

  if (s->M2 < 0.0f) {
    /* This is the first iteration, just initialize moments. */
    s->M2 = m2;
    s->M4 = m4;
  } else {
    s->M2 += (m2 - s->M2) * CN0_MM_ALPHA;
    s->M4 += (m4 - s->M4) * CN0_MM_ALPHA;
  }

  float tmp = 2.0f * s->M2 * s->M2 - s->M4;
  if (0.0f > tmp) {
    tmp = 0.0f;
  }

  float Pd = sqrtf(tmp);
  float Pn = s->M2 - Pd;

  float *Pn_p;
  if (CODE_GPS_L1CA == mesid.code) {
    Pn_p = &Pn_gps_l1;
  } else if (CODE_GPS_L2CM == mesid.code) {
    Pn_p = &Pn_gps_l2;
  } else if (CODE_GLO_L1OF == mesid.code) {
    Pn_p = &Pn_glo_l1;
  } else if (CODE_GLO_L2OF == mesid.code) {
    Pn_p = &Pn_glo_l2;
  } else if (CODE_GAL_E1B == mesid.code) {
    Pn_p = &Pn_gal_l1;
  } else if (CODE_GAL_E7I == mesid.code) {
    Pn_p = &Pn_gal_l2;
  } else if (CODE_BDS2_B1 == mesid.code) {
    Pn_p = &Pn_bds_l1;
  } else if (CODE_BDS2_B2 == mesid.code) {
    Pn_p = &Pn_bds_l2;
  } else if (CODE_SBAS_L1CA == mesid.code) {
    Pn_p = &Pn_sbas_l1;
  } else {
    log_error_mesid(mesid, "CN0: Unsupported code! %lf", (float)mesid.code);
    s->cn0_dbhz = 0.0f;
    return s->cn0_dbhz;
  }

  if (confirmed) {
    *Pn_p += (Pn - *Pn_p) * CN0_MM_PN_ALPHA;
  }

  float snr = m2 / *Pn_p;

  if (!isfinite(snr) || (snr <= 0.0f)) {
    /* CN0 out of limits, no updates. */
    return s->cn0_dbhz;
  }

  float snr_db = 10.0f * log10f(snr);

  /* Compute CN0 */
  float cn0_dbhz = p->log_bw + snr_db;
  if (cn0_dbhz < 0.0f) {
    cn0_dbhz = 0.0f;
  } else if (cn0_dbhz > 60.0f) {
    cn0_dbhz = 60.0f;
  }
  s->cn0_dbhz = cn0_dbhz;

  return s->cn0_dbhz;
}

/** \} */
