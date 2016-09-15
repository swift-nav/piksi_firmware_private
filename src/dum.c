/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "board.h"
#include "dum.h"
#include "ndb.h"
#include "timing.h"

#include <libswiftnav/dopp_unc.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/time.h>

#include <assert.h>

/* maximum amount of cycles per satellite, u8 */
#define DUM_FAILS_MAX           3

/* how old ephemerides are considered valid, two weeks, [s] */
#define DUM_FIT_INTERVAL_VALID  (WEEK_SECS * 2)

/* TCXO induced Doppler [Hz] */
#define DUM_TCXO_ERROR          (DOPPLER_TCXO_ERROR_FACTOR * TCXO_FREQ_STAB)

typedef struct {
  u8 fails;     /* failed acquisition cycles amount */
  float radius; /* radius, the circle of probable receiver location [m] */
} dum_info_t;

/* Array to store uncertainty management info for each satellite */
static dum_info_t dum_info[NUM_SATS_GPS];

/** Estimate a satellite specific Doppler search window center and width based
 *  on given time estimate and LGF data which also includes TCXO offset and
 *  drift. Function returns the lower and upper limit of the estimated Doppler
 *  search window and the center point between these limits is the most likely
 *  Doppler value. Function will widen the search window based on the count of
 *  earlier failed acquisition tries.
 * 
 * \param[in] sid Signal id pointer
 * \param[in] t Current time estimate
 * \param[in] lgf Last Good Fix
 * \param[out] doppler_min Output window floow
 * \param[out] doppler_max Output window ceiling
 */
void dum_get_doppler_wndw(const gnss_signal_t *sid,
                          const gps_time_t *t,
                          const last_good_fix_t *lgf,
                          float *doppler_min,
                          float *doppler_max)
{
  if (!sid_valid(*sid) ||
      CONSTELLATION_GPS != sid_to_constellation(*sid)) {
    *doppler_min = DOPPLER_MIN;
    *doppler_max = DOPPLER_MAX;
    return;
  }

  if (DUM_FAILS_MAX < dum_info[sid->sat - GPS_FIRST_PRN].fails ||
      NULL == t ||
      TIME_COARSE > time_quality ||
      NULL == lgf ||
      POSITION_UNKNOWN == lgf->position_quality) {
    *doppler_min = DOPPLER_MIN;
    *doppler_max = DOPPLER_MAX;
    return;
  }

  ephemeris_t e;
  ndb_ephemeris_read(*sid, &e);

  if (!ephemeris_params_valid(e.valid, DUM_FIT_INTERVAL_VALID, &(e.toe), t)) {
    *doppler_min = DOPPLER_MIN;
    *doppler_max = DOPPLER_MAX;
    return;
  }

  if (calc_sat_doppler_wndw(&e,
                            t,
                            &lgf->position_solution,
                            dum_info[sid->sat - GPS_FIRST_PRN].fails,
                            &dum_info[sid->sat - GPS_FIRST_PRN].radius,
                            doppler_min,
                            doppler_max)) {
    *doppler_min = DOPPLER_MIN;
    *doppler_max = DOPPLER_MAX;
    return;
  }

  /* Add TCXO elements */
  *doppler_min -= DUM_TCXO_ERROR;
  *doppler_max += DUM_TCXO_ERROR;
}

/** Keep track on the SV specific acquisition cycle count. A failed try will
 *  increase the counter by one and a successful try will clear the counter.
 * 
 * \param[in] sid Signal id pointer
 * \param[in] res acquisition result, true acq success, false acq fail
 */
void dum_report_acq_result(const gnss_signal_t *sid, bool res)
{
  if (res) {
    dum_info[sid->sat - GPS_FIRST_PRN].fails = 0;
    dum_info[sid->sat - GPS_FIRST_PRN].radius = 0;
  } else {
    dum_info[sid->sat - GPS_FIRST_PRN].fails++;
    /* overflow, shouldn't happen if DUM_FAILS_MAX limit is followed */
    assert(0 != dum_info[sid->sat - GPS_FIRST_PRN].fails);
  }
}
