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

#include "dum.h"
#include "ndb.h"
#include "timing.h"
#include "manage.h"
#include "signal.h"

#include <libswiftnav/dopp_unc.h>

#include <assert.h>

/**
  The user position uncertainty is modeled as a sphere expanding its
  radius. The center of the sphere is LGF. Then each meter of the radius contributes
  to the final Doppler uncertainty at a fixed scaling factor of
  #DUM_DIST_UNC_FACTOR Hz/m.
  Then the user's most probable location for the purpose of
  acquisition is considered to be within this sphere. [m]
*/

/* Ephemerides fit interval for the purpose of (re-)acq, two weeks, [s] */
#define DUM_FIT_INTERVAL_VALID  (WEEK_SECS * 2)

/** Signals search will be done in the assumption, that user most probable
    location is a sphere with LFG in the center and the radius of this size.
    Measured in [m]. */
#define DUM_LGF_VICINITY_RADIUS_M (100.f * 1000.f)

/** Doppler estimation methods */
typedef enum {
  DUM_LGF,                 /*!< LGF + #DUM_LGF_VICINITY_RADIUS_M */
  DUM_LGF_PROPAGATION,     /*!< LGF + time based Doppler uncertainty */
  DUM_METHOD_NUM           /*!< Total number of methods */
} dum_method_e;

typedef struct {
  dum_method_e next_method; /*!< Next Doppler estimation method to use */
} dum_sat_info_t;

typedef struct {
  dum_sat_info_t svs[NUM_SATS_GPS];
} dum_info_t;

static dum_info_t dum_info = {0};

/** Compute Doppler uncertainty using LGF, time and ephemeris.
 * \param[in] mesid ME signal id pointer
 * \param[in] t Current time estimate
 * \param[in] lgf Last Good Fix
 * \param[in] radius The radius of user location uncertainty [m]
 * \param[out] doppler_min Output Doppler window floor [Hz]
 * \param[out] doppler_max Output Doppler window ceiling [Hz]
 * \retval 0 Success
 * \retval -1 Failure
 */
static int get_doppler(const me_gnss_signal_t *mesid,
                       const gps_time_t *t,
                       const last_good_fix_t *lgf,
                       float radius,
                       float *doppler_min,
                       float *doppler_max)
{
  if (NULL == t ||
      TIME_COARSE > time_quality ||
      NULL == lgf ||
      POSITION_UNKNOWN == lgf->position_quality) {
    return -1;
  }

  /* TODO GLO: Handle GLO signals properly. */
  assert(!is_glo_sid(*mesid));
  ephemeris_t e;
  gnss_signal_t sid = mesid2sid(*mesid, GLO_ORBIT_SLOT_UNKNOWN);
  if (NDB_ERR_NONE != ndb_ephemeris_read(sid, &e)) {
    return -1;
  }

  e.fit_interval = DUM_FIT_INTERVAL_VALID;
  if (!ephemeris_params_valid(e.valid, e.fit_interval, &(e.toe), t)) {
    return -1;
  }

  if (0 != calc_sat_doppler_wndw(&e,
                                 t,
                                 &lgf->position_solution,
                                 radius,
                                 doppler_min,
                                 doppler_max)) {
    return -1;
  }
  return 0;
}

/** Compute Doppler uncertainty using LGF, time and ephemeris.
 * The Doppler uncertainty is computed by expanding the user
 * location uncertainty using a predicted user speed.
 * \param[in] mesid Me signal id pointer
 * \param[in] t Current time estimate
 * \param[in] lgf Last Good Fix
 * \param[in] speed The predicted user speed [m/s]
 * \param[out] doppler_min Output Doppler window floor [Hz]
 * \param[out] doppler_max Output Doppler window ceiling [Hz]
 * \retval 0 Success
 * \retval -1 Failure
 */
static int get_doppler_by_lgf_propagation(const me_gnss_signal_t *mesid,
                                          const gps_time_t *t,
                                          const last_good_fix_t *lgf,
                                          float speed,
                                          float *doppler_min,
                                          float *doppler_max)
{
  double diff_s = gpsdifftime(t, &lgf->position_solution.time);
  float radius = diff_s * speed;

  return get_doppler(mesid, t, lgf, radius, doppler_min, doppler_max);
}

/** Compute Doppler uncertainty using LGF and ephemeris.
 * The Doppler uncertainty is computed by using a fixed user
 * location uncertainty.
 * \param[in] mesid ME signal id pointer
 * \param[in] t Current time estimate
 * \param[in] lgf Last Good Fix
 * \param[out] doppler_min Output Doppler window floor [Hz]
 * \param[out] doppler_max Output Doppler window ceiling [Hz]
 * \retval 0 Success
 * \retval -1 Failure
 */
static int get_doppler_by_lgf(const me_gnss_signal_t *mesid,
                              const gps_time_t *t,
                              const last_good_fix_t *lgf,
                              float *doppler_min,
                              float *doppler_max)
{
  float radius = DUM_LGF_VICINITY_RADIUS_M;

  return get_doppler(mesid, t, lgf, radius, doppler_min, doppler_max);
}

/** Estimate a satellite specific Doppler search window center and width based
 *  on given time estimate and LGF data which also includes TCXO offset and
 *  drift. Function returns the lower and upper limit of the estimated Doppler
 *  search window and the center point between these limits is the most likely
 *  Doppler value. Function will widen the search window based on the count of
 *  earlier failed acquisition tries.
 *
 * \param[in] mesid ME signal id pointer
 * \param[in] t Current time estimate
 * \param[in] lgf Last Good Fix
 * \param[in] speed The predicted user speed [m/s]
 * \param[out] doppler_min Output window floor [Hz]
 * \param[out] doppler_max Output window ceiling [Hz]
 */
void dum_get_doppler_wndw(const me_gnss_signal_t *mesid,
                          const gps_time_t *t,
                          const last_good_fix_t *lgf,
                          float speed,
                          float *doppler_min,
                          float *doppler_max)
{
  if ((NULL == mesid) || !mesid_valid(*mesid)) {
    assert(!"Unexpected input for Doppler estimation");
    return;
  }

  if ((CONSTELLATION_GPS != mesid_to_constellation(*mesid)) ||
      (CODE_GPS_L1CA != mesid->code)) {
    assert(!"Unsupported signal for Doppler estimation");
    return;
  }

  float default_doppler_min = code_to_sv_doppler_min(mesid->code) +
                              code_to_tcxo_doppler_min(mesid->code);
  float default_doppler_max = code_to_sv_doppler_max(mesid->code) +
                              code_to_tcxo_doppler_max(mesid->code);
  int ret = -1;
  dum_method_e method;
  u32 i = mesid->sat - GPS_FIRST_PRN;
  u32 j;

  method = dum_info.svs[i].next_method;
  for (j = 0; j < DUM_METHOD_NUM && (ret != 0); j++) {
    switch (method) {
    case DUM_LGF:
      ret = get_doppler_by_lgf(mesid, t, lgf, doppler_min, doppler_max);
      break;

    case DUM_LGF_PROPAGATION:
      ret = get_doppler_by_lgf_propagation(mesid, t, lgf, speed,
                                           doppler_min,
                                           doppler_max);
      break;

    case DUM_METHOD_NUM:
    default:
      assert(!"Unexpected method ID!");
      break;
    }

    method = (method + 1) % DUM_METHOD_NUM;
  }

  dum_info.svs[i].next_method = method;

  if (-1 == ret) {
    *doppler_min = default_doppler_min;
    *doppler_max = default_doppler_max;
  } else {
    *doppler_min = MAX(*doppler_min, default_doppler_min - ACQ_FULL_CF_STEP);
    *doppler_max = MIN(*doppler_max, default_doppler_max + ACQ_FULL_CF_STEP);
  }
}
