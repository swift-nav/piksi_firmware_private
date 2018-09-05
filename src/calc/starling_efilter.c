/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>

#include <libswiftnav/ephemeris.h>
#include <libswiftnav/signal.h>
#include "signal_db/signal_db.h"
#include "starling_efilter.h"
#include "starling_platform_shim.h"

static MUTEX_DECL(starling_efilter_lock);

typedef struct {
  ephemeris_t prev_ephe;
  ephemeris_t cur_ephe;
} estack_t;

static estack_t estack_arr[NUM_SATS] = {0};
sbas_has_corrections_t sbas_has_corrections_ = NULL;

void starling_efilter_set_sbas_cb(sbas_has_corrections_t sbas_has_corrections) {
  assert(sbas_has_corrections);
  sbas_has_corrections_ = sbas_has_corrections;
}

static void set_ephe(const ephemeris_t *e) {
  u16 idx = sid_to_sv_index(e->sid);
  assert(idx < ARRAY_SIZE(estack_arr));
  estack_t *estack = &estack_arr[idx];

  if (estack->cur_ephe.valid) {
    estack->cur_ephe.sid = e->sid;
    if (ephemeris_equal(&estack->cur_ephe, e)) {
      return;
    }
  }
  estack->prev_ephe = estack->cur_ephe;
  estack->cur_ephe = *e;
}

void starling_efilter_set_ephe(const ephemeris_t *e) {
  assert(e);
  assert(e->valid);
  assert(sid_valid(e->sid));

  platform_mutex_lock(&starling_efilter_lock);
  set_ephe(e);
  platform_mutex_unlock(&starling_efilter_lock);
}

static u8 get_ephe_iode(const ephemeris_t *e) {
  assert(e->valid);
  switch (sid_to_constellation(e->sid)) {
    case CONSTELLATION_GPS:
    case CONSTELLATION_BDS2:
    case CONSTELLATION_GAL:
    case CONSTELLATION_QZS:
      return e->kepler.iode;
    case CONSTELLATION_SBAS:
      return 0;
    case CONSTELLATION_GLO:
      return e->glo.iod;
    case CONSTELLATION_INVALID:
    case CONSTELLATION_COUNT:
    default:
      assert(!"Unsupported constellation");
      return -1;
  }
  assert(0);
  return 0;
}

static bool sbas_has_corrections_for_ephe(const ephemeris_t *e) {
  assert(sbas_has_corrections_);
  u8 iode = get_ephe_iode(e);
  return sbas_has_corrections_(&e->sid, iode);
}

static ephemeris_t get_ephe(const gnss_signal_t *sid) {
  u16 idx = sid_to_sv_index(*sid);
  assert(idx < ARRAY_SIZE(estack_arr));
  estack_t *estack = &estack_arr[idx];

  if (!estack->cur_ephe.valid) {
    return estack->cur_ephe;
  }

  if (sbas_has_corrections_for_ephe(&estack->cur_ephe)) {
    estack->cur_ephe.sid = *sid;
    return estack->cur_ephe;
  }

  if (estack->prev_ephe.valid &&
      sbas_has_corrections_for_ephe(&estack->prev_ephe)) {
    estack->prev_ephe.sid = *sid;
    return estack->prev_ephe;
  }

  estack->cur_ephe.sid = *sid;
  return estack->cur_ephe;
}

ephemeris_t starling_efilter_get_ephe(const gnss_signal_t *sid) {
  assert(sid);
  assert(sid_valid(*sid));

  platform_mutex_lock(&starling_efilter_lock);
  ephemeris_t e = get_ephe(sid);
  platform_mutex_unlock(&starling_efilter_lock);

  return e;
}
