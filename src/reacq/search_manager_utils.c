/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "acq/manage.h"
#include "board/nap/nap_common.h"
#include "me_constants.h"
#include "ndb/ndb.h"
#include "search_manager_api.h"
#include "shm/shm.h"
#include "sv_visibility/sv_visibility.h"
#include "swiftnav/glo_map.h"
#include "timing/timing.h"
#include "track/track_sid_db.h"

/* Ephemerides fit interval for the purpose of (re-)acq, two weeks, [s] */
#define SM_FIT_INTERVAL_VALID (WEEK_SECS * 2)

static void known_and_visible_getter(const gnss_signal_t sid,
                                     bool *visible,
                                     bool *known) {
  float elev = 0.0f;
  (*known) = sid_db_elevation_degrees_get(sid, &elev);
  if ((*known) && (elev >= get_solution_elevation_mask())) {
    *visible = true;
  }
}

/* Search manager functions which call other modules */
/** Get SV visibility flags.
 *
 * \param[in] sid GNSS signal SV identifier
 * \param[out] visible is set if SV is visible. Valid only if known is set
 * \param[out] known set if SV is known visible or known invisible
 */
u16 sm_get_visibility_flags(const me_gnss_signal_t mesid,
                            bool *visible,
                            bool *known) {
  *visible = false;
  *known = false;

  constellation_t con = code_to_constellation(mesid.code);
  /* All constellations but Glonass */
  if (CONSTELLATION_GLO != con) {
    const gnss_signal_t sid = construct_sid(mesid.code, mesid.sat);
    known_and_visible_getter(sid, visible, known);
    return mesid.sat;
  }
  /* Glonass is always special */
  u16 slot1, slot2;
  u8 ret = glo_map_get_slot_id(mesid.sat, &slot1, &slot2);
  if (0 == ret) {
    return GLO_ORBIT_SLOT_UNKNOWN; /* unknown */
  }
  const gnss_signal_t sid1 = construct_sid(mesid.code, slot1);
  known_and_visible_getter(sid1, visible, known);
  if ((*known) && (*visible)) return slot1;
  if (!(*known) && (2 == ret)) {
    const gnss_signal_t sid2 = construct_sid(mesid.code, slot2);
    known_and_visible_getter(sid2, visible, known);
    if ((*known) && (*visible)) return slot2;
  }
  return GLO_ORBIT_SLOT_UNKNOWN;
}

/* Search manager functions which call other modules */
/** Get SV visibility flags.
 *
 * \param[in] sid GNSS signal SV identifier
 * \param[out] visible is set if SV is visible. Valid only if known is set
 * \param[out] known set if SV is known visible or known invisible
 */
u16 sm_mesid_to_sat(const me_gnss_signal_t mesid) {
  bool visible;
  bool known;

  return sm_get_visibility_flags(mesid, &visible, &known);
}