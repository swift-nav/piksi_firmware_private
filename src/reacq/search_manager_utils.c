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
#include <board/nap/nap_common.h>
#include "manage.h"
#include "me_constants.h"
#include "ndb/ndb.h"
#include "search_manager_api.h"
#include "shm/shm.h"
#include "timing/timing.h"

#include "sv_visibility/sv_visibility.h"

/* Ephemerides fit interval for the purpose of (re-)acq, two weeks, [s] */
#define SM_FIT_INTERVAL_VALID (WEEK_SECS * 2)

typedef struct {
  bool visible; /** Visible flag */
  bool known;   /** Known flag */
} sm_glo_sv_vis_t;

/* The array keeps latest visibility flags of each GLO SV */
static sm_glo_sv_vis_t glo_sv_vis[NUM_SATS_GLO] = {0};

/* Search manager functions which call other modules */
/** Get SV visibility flags.
 *
 * \param[in] sid GNSS signal SV identifier
 * \param[out] visible is set if SV is visible. Valid only if known is set
 * \param[out] known set if SV is known visible or known invisible
 */
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known) {
  last_good_fix_t lgf;
  ephemeris_t ephe;

  *visible = false;
  *known = false;

  if (NDB_ERR_NONE != ndb_lgf_read(&lgf) ||
      POSITION_FIX != lgf.position_quality) {
    return;
  }

  ndb_op_code_t op_code = ndb_ephemeris_read(sid, &ephe);
  if (NDB_ERR_NONE != op_code && NDB_ERR_UNCONFIRMED_DATA != op_code) {
    return;
  }

  gps_time_t t = get_current_time();

  ephe.fit_interval = SM_FIT_INTERVAL_VALID;
  if (!ephemeris_valid(&ephe, &t)) {
    return;
  }

  sv_vis_config_t vis_cfg;

  vis_cfg.e = &ephe;
  vis_cfg.lgf_ecef[0] = lgf.position_solution.pos_ecef[0];
  vis_cfg.lgf_ecef[1] = lgf.position_solution.pos_ecef[1];
  vis_cfg.lgf_ecef[2] = lgf.position_solution.pos_ecef[2];
  vis_cfg.lgf_time = lgf.position_solution.time;
  vis_cfg.user_velocity = MAX_USER_VELOCITY_MPS;
  vis_cfg.time_delta = (u32)(
      (nap_timing_count() - gpstime2napcount(&lgf.position_solution.time)) *
      RX_DT_NOMINAL);

  sv_visibility_status_get(&vis_cfg, visible, known);
}

/** Get HW time of the last good fix (LGF)
 *
 * \param[out] lgf_stamp time of LGF (ms)
 * \return true lgf_stamp is valid, false otherwise
 */
bool sm_lgf_stamp(u64 *lgf_stamp) {
  last_good_fix_t lgf;
  if (TIME_UNKNOWN == get_time_quality()) {
    return false;
  }
  if (ndb_lgf_read(&lgf) != NDB_ERR_NONE ||
      lgf.position_quality != POSITION_FIX) {
    return false;
  }

  *lgf_stamp = (u64)(gpstime2napcount(&lgf.position_solution.time) *
                     (RX_DT_NOMINAL * 1000.0));
  return true;
}

/**
 * The function calculates and stores visibility flags for all GLO SV
 *
 * Since work time of Runge-Kutta algorithm depends on GLO SV position
 * calculation period, due to iteration number
 * (see modeling https://github.com/swift-nav/exafore_planning/issues/681)
 * we continuously calculate the position.
 */
void sm_calc_all_glo_visibility_flags(void) {
  if (!is_glo_enabled()) {
    return;
  }

  for (u16 glo_sat = 1; glo_sat <= NUM_SATS_GLO; glo_sat++) {
    gnss_signal_t glo_sid = construct_sid(CODE_GLO_L1OF, glo_sat);
    bool visible, known;
    sm_get_visibility_flags(glo_sid, &visible, &known);
    glo_sv_vis[glo_sat - 1].visible = visible;
    glo_sv_vis[glo_sat - 1].known = known;
  }
}

/** Get GLO SV visibility flags. Function simply copies previously calculated
 * visibility flags for GLO SV
 *
 * \param[in] sat GLO SV orbital slot
 * \param[out] visible is set if SV is visible. Valid only if known is set
 * \param[out] known set if SV is known visible or known invisible
 */
void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known) {
  *visible = glo_sv_vis[sat - 1].visible;
  *known = glo_sv_vis[sat - 1].known;
}
