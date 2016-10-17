/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Perttu Salmela <psalmela@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include "search_manager_api.h"
#include <timing.h>
#include <shm.h>
#include <ndb.h>
#include <board/nap/nap_common.h>
#include <libswiftnav/sv_visibility.h>

/* Search manager functions which call other modules */
/** Get SV visibility flags
 *
 * \param sid SV identifier
 * \param [out] visible set if SV is visible
 * \param [out] known set if SV is known visible or known invisible
 */
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known)
{
  last_good_fix_t lgf;
  ephemeris_t ephe;

  *visible = false;
  *known = false;

  if (NDB_ERR_NONE == ndb_lgf_read(&lgf) &&
      POSITION_FIX == lgf.position_quality &&
      NDB_ERR_NONE != ndb_ephemeris_read(sid, &ephe)) {

    sv_vis_config_t vis_cfg;

    vis_cfg.e = &ephe;
    vis_cfg.lgf_ecef[0] = lgf.position_solution.pos_ecef[0];
    vis_cfg.lgf_ecef[1] = lgf.position_solution.pos_ecef[1];
    vis_cfg.lgf_ecef[2] = lgf.position_solution.pos_ecef[2];
    vis_cfg.lgf_time = lgf.position_solution.time;
    vis_cfg.user_velocity = ACQ_MAX_USER_VELOCITY_MPS;
    vis_cfg.time_delta = (u32)((nap_timing_count() -
				gps2rxtime(&lgf.position_solution.time)) *
                               RX_DT_NOMINAL);

    sv_visibility_status_get(&vis_cfg, visible, known);
  }
}

/** Check if SV is healthy
 *
 * \param sid SV identifier
 *
 * \return TRUE is SV is healthy, FALSE otherwise
 */
bool sm_is_healthy(gnss_signal_t sid)
{
  return shm_get_sat_state(sid) != CODE_NAV_STATE_INVALID;
}

/** Get HW time of the last good fix (LGF)
 *
 * \return HW time (ms) of the last good fix
 */
u64 sm_lgf_stamp(void)
{
  last_good_fix_t lgf;
  if (ndb_lgf_read(&lgf) == NDB_ERR_NONE &&
      lgf.position_quality == POSITION_FIX) {

    return (u64)(gps2rxtime(&lgf.position_solution.time)
                 * (RX_DT_NOMINAL * 1000.0));
  }
  log_error("No LGF but in re-acquisition mode");
  /* Return time stamp which would trigger any timeouts */
  return timing_getms() - MAX(ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS,
			      ACQ_LGF_TIMEOUT_INVIS_MS);
}

