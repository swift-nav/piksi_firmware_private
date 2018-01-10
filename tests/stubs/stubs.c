/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "ch.h"
#include "manage.h"

extern test_case_t *test_case;
extern test_case_t test_cases;

void chSysLock(){};

void chSysUnlock(){};

systime_t chVTGetSystemTimeX() { return 0; };

systime_t chThdSleep(systime_t time) {
  (void)time;
  return 1;
}

systime_t chThdSleepS(systime_t time) {
  (void)time;
  return 1;
}

/** Check if SV is tracked
 *
 * \param sid SV identifier
 *
 * \return true is SV is tracked, false otherwise
 */
bool mesid_is_tracked(const me_gnss_signal_t mesid) {
  if (0 != (test_case->track_mask & (1 << mesid_to_code_index(mesid)))) {
    return true;
  }
  return false;
}

bool is_glo_enabled(void) { return true; }

bool is_sbas_enabled(void) { return true; }

bool is_bds2_enabled(void) { return true; }

bool is_qzss_enabled(void) { return true; }

bool is_galileo_enabled(void) { return true; }

void sm_get_glo_visibility_flags(u16 sat, bool *visible, bool *known) {
  (void)sat;
  *visible = true;
  *known = true;
}

/** Get SV visibility flags
 *
 * \param[in] sid GNSS signal identifier
 * \param[out] visible set if SV is visible
 * \param[out] known set if SV is known visible or known invisible
 */
void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known) {
  if (0 != (test_case->vis_mask & (1 << sid_to_code_index(sid)))) {
    *visible = true;
  } else {
    *visible = false;
  }
  if (0 != (test_case->known_mask & (1 << sid_to_code_index(sid)))) {
    *known = true;
  } else {
    *known = false;
  }
}

void sm_calc_all_glo_visibility_flags(void){};

/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
u64 timing_getms(void) { return (u64)test_case->now_ms; }

/** Get HW time of the last good fix (LGF)
 *
 * \param[out] lgf_stamp time of LGF (ms)
 * \return true lgf_stamp is valid, false otherwise
 */
bool sm_lgf_stamp(u64 *lgf_stamp) {
  *lgf_stamp = (u64)test_case->lgf_stamp_ms;
  return true;
}
