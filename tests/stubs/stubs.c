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

bool mesid_is_tracked(const me_gnss_signal_t mesid) {
  (void)mesid;
  return true;
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

void sm_get_visibility_flags(gnss_signal_t sid, bool *visible, bool *known) {
  (void)sid;
  *visible = true;
  *known = true;
}

void sm_calc_all_glo_visibility_flags(void){};
u64 timing_getms(void) { return (u64)0; }
bool sm_lgf_stamp(u64 *lgf_stamp) {
  (void)lgf_stamp;
  return true;
}
