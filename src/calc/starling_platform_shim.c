/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <libsbp/sbp.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/sbas_raw_data.h>
#include <libswiftnav/sid_set.h>
#include <libswiftnav/single_epoch_solver.h>
#include <libswiftnav/troposphere.h>

#include "calc_base_obs.h"
#include "calc_pvt_common.h"
#include "calc_pvt_me.h"
#include "main.h"
#include "manage.h"
#include "me_msg/me_msg.h"
#include "ndb/ndb.h"
#include "nmea/nmea.h"
#include "peripherals/leds.h"
#include "piksi_systime.h"
#include "position/position.h"
#include "sbas_select/sbas_select.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "simulator.h"
#include "starling_platform_shim.h"
#include "starling_threads.h"
#include "system_monitor/system_monitor.h"
#include "timing/timing.h"

/*******************************************************************************
 * Local Helpers
 ******************************************************************************/

static bool enable_fix_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  bool enable_fix = value == 0 ? false : true;
  starling_set_enable_fix_mode(enable_fix);
  *(dgnss_filter_t *)s->addr = value;
  return ret;
}

static bool set_max_age(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  starling_set_max_correction_age(value);
  *(int *)s->addr = value;
  return ret;
}

/*******************************************************************************
 * Platform Shim Calls
 ******************************************************************************/

void platform_mutex_lock(void *mtx) { chMtxLock((mutex_t *)mtx); }

void platform_mutex_unlock(void *mtx) { chMtxUnlock((mutex_t *)mtx); }

void platform_pool_free(void *pool, void *buf) { chPoolFree(pool, buf); }

void platform_thread_create_static(
    void *wa, size_t wa_size, int prio, void (*fn)(void *), void *user) {
  chThdCreateStatic(wa, wa_size, prio, fn, user);
}

void platform_thread_set_name(const char *name) { chRegSetThreadName(name); }

/* Return true on success. */
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph) {
  return (ndb_ephemeris_read(sid, eph) == NDB_ERR_NONE);
}

/* Return true on success. */
bool platform_try_read_iono_corr(ionosphere_t *params) {
  return (ndb_iono_corr_read(params) == NDB_ERR_NONE);
}

void platform_watchdog_notify_starling_main_thread() {
  watchdog_notify(WD_NOTIFY_STARLING);
}

bool platform_simulation_enabled() { return simulation_enabled(); }

void platform_initialize_starling_filter_settings() {
  static const char *const dgnss_filter_enum[] = {"Float", "Fixed", NULL};
  static struct setting_type dgnss_filter_setting;
  static dgnss_filter_t dgnss_filter_mode = FILTER_FIXED;
  int TYPE_GNSS_FILTER =
      settings_type_register_enum(dgnss_filter_enum, &dgnss_filter_setting);

  SETTING_NOTIFY("solution",
                 "dgnss_filter",
                 dgnss_filter_mode,
                 TYPE_GNSS_FILTER,
                 enable_fix_mode);

  static u32 max_age_of_differential = 0;
  SETTING_NOTIFY("solution",
                 "correction_age_max",
                 max_age_of_differential,
                 TYPE_INT,
                 set_max_age);
}

void platform_initialize_starling_settings(void) {}
