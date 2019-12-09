/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "system_monitor.h"

#include <hal.h>
#include <inttypes.h>
#include <libsbp/system.h>
#include <libsbp/version.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>

#include "acq/manage.h"
#include "board.h"
#include "calc/calc_base_obs.h"
#include "hal/piksi_systime.h"
#include "main/main.h"
#include "nap/nap_common.h"
#include "ndb/ndb.h"
#include "nt1065.h"
#include "peripherals/antenna.h"
#include "position/position.h"
#include "remoteproc/rpmsg.h"
#include "sbp/sbp.h"
#include "settings/settings_client.h"
#include "simulator/simulator.h"

#define WATCHDOG_THREAD_PERIOD_MS 15000
extern const WDGConfig board_wdg_config;

#define WATCHDOG_NOTIFY_FLAG(id) (1UL << (id))
#define WATCHDOG_NOTIFY_FLAG_ALL \
  ((WATCHDOG_NOTIFY_FLAG(WD_NOTIFY_NUM_THREADS)) - 1)

#define FRONTEND_AOK_ERROR_FLAG 1

#define SYSTEM_MONITOR_THREAD_PRIORITY (HIGHPRIO - 2)
#define SYSTEM_MONITOR_THREAD_STACK (2 * 1024)

#define WATCHDOG_THREAD_PRIORITY (HIGHPRIO)
#define WATCHDOG_THREAD_STACK (1 * 1024)

/* Time between sending system monitor and heartbeat messages in milliseconds */
static uint32_t heartbeat_period_milliseconds = 1000;
/* Use watchdog timer or not */
static bool use_wdt = true;

static u32 watchdog_watch_mask = WATCHDOG_NOTIFY_FLAG_ALL;
static u32 watchdog_notify_flags = 0;
static u32 frontend_notify_flags = 0;
static bool frontend_errors = false;

/* Base station mode settings. */
/* TODO: Relocate to a different file? */
static bool broadcast_surveyed_position = false;
static double base_llh[3];

/* Global CPU time accumulator, used to measure thread CPU usage. */
u64 g_ctime = 0;

u32 check_stack_free(thread_t *tp) {
  u32 *stack = (u32 *)tp->p_stklimit;
  u32 i;
  for (i = 0; i < 65536 / sizeof(u32); i++) {
    if (stack[i] != 0x55555555) break;
  }
  return 4 * (i - 1);
}

void send_thread_states(void) {
  thread_t *tp = chRegFirstThread();
  while (tp) {
    msg_thread_state_t tp_state;
    u16 cpu = 1000.0f * tp->p_ctime / (float)g_ctime;
    tp_state.cpu = cpu;
    tp_state.stack_free = check_stack_free(tp);
    const char *name = chRegGetThreadNameX(tp);
    if (name != NULL) {
      strncpy(tp_state.name, name, sizeof(tp_state.name));
    } else {
      memset(tp_state.name, 0, sizeof(tp_state.name));
    }
    sbp_send_msg(SBP_MSG_THREAD_STATE, sizeof(tp_state), (u8 *)&tp_state);

    tp->p_ctime = 0; /* Reset thread CPU cycle count */
    tp = chRegNextThread(tp);
  }
  g_ctime = 0;
}

static void check_frontend_errors(void) {
  if (!frontend_errors) {
    chSysLock();
    frontend_errors = (frontend_notify_flags & FRONTEND_AOK_ERROR_FLAG) != 0;
    frontend_notify_flags = 0;
    chSysUnlock();

    if (frontend_errors) {
      log_error("nt1065: AOK error flag set");
    }
  }

  if (frontend_errors) {
    if (nt1065_check_aok_status()) {
      log_info("nt1065: AOK error flag cleared");
      frontend_errors = false;
    } else if (nt1065_check_plls() && nt1065_check_standby() &&
               nt1065_check_calibration()) {
      log_error("nt1065: AOK failed with unknown cause");
    }
  }
}

/**
 * How often to perform RF front-end LPF and PLL calibration in
 * absense of temperature readings [s].
 * Normally temperature reading should be available at all times.
 * This threshold is a safety measure in case of missing temperature
 * readings.
 * The selection of 1 minute is arbitrary assuming that temperature
 * does not change much during this time.
 */
#define FE_CALIBRATION_REPEAT_TIMEOUT_S (1 * 60)

/**
 * The absolute value temperature change needed to trigger
 * RF front-end LPF and PLL calibration [Celsius].
 * NT1065 tech support was contacted about this value and
 * the recommended maximum was 50 degrees Celsius.
 * The selected smaller value is an extra safety measure.
 */
#define FE_CALIBRATION_THRESHOLD_C 30.f

/**
 * Performs calibration of NT1065 PLLs and LPF as a function of RF temperature
 * If temperature readings are not available, a time-out based calibration
 * is performed.
 * @param now current system time
 */
static void calibrate_rf_plls_and_lpf(const piksi_systime_t *now) {
  assert(now);

  static piksi_systime_t calibrate_epoch = {0};
  static double calibrate_temperature_c = 0;
  double temperature_c = 0;
  bool temperature_valid = nt1065_get_temperature(&temperature_c);

  static int init_done = 0;
  if (!init_done) {
    init_done = 1;
    if (temperature_valid) {
      calibrate_temperature_c = temperature_c;
    }
    calibrate_epoch = *now;
    return;
  }

  if (temperature_valid) {
    double diff_c = fabs(temperature_c - calibrate_temperature_c);
    if (diff_c < FE_CALIBRATION_THRESHOLD_C) {
      return;
    }
  } else {
    u64 diff_s = piksi_systime_elapsed_since_s(now);
    if (diff_s < FE_CALIBRATION_REPEAT_TIMEOUT_S) {
      return;
    }
  }

  /* we either face a significant temperature change or
     too much time has elapsed since last RF PLL and LPF calibration:
     re-calibrate RF front-end PLLs and LPF */

  const char* plls = nt1065_calibrate_plls() ? "success" : "failure";
  log_warn("Calibration of NT1065 PLLs at %.1lf C: %s", temperature_c, plls);

  const char* lpf = nt1065_calibrate_lpf() ? "success" : "failure";
  log_warn("Calibration of NT1065 LPF at %.1lf C: %s", temperature_c, lpf);

  calibrate_epoch = *now;
  calibrate_temperature_c = temperature_c;
}

static THD_WORKING_AREA(wa_system_monitor_thread, SYSTEM_MONITOR_THREAD_STACK);
static void system_monitor_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("system monitor");

  piksi_systime_t time;

  while (TRUE) {
    piksi_systime_get(&time);

    u32 status_flags = antenna_present() << 31 | antenna_shorted() << 30 |
                       SBP_MAJOR_VERSION << 16 | SBP_MINOR_VERSION << 8;
    sbp_send_msg(SBP_MSG_HEARTBEAT, sizeof(status_flags), (u8 *)&status_flags);

    /* If we are in base station mode then broadcast our known location. */
    if (broadcast_surveyed_position) {
      last_good_fix_t lgf;
      if (ndb_lgf_read(&lgf) == NDB_ERR_NONE &&
          lgf.position_quality == POSITION_FIX) {
        double tmp[3];
        double base_ecef[3];
        double base_distance;

        llhdeg2rad(base_llh, tmp);
        wgsllh2ecef(tmp, base_ecef);

        base_distance =
            vector_distance(3, base_ecef, lgf.position_solution.pos_ecef);

        if (base_distance > SPP_BASE_STATION_DIFFERENCE_WARNING_THRESHOLD) {
          log_warn(
              "Invalid surveyed position coordinates. No base position message "
              "will be sent.");
        } else {
          sbp_send_msg(SBP_MSG_BASE_POS_ECEF,
                       sizeof(msg_base_pos_ecef_t),
                       (u8 *)&base_ecef);
        }
      }
    }

    DO_EVERY(2, send_thread_states(););

    DO_EVERY(3, board_send_state(););

    DO_EACH_MS(FE_CALIBRATION_REPEAT_TIMEOUT_S * 1000,
      calibrate_rf_plls_and_lpf(&time););

    DO_EVERY(3, check_frontend_errors(););
    piksi_systime_sleep_until_windowed_ms(&time, heartbeat_period_milliseconds);
  }
}

static void debug_threads(void) {
  const char *state[] = {CH_STATE_NAMES};
  thread_t *tp = chRegFirstThread();
  while (tp) {
    log_info("%s (%u: %s): prio: %" PRIu32 ", flags: %u, wtobjp: %p",
             tp->p_name,
             tp->p_state,
             state[tp->p_state],
             tp->p_prio,
             tp->p_flags,
             tp->p_u.wtobjp);
    tp = chRegNextThread(tp);
  }
}

static void panic_dead_thread(u32 threads_dead) {
  const char *state[] = {CH_STATE_NAMES};
  u32 thd_cnt = 0;
  thread_t *tp = chRegFirstThread();
  while (tp) {
    if (threads_dead & WATCHDOG_NOTIFY_FLAG(thd_cnt)) {
      log_error("Thread Died: %s (%u: %s): prio: %" PRIu32
                ", flags: %u, wtobjp: %p",
                tp->p_name,
                tp->p_state,
                state[tp->p_state],
                tp->p_prio,
                tp->p_flags,
                tp->p_u.wtobjp);
    }
    tp = chRegNextThread(tp);
    thd_cnt++;
  }
  chSysHalt("Forced halt due to dead or starved thread.");
}

static void declare_panic(void) {
  mutex_t *mtx = rpmsg_fw_panic();
  if (mtx != NULL) {
    chMtxLock(mtx);
    chMtxUnlock(mtx);
  }
}

static THD_WORKING_AREA(wa_watchdog_thread, WATCHDOG_THREAD_STACK);
static void watchdog_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("Watchdog");

  /* Allow an extra period at startup since some of the other threads
     take a little while to get going */
  chThdSleepMilliseconds(WATCHDOG_THREAD_PERIOD_MS);

  if (use_wdt) wdgStart(&WDGD1, &board_wdg_config);

  static const u64 tc_thres_lo =
      (((WATCHDOG_THREAD_PERIOD_MS / 1000) - 1) * NAP_TRACK_SAMPLE_RATE_Hz);
  static const u64 tc_thres_hi =
      (((WATCHDOG_THREAD_PERIOD_MS / 1000) + 1) * NAP_TRACK_SAMPLE_RATE_Hz);
  u64 tc_prev = nap_timing_count();
  while (TRUE) {
    /* Wait for all threads to set a flag indicating they are still
       alive and performing their function */
    chThdSleepMilliseconds(WATCHDOG_THREAD_PERIOD_MS);

    u64 tc_now = nap_timing_count();
    u64 tc_diff = tc_now - tc_prev;
    tc_prev = tc_now;
    bool tc_failure = false;
    if ((tc_diff < tc_thres_lo) || (tc_diff > tc_thres_hi)) {
      tc_failure = true;
    }
    if (tc_failure) {
      log_error(
          "NAP timing mismatch of %.1f seconds. "
          "Watchdog reset %s.",
          tc_diff * RX_DT_NOMINAL,
          use_wdt ? "imminent" : "disabled");
    }

    chSysLock();
    u32 threads_dead = watchdog_notify_flags ^ watchdog_watch_mask;
    watchdog_notify_flags = 0;
    chSysUnlock();

    if (threads_dead) {
      log_error(
          "One or more threads appear to be dead: 0x%08X. "
          "Watchdog reset %s.",
          (unsigned int)threads_dead,
          use_wdt ? "imminent" : "disabled");
    }

    if (threads_dead || tc_failure) {
      /* TODO: ChibiOS thread state dump */
      declare_panic();
      debug_threads();
      if (use_wdt) {
        panic_dead_thread(threads_dead);
      }
    } else {
      if (use_wdt) wdgReset(&WDGD1);
    }
  }
}

void system_monitor_pre_init(void) { wdgStart(&WDGD1, &board_wdg_config); }

void system_monitor_setup(void) {
  SETTING("system_monitor",
          "heartbeat_period_milliseconds",
          heartbeat_period_milliseconds,
          SETTINGS_TYPE_INT);
  SETTING("system_monitor", "watchdog", use_wdt, SETTINGS_TYPE_BOOL);

  SETTING("surveyed_position",
          "broadcast",
          broadcast_surveyed_position,
          SETTINGS_TYPE_BOOL);
  SETTING(
      "surveyed_position", "surveyed_lat", base_llh[0], SETTINGS_TYPE_FLOAT);
  SETTING(
      "surveyed_position", "surveyed_lon", base_llh[1], SETTINGS_TYPE_FLOAT);
  SETTING(
      "surveyed_position", "surveyed_alt", base_llh[2], SETTINGS_TYPE_FLOAT);

  chThdCreateStatic(wa_system_monitor_thread,
                    sizeof(wa_system_monitor_thread),
                    SYSTEM_MONITOR_THREAD_PRIORITY,
                    system_monitor_thread,
                    NULL);
  chThdCreateStatic(wa_watchdog_thread,
                    sizeof(wa_watchdog_thread),
                    WATCHDOG_THREAD_PRIORITY,
                    watchdog_thread,
                    NULL);
}

void watchdog_thread_watch(watchdog_notify_t thread_id) {
  chSysLock();
  watchdog_watch_mask |= WATCHDOG_NOTIFY_FLAG(thread_id);
  chSysUnlock();
}

void watchdog_thread_ignore(watchdog_notify_t thread_id) {
  chSysLock();
  watchdog_watch_mask &= ~WATCHDOG_NOTIFY_FLAG(thread_id);
  chSysUnlock();
}

/** Called by each important system thread after doing its important
 * work, to notify the system monitor that it's functioning normally.
 *
 * \param thread_id Unique identifier for the thread.
 **/
void watchdog_notify(watchdog_notify_t thread_id) {
  chSysLock();
  watchdog_notify_flags |= WATCHDOG_NOTIFY_FLAG(thread_id);
  chSysUnlock();
}

/** Called by ISR for frontend AOK signal to notify error occured
 **/
void frontend_error_notify_isr() {
  chSysLockFromISR();
  frontend_notify_flags |= FRONTEND_AOK_ERROR_FLAG;
  chSysUnlockFromISR();
}

/** Called during normal execution to notify frontend error occured
 **/
void frontend_error_notify_sys() {
  chSysLock();
  frontend_notify_flags |= FRONTEND_AOK_ERROR_FLAG;
  chSysUnlock();
}

/** \} */
