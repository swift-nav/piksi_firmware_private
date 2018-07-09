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

#include <string.h>

#include <hal.h>

#include <libsbp/system.h>
#include <libsbp/version.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include <libswiftnav/logging.h>

#include "board.h"
#include "board/nap/nap_common.h"
#include "calc_base_obs.h"
#include "main.h"
#include "manage.h"
#include "ndb/ndb.h"
#include "nt1065.h"
#include "peripherals/antenna.h"
#include "piksi_systime.h"
#include "position/position.h"
#include "sbp.h"
#include "settings/settings.h"
#include "simulator.h"
#include "system_monitor.h"

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

extern u64 st2ms(u64 st);

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

    log_info("D# %d %d", (int)tp->p_prio, (int)st2ms(tp->p_max));

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

extern u64 timing_getms(void);

u16 heartbeat(int prio, u16 prev_ms) {
  (void)prio;
  (void)prev_ms;
  u16 now_ms = (u16)timing_getms();
  //log_info("H# %d %" PRIu16, prio, (u16)(now_ms - prev_ms));
  return now_ms;
}

static THD_WORKING_AREA(wa_system_monitor_thread, SYSTEM_MONITOR_THREAD_STACK);
static void system_monitor_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("system monitor");

  piksi_systime_t time;

  while (TRUE) {
    DO_EACH_MS(3000, {
      static u16 prev_ms = 0;
      prev_ms = heartbeat(SYSTEM_MONITOR_THREAD_PRIORITY, prev_ms);
    });

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

    DO_EVERY(3, check_frontend_errors(););
    piksi_systime_sleep_until_windowed_ms(&time, heartbeat_period_milliseconds);
  }
}

static void debug_threads(void) {
  const char *state[] = {CH_STATE_NAMES};
  thread_t *tp = chRegFirstThread();
  while (tp) {
    log_info("%s (%u: %s): prio: %lu, flags: %u, wtobjp: %p",
             tp->p_name,
             tp->p_state,
             state[tp->p_state],
             tp->p_prio,
             tp->p_flags,
             tp->p_u.wtobjp);
    tp = chRegNextThread(tp);
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

  while (TRUE) {
    /* Wait for all threads to set a flag indicating they are still
       alive and performing their function */
    chThdSleepMilliseconds(WATCHDOG_THREAD_PERIOD_MS);

    DO_EACH_MS(3000, {
      static u16 prev_ms = 0;
      prev_ms = heartbeat(WATCHDOG_THREAD_PRIORITY, prev_ms);
    });

    chSysLock();
    u32 threads_dead = watchdog_notify_flags ^ WATCHDOG_NOTIFY_FLAG_ALL;
    watchdog_notify_flags = 0;
    chSysUnlock();

    if (threads_dead) {
      /* TODO: ChibiOS thread state dump */
      log_error(
          "One or more threads appear to be dead: 0x%08X. "
          "Watchdog reset %s.",
          (unsigned int)threads_dead,
          use_wdt ? "imminent" : "disabled");
      debug_threads();
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
          TYPE_INT);
  SETTING("system_monitor", "watchdog", use_wdt, TYPE_BOOL);

  SETTING(
      "surveyed_position", "broadcast", broadcast_surveyed_position, TYPE_BOOL);
  SETTING("surveyed_position", "surveyed_lat", base_llh[0], TYPE_FLOAT);
  SETTING("surveyed_position", "surveyed_lon", base_llh[1], TYPE_FLOAT);
  SETTING("surveyed_position", "surveyed_alt", base_llh[2], TYPE_FLOAT);

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
