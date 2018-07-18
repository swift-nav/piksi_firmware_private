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

#ifndef STARLING_CALC_STARLING_PLATFORM_SHIM_H
#define STARLING_CALC_STARLING_PLATFORM_SHIM_H

#include <errno.h>
#include <stdlib.h>

#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/sbas_raw_data.h>
#include <libswiftnav/signal.h>

/* TODO(kevin) Ultimately the platform layer should have no dependency
 * on starling headers. */
#include "calc/starling_threads.h"
#include "me_msg/me_msg.h"

#ifndef __STDC_LIB_EXT1__
typedef int errno_t;
#endif

typedef enum thread_id_e { THREAD_ID_TMO = 0 } thread_id_t;
typedef enum mailbox_id_e {
  MB_ID_TIME_MATCHED_OBS = 0,
  MB_ID_BASE_OBS = 1,
  MB_ID_ME_OBS = 2,
  MB_ID_SBAS_DATA = 3,
  MB_ID_COUNT = 4
} mailbox_id_t;

/* Mutex */
void platform_mutex_lock(void *mtx);
void platform_mutex_unlock(void *mtx);
/* Thread */
typedef void(platform_routine_t)(void *);

void platform_thread_create(const thread_id_t id, platform_routine_t *fn);
void platform_thread_set_name(const char *name);
/* Database */
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph);
bool platform_try_read_iono_corr(ionosphere_t *params);
/* Miscellaneous */
void platform_watchdog_notify_starling_main_thread(void);

/* internal communication between threads */
void platform_mailbox_init(mailbox_id_t id);
errno_t platform_mailbox_post(mailbox_id_t id,
                                 void *msg,
                                 uint32_t timeout_ms);
errno_t platform_mailbox_post_ahead(mailbox_id_t id,
                                       void *msg,
                                       uint32_t timeout_ms);
errno_t platform_mailbox_fetch(mailbox_id_t id,
                                  void **msg,
                                  uint32_t timeout_ms);
void *platform_mailbox_item_alloc(mailbox_id_t id);
void platform_mailbox_item_free(mailbox_id_t id, void *ptr);

#define TIME_MATCHED_OBS_THREAD_STACK (6 * 1024 * 1024)
/* Reference is <TBD> prio */
#define TIME_MATCHED_OBS_THREAD_PRIORITY (-3)

#define ME_OBS_MSG_N_BUFF 6

#endif
