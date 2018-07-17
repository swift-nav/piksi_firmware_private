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

#include <stdlib.h>

#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/sbas_raw_data.h>
#include <libswiftnav/signal.h>

/* TODO(kevin) Ultimately the platform layer should have no dependency
 * on starling headers. */
#include "calc/starling_threads.h"
#include "me_msg/me_msg.h"

typedef struct platform_thread_info_s platform_thread_info_t;

typedef enum thread_id_e { THREAD_ID_TMO = 0 } thread_id_t;

/* Mutex */
void platform_mutex_lock(void *mtx);
void platform_mutex_unlock(void *mtx);
/* Thread */
typedef void(platform_routine_t)(void *);
void platform_thread_info_init(const thread_id_t id,
                               platform_thread_info_t *info);
inline void platform_thread_info_destroy(platform_thread_info_t *info) {
  free(info);
};
void platform_thread_create(platform_thread_info_t *info,
                            int prio,
                            platform_routine_t *fn,
                            void *user);
void platform_thread_set_name(const char *name);
/* Database */
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph);
bool platform_try_read_iono_corr(ionosphere_t *params);
/* Miscellaneous */
void platform_watchdog_notify_starling_main_thread(void);

/* internal communication between threads */
void platform_time_matched_obs_mailbox_init(void);
int32_t platform_time_matched_obs_mailbox_post(int32_t msg, uint32_t timeout);
int32_t platform_time_matched_obs_mailbox_post_ahead(int32_t msg,
                                                     uint32_t timeout);
int32_t platform_time_matched_obs_mailbox_fetch(int32_t *msg, uint32_t timeout);

/* memory management for internal communication */
obss_t *platform_time_matched_obs_alloc(void);
void platform_time_matched_obs_free(obss_t *ptr);

/* used for receiving obs messages */
int32_t platform_base_obs_mailbox_fetch(int32_t *msg, uint32_t timeout);
void platform_base_obs_free(obss_t *ptr);

/* used for receiving me messages */
int32_t platform_me_obs_msg_mailbox_fetch(int32_t *msg, uint32_t timeout);
void platform_me_obs_msg_free(me_msg_obs_t *ptr);

/* used for receiving sbas messages */
void platform_sbas_data_mailbox_setup(void);
void platform_sbas_data_mailbox_post(const sbas_raw_data_t *sbas_data);
int32_t platform_sbas_data_mailbox_fetch(int32_t *msg, uint32_t timeout);
void platform_sbas_data_free(sbas_raw_data_t *ptr);

#define TIME_MATCHED_OBS_THREAD_STACK (6 * 1024 * 1024)
/* Reference is <TBD> prio */
#define TIME_MATCHED_OBS_THREAD_PRIORITY (-3)

#endif
