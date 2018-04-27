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

#include <libsbp/sbas.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/signal.h>

/* TODO(kevin) Put required types into separate header. */
#include "calc_base_obs.h"
#include "me_msg/me_msg.h"

/* Mutex */
void platform_mutex_lock(void *mtx);
void platform_mutex_unlock(void *mtx);
/* Thread */
void platform_thread_create_static(
    void *wa, size_t wa_size, int prio, void (*fn)(void *), void *user);
void platform_thread_set_name(const char *name);
/* Database */
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph);
bool platform_try_read_iono_corr(ionosphere_t *params);
/* Miscellaneous */
void platform_watchdog_notify_starling_main_thread(void);
bool platform_simulation_enabled(void);

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
int32_t platform_sbas_msg_mailbox_fetch(int32_t *msg, uint32_t timeout);
void platform_sbas_msg_free(msg_sbas_raw_t *ptr);

#endif
