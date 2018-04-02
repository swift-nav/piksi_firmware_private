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

#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/signal.h>

void platform_mutex_lock(void *mtx);
void platform_mutex_unlock(void *mtx);
void platform_pool_free(void *pool, void *buf);
void platform_thread_create_static(
    void *wa, size_t wa_size, int prio, void (*fn)(void *), void *user);
void platform_thread_set_name(const char *name);
bool platform_try_read_ephemeris(const gnss_signal_t sid, ephemeris_t *eph);
bool platform_try_read_iono_corr(ionosphere_t *params);
void platform_watchdog_notify_starling_main_thread(void);
bool platform_simulation_enabled(void);
void platform_initialize_filter_settings(void);

#endif
