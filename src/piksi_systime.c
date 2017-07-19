/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>

#include <libswiftnav/logging.h>

#include "piksi_systime.h"

#define PIKSI_SYSTIME_CH_KERNEL_MAJOR 3
#define PIKSI_SYSTIME_CH_KERNEL_MINOR 1
#define PIKSI_SYSTIME_CH_KERNEL_PATCH 3

/* There will be some related changes to this module when ChibiOS is updated to
 * to 4.x.x, one example is the introduction of TIME_MAXIMUM constant.
 */
#if (CH_KERNEL_MAJOR != PIKSI_SYSTIME_CH_KERNEL_MAJOR || \
     CH_KERNEL_MINOR != PIKSI_SYSTIME_CH_KERNEL_MINOR || \
     CH_KERNEL_PATCH != PIKSI_SYSTIME_CH_KERNEL_PATCH)
#error Verify ChibiOS compatibility and update version check.
#endif

/** Get system time. Function detects ChibiOS systime rollover and keeps count
 * of them.
 *
 * \note Function works properly only if time between subsequent calls is less
 * or equal to ((systime)-1)
 *
 * \param[out] t            Pointer to piksi_systime_t variable, system time
 *                          will be set to this variable.
 *
 * \return TRUE: No errors; False: Failed.
 */
static bool piksi_systime_get_internal(piksi_systime_t *t)
{
  static piksi_systime_t prev = PIKSI_SYSTIME_INIT;

  if (NULL == t) {
    return FALSE;
  }

  systime_t current;

  current = chVTGetSystemTimeX();

  if (prev.systime > current) {
    log_info("System tick rollover");
    prev.rollover_cnt++;
  }

  t->systime = current;
  t->rollover_cnt = prev.rollover_cnt;

  return TRUE;
}

/* Lock version */
bool piksi_systime_get(piksi_systime_t *t)
{
  chSysLock();
  bool ret = piksi_systime_get_internal(t);
  chSysUnlock();
  return ret;
}

/* No lock version, this should be used if caller already has the lock. */
bool piksi_systime_get_x(piksi_systime_t *t)
{
  return piksi_systime_get_internal(t);
}

/** Get tick count since specific time.
 *
 * \param[in] t            Pointer to time variable indicating the start moment.
 *
 * \return System ticks since t.
 */
systime_t piksi_systime_elapsed_since_x(const piksi_systime_t *t)
{
  piksi_systime_t now;

  piksi_systime_get_x(&now);

  return piksi_systime_sub(&now, t);
}

/** Add system ticks to piksi_system_t.
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] a             System tick value to be added.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_add(piksi_systime_t *t, systime_t a)
{
  if (NULL == t) {
    return FALSE;
  }

  systime_t space = TIME_INFINITE - t->systime;

  if (space < a) {
    t->rollover_cnt++;
    t->systime = a - space;
  } else {
    t->systime += a;
  }

  return TRUE;
}

/** Compare a and b.
 *
 * \param[in] a           1st variable to compare.
 * \param[in] b           2nd variable to compare.
 *
 * \return -1 if a > b; 0 if a == b; 1 if a < b
 */
s8 piksi_systime_cmp(const piksi_systime_t *a, const piksi_systime_t *b)
{
  if (a->rollover_cnt > b->rollover_cnt) {
    return -1;
  }

  if (b->rollover_cnt > a->rollover_cnt) {
    return 1;
  }

  if (a->systime > b->systime) {
    return -1;
  }

  if (b->systime > a->systime) {
    return 1;
  }

  return 0;
}

/** Subtract b from a.
 *
 * \note Function works correctly only if the difference fits to systime_t.
 *
 * \param[in] a           Subtrahend.
 * \param[in] b           Minuend.
 *
 * \return                Difference.
 */
systime_t piksi_systime_sub(const piksi_systime_t *a, const piksi_systime_t *b)
{
  s64 a_tot = a->rollover_cnt * TIME_INFINITE + a->systime;
  s64 b_tot = b->rollover_cnt * TIME_INFINITE + b->systime;

  s64 res = a_tot - b_tot;

  /* check that the answer fits into the return type */
  assert(0 == (res ^ ((systime_t)res)));

  return res;
}

/** Suspends the invoking thread until the specified window closes.
 *
 * \param[in] t            Window start.
 * \param[in] window_len   Window length [system ticks].
 *
 * \return Time spent in sleep [system ticks].
 */
systime_t piksi_systime_sleep_until_windowed(const piksi_systime_t *t,
                                             systime_t window_len)
{
  chSysLock();
  systime_t elapsed = piksi_systime_elapsed_since_x(t);

  systime_t sleep_len = 0;

  if (elapsed < window_len) {
    sleep_len = window_len - elapsed;
    chThdSleepS(sleep_len);
  }
  chSysUnlock();

  return sleep_len;
}

