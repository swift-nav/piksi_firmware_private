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

  t->systime = prev.systime = current;
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

/** Convert piksi_systime to system ticks.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to system ticks.
 */
static u64 piksi_systime_to_ticks(const piksi_systime_t *t)
{
  return t->rollover_cnt * ((u64)TIME_INFINITE + 1) + t->systime;
}

/** Convert piksi_systime to microseconds.
 *
 * \note The result is rounded up to the next microsecond boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to microseconds.
 */
u64 piksi_systime_to_us(const piksi_systime_t *t)
{
  return ST2US(piksi_systime_to_ticks(t));
}

/** Convert piksi_systime to milliseconds.
 *
 * \note The result is rounded up to the next millisecond boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to milliseconds.
 */
u64 piksi_systime_to_ms(const piksi_systime_t *t)
{
  return ST2MS(piksi_systime_to_ticks(t));
}

/** Convert piksi_systime to seconds.
 *
 * \note The result is rounded up to the next second boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to seconds.
 */
u64 piksi_systime_to_s(const piksi_systime_t *t)
{
  return ST2S(piksi_systime_to_ticks(t));
}

/** Subtract b from a.
 *
 * \note Function works correctly only if the difference fits to s64.
 *
 * \param[in] a           Subtrahend.
 * \param[in] b           Minuend.
 *
 * \return                Difference.
 */
static systime_t piksi_systime_sub_internal(const piksi_systime_t *a,
                                            const piksi_systime_t *b)
{
  u64 a_tot = piksi_systime_to_ticks(a);
  u64 b_tot = piksi_systime_to_ticks(b);

  s64 res = a_tot - b_tot;

  /* check that the answer fits into the return type */
  assert(b_tot + res == a_tot);

  return res;
}

/** Carry out subtraction and return result as microseconds.
 *
 * \note The result is rounded up to the next microsecond boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \param[in] a           Subtrahend.
 * \param[in] b           Minuend.
 *
 * \return a - b result as microseconds.
 */
u32 piksi_systime_sub_us(const piksi_systime_t *a, const piksi_systime_t *b)
{
  return ST2US(piksi_systime_sub_internal(a, b));
}

/** Carry out subtraction and return result as milliseconds.
 *
 * \note The result is rounded up to the next millisecond boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \param[in] a           Subtrahend.
 * \param[in] b           Minuend.
 *
 * \return a - b result as milliseconds.
 */
u32 piksi_systime_sub_ms(const piksi_systime_t *a, const piksi_systime_t *b)
{
  return ST2MS(piksi_systime_sub_internal(a, b));
}

/** Carry out subtraction and return result as seconds.
 *
 * \note The result is rounded up to the next second boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \param[in] a           Subtrahend.
 * \param[in] b           Minuend.
 *
 * \return a - b result as seconds.
 */
u32 piksi_systime_sub_s(const piksi_systime_t *a, const piksi_systime_t *b)
{
  return ST2S(piksi_systime_sub_internal(a, b));
}

/** Get tick count since specific time.
 *
 * \param[in] t            Pointer to time variable indicating the start moment.
 *
 * \return System ticks since t.
 */
static systime_t piksi_systime_elapsed_since_x(const piksi_systime_t *t)
{
  piksi_systime_t now;

  piksi_systime_get_x(&now);

  return piksi_systime_sub_internal(&now, t);
}

/** Get microsecond count since specific time.
 *
 * \note The result is rounded up to the next microsecond boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \return Microseconds since t.
 */
u32 piksi_systime_elapsed_since_us_x(const piksi_systime_t *t)
{
  return ST2US(piksi_systime_elapsed_since_x(t));
}

/** Get millisecond count since specific time.
 *
 * \note The result is rounded up to the next millisecond boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \return Milliseconds since t.
 */
u32 piksi_systime_elapsed_since_ms_x(const piksi_systime_t *t)
{
  return ST2MS(piksi_systime_elapsed_since_x(t));
}

/** Get second count since specific time.
 *
 * \note The result is rounded up to the next second boundary.
 * TODO: Investigate rollovers and add checks and security against them.
 *
 * \return Seconds since t.
 */
u32 piksi_systime_elapsed_since_s_x(const piksi_systime_t *t)
{
  return ST2S(piksi_systime_elapsed_since_x(t));
}

/** Increment piksi_system_t.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           System tick value to be added.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_inc_internal(piksi_systime_t *t, systime_t inc)
{
  if (NULL == t) {
    return FALSE;
  }

  systime_t space = TIME_INFINITE - t->systime;

  if (space < inc) {
    t->rollover_cnt++;
    t->systime = inc - space - 1;
  } else {
    t->systime += inc;
  }

  return TRUE;
}

/** Increment piksi_system_t with microsecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 * TODO: Investigate conversion rollovers and add checks and security.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Microsecond value to be added.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_inc_us(piksi_systime_t *t, u32 inc)
{
  return piksi_systime_inc_internal(t, US2ST(inc));
}

/** Increment piksi_system_t with millisecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 * TODO: Investigate conversion rollovers and add checks and security.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Millisecond value to be added.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_inc_ms(piksi_systime_t *t, u32 inc)
{
  return piksi_systime_inc_internal(t, MS2ST(inc));
}

/** Increment piksi_system_t with second value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 * TODO: Investigate conversion rollovers and add checks and security.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Millisecond value to be added.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_inc_s(piksi_systime_t *t, u32 inc)
{
  return piksi_systime_inc_internal(t, S2ST(inc));
}

/** Decrease piksi_system_t.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           System tick value to be decreased.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_dec_internal(piksi_systime_t *t, systime_t inc)
{
  if (NULL == t) {
    return FALSE;
  }

  if (inc > t->systime) {
    assert(0 < t->rollover_cnt);
    t->rollover_cnt--;
    inc -= (t->systime + 1);
  }

  t->systime -= inc;

  return TRUE;
}

/** Decrease piksi_system_t with microsecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 * TODO: Investigate conversion rollovers and add checks and security.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Microsecond value to be decreased.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_dec_us(piksi_systime_t *t, u32 dec)
{
  return piksi_systime_dec_internal(t, US2ST(dec));
}

/** Decrease piksi_system_t with millisecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 * TODO: Investigate conversion rollovers and add checks and security.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Millisecond value to be decreased.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_dec_ms(piksi_systime_t *t, u32 dec)
{
  return piksi_systime_dec_internal(t, MS2ST(dec));
}

/** Decrease piksi_system_t with second value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 * TODO: Investigate conversion rollovers and add checks and security.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Second value to be decreased.
 *
 * \return TRUE: No errors; False: Failed.
 */
bool piksi_systime_dec_s(piksi_systime_t *t, u32 dec)
{
  return piksi_systime_dec_internal(t, S2ST(dec));
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

/** Suspends the invoking thread for the specified time.
 *
 * \param[in] len   Sleep length [system ticks].
 */
void piksi_systime_sleep_s_internal(systime_t len)
{
  chThdSleepS(len);
}

void piksi_systime_sleep_us_s(u32 len_us)
{
  piksi_systime_sleep_s_internal(US2ST(len_us));
}

/** Suspends the invoking thread until the specified window closes.
 *
 * \param[in] t            Window start.
 * \param[in] window_len   Window length [system ticks].
 *
 * \return Time spent in sleep [system ticks].
 */
systime_t piksi_systime_sleep_until_windowed_internal(const piksi_systime_t *t,
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

u32 piksi_systime_sleep_until_windowed_us(const piksi_systime_t *t,
                                          u32 window_len_us)
{
  systime_t ret =\
    piksi_systime_sleep_until_windowed_internal(t, US2ST(window_len_us));

  return ST2US(ret);
}

u32 piksi_systime_sleep_until_windowed_ms(const piksi_systime_t *t,
                                          u32 window_len_ms)
{
  systime_t ret =\
    piksi_systime_sleep_until_windowed_internal(t, MS2ST(window_len_ms));

  return ST2MS(ret);
}

