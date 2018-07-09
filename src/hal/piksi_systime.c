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

#include <libswiftnav/gnss_time.h>
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

/** Convert time to system ticks. Based on *2ST macros in
 * ChibiOS/os/rt/include/chvt.h with wider variable types.
 *
 * \note The result is rounded upward to the next tick boundary.
 */
#define PIKSI_TIME2ST(t, prefix) \
  (((u64)(t)*CH_CFG_ST_FREQUENCY + ((prefix)-1)) / (prefix))

#define PIKSI_US2ST(t) PIKSI_TIME2ST(t, SECS_US)
#define PIKSI_MS2ST(t) PIKSI_TIME2ST(t, SECS_MS)
#define PIKSI_S2ST(t) PIKSI_TIME2ST(t, 1)

/** Define the maximum values that PIKSI_*2ST macros can handle.
 *
 * Dividend (u64)t * PIKSI_ST_FREQ + (prefix - 1) must fit into u64.
 */
#define PIKSI_TIME2_LIMIT(prefix) \
  (((u64)-1 - (u64)(prefix) + 1) / (u64)CH_CFG_ST_FREQUENCY)

#define PIKSI_US2ST_LIMIT PIKSI_TIME2_LIMIT(SECS_US)
#define PIKSI_MS2ST_LIMIT PIKSI_TIME2_LIMIT(SECS_MS)
#define PIKSI_S2ST_LIMIT PIKSI_TIME2_LIMIT(1)

/** Convert system ticks to time. Based on ST2* macros in
 * ChibiOS/os/rt/include/chvt.h with wider variable types.
 *
 * \note The result is rounded up to the next time unit boundary.
 */
#define PIKSI_ST2TIME(n, prefix)                                 \
  (((u64)(n) * (u64)(prefix) + ((u64)CH_CFG_ST_FREQUENCY - 1)) / \
   (u64)CH_CFG_ST_FREQUENCY)

#define PIKSI_ST2US(n) PIKSI_ST2TIME(n, SECS_US)
#define PIKSI_ST2MS(n) PIKSI_ST2TIME(n, SECS_MS)
#define PIKSI_ST2S(n) PIKSI_ST2TIME(n, 1)

/** Define the maximum values that PIKSI_ST2* macros can handle.
 *
 * Dividend (u64)n * (u64)prefix + (CH_CFG_ST_FREQUENCY - 1) must fit into u64.
 */
#define PIKSI_ST2_LIMIT(prefix) \
  (((u64)-1 - (u64)CH_CFG_ST_FREQUENCY + 1) / (u64)(prefix))

#define PIKSI_ST2US_LIMIT PIKSI_ST2_LIMIT(SECS_US)
#define PIKSI_ST2MS_LIMIT PIKSI_ST2_LIMIT(SECS_MS)
#define PIKSI_ST2S_LIMIT PIKSI_ST2_LIMIT(1)

static inline u64 st2us(u64 st) {
  assert(st <= PIKSI_ST2US_LIMIT);

  return PIKSI_ST2US(st);
}

u64 st2ms(u64 st) {
  assert(st <= PIKSI_ST2MS_LIMIT);

  return PIKSI_ST2MS(st);
}

static inline u64 st2s(u64 st) {
  assert(st <= PIKSI_ST2S_LIMIT);

  return PIKSI_ST2S(st);
}

static inline u64 us2st(u64 us) {
  assert(us <= PIKSI_US2ST_LIMIT);

  return PIKSI_US2ST(us);
}

static inline u64 ms2st(u64 ms) {
  assert(ms <= PIKSI_MS2ST_LIMIT);

  return PIKSI_MS2ST(ms);
}

static inline u64 s2st(u64 s) {
  assert(s <= PIKSI_S2ST_LIMIT);

  return PIKSI_S2ST(s);
}

/** Get system time. Function detects ChibiOS systime rollover and keeps count
 * of them.
 *
 * \note Function works properly only if time between subsequent calls is less
 * or equal to ((systime)-1)
 *
 * \param[out] t            Pointer to piksi_systime_t variable, system time
 *                          will be set to this variable.
 */
static void piksi_systime_get_internal(piksi_systime_t *t) {
  static piksi_systime_t prev = {PIKSI_SYSTIME_SYSTIME_INIT,
                                 PIKSI_SYSTIME_RO_CNT_INIT};

  assert(t);

  systime_t current = chVTGetSystemTimeX();

  if (prev.systime > current) {
    prev.rollover_cnt++;
  }

  t->systime = prev.systime = current;
  t->rollover_cnt = prev.rollover_cnt;
}

/* Lock version */
void piksi_systime_get(piksi_systime_t *t) {
  chSysLock();
  piksi_systime_get_internal(t);
  chSysUnlock();
}

/* No lock version, this should be used if caller already has the lock. */
static void piksi_systime_get_x(piksi_systime_t *t) {
  piksi_systime_get_internal(t);
}

/** Convert piksi_systime to system ticks.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to system ticks.
 */
static u64 piksi_systime_to_ticks(const piksi_systime_t *t) {
  return t->rollover_cnt * ((u64)TIME_INFINITE + 1) + t->systime;
}

/** Convert piksi_systime to microseconds.
 *
 * \note The result is rounded up to the next microsecond boundary.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to microseconds.
 */
u64 piksi_systime_to_us(const piksi_systime_t *t) {
  u64 ticks = piksi_systime_to_ticks(t);

  return st2us(ticks);
}

/** Convert piksi_systime to milliseconds.
 *
 * \note The result is rounded up to the next millisecond boundary.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to milliseconds.
 */
u64 piksi_systime_to_ms(const piksi_systime_t *t) {
  u64 ticks = piksi_systime_to_ticks(t);

  return st2ms(ticks);
}

/** Convert piksi_systime to seconds.
 *
 * \note The result is rounded up to the next second boundary.
 *
 * \param[in] t            Pointer to piksi_systime to convert.
 *
 * \return Value converted to seconds.
 */
u64 piksi_systime_to_s(const piksi_systime_t *t) {
  u64 ticks = piksi_systime_to_ticks(t);

  return st2s(ticks);
}

/** Subtract b from a.
 *
 * \note Function works correctly only if the difference fits to s64.
 *
 * \param[in] a           Minuend.
 * \param[in] b           Subtrahend.
 *
 * \return                Difference.
 */
static s64 piksi_systime_sub_internal(const piksi_systime_t *a,
                                      const piksi_systime_t *b) {
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
 *
 * \param[in] a           Minuend.
 * \param[in] b           Subtrahend.
 *
 * \return a - b result as microseconds.
 */
s64 piksi_systime_sub_us(const piksi_systime_t *a, const piksi_systime_t *b) {
  s64 ticks = piksi_systime_sub_internal(a, b);

  if (ticks < 0) {
    /* Remove sign during conversion */
    return st2us(ticks * (-1)) * (-1);
  } else {
    return st2us(ticks);
  }
}

/** Carry out subtraction and return result as milliseconds.
 *
 * \note The result is rounded up to the next millisecond boundary.
 *
 * \param[in] a           Minuend.
 * \param[in] b           Subtrahend.
 *
 * \return a - b result as milliseconds.
 */
s64 piksi_systime_sub_ms(const piksi_systime_t *a, const piksi_systime_t *b) {
  s64 ticks = piksi_systime_sub_internal(a, b);

  if (ticks < 0) {
    /* Remove sign during conversion */
    return st2ms(ticks * (-1)) * (-1);
  } else {
    return st2ms(ticks);
  }
}

/** Carry out subtraction and return result as seconds.
 *
 * \note The result is rounded up to the next second boundary.
 *
 * \param[in] a           Minuend.
 * \param[in] b           Subtrahend.
 *
 * \return a - b result as seconds.
 */
s64 piksi_systime_sub_s(const piksi_systime_t *a, const piksi_systime_t *b) {
  s64 ticks = piksi_systime_sub_internal(a, b);

  if (ticks < 0) {
    /* Remove sign during conversion */
    return st2s(ticks * (-1)) * (-1);
  } else {
    return st2s(ticks);
  }
}

/** Get tick count since specific time.
 *
 * \param[in] t            Pointer to time variable indicating the start moment.
 *
 * \return System ticks since t.
 */
static u64 piksi_systime_elapsed_since(const piksi_systime_t *t) {
  piksi_systime_t now;

  piksi_systime_get(&now);

  return piksi_systime_sub_internal(&now, t);
}

/** Get microsecond count since specific time.
 *
 * \note The result is rounded up to the next microsecond boundary.
 *
 * \return Microseconds since t.
 */
u64 piksi_systime_elapsed_since_us(const piksi_systime_t *t) {
  u64 ticks = piksi_systime_elapsed_since(t);

  return st2us(ticks);
}

/** Get millisecond count since specific time.
 *
 * \note The result is rounded up to the next millisecond boundary.
 *
 * \return Milliseconds since t.
 */
u64 piksi_systime_elapsed_since_ms(const piksi_systime_t *t) {
  u64 ticks = piksi_systime_elapsed_since(t);

  return st2ms(ticks);
}

/** Get second count since specific time.
 *
 * \note The result is rounded up to the next second boundary.
 *
 * \return Seconds since t.
 */
u64 piksi_systime_elapsed_since_s(const piksi_systime_t *t) {
  u64 ticks = piksi_systime_elapsed_since(t);

  return st2s(ticks);
}

/** Get tick count since specific time. No lock version.
 *
 * \param[in] t            Pointer to time variable indicating the start moment.
 *
 * \return System ticks since t.
 */
static u64 piksi_systime_elapsed_since_x(const piksi_systime_t *t) {
  piksi_systime_t now;

  piksi_systime_get_x(&now);

  return piksi_systime_sub_internal(&now, t);
}

/** Increment piksi_system_t.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           System tick value to be added.
 *
 */
void piksi_systime_inc_internal(piksi_systime_t *t, u64 inc) {
  assert(t);

  if (0 == inc) {
    return;
  }

  /* TODO: modify function to support larger increments */
  assert(inc <= TIME_INFINITE);

  systime_t space = TIME_INFINITE - t->systime;

  if (space < inc) {
    t->rollover_cnt++;
    t->systime = inc - space - 1;
  } else {
    t->systime += inc;
  }
}

/** Increment piksi_system_t with microsecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Microsecond value to be added.
 */
void piksi_systime_inc_us(piksi_systime_t *t, u64 inc) {
  piksi_systime_inc_internal(t, us2st(inc));
}

/** Increment piksi_system_t with millisecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Millisecond value to be added.
 */
void piksi_systime_inc_ms(piksi_systime_t *t, u64 inc) {
  piksi_systime_inc_internal(t, ms2st(inc));
}

/** Increment piksi_system_t with second value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Seconds value to be added.
 */
void piksi_systime_inc_s(piksi_systime_t *t, u64 inc) {
  piksi_systime_inc_internal(t, s2st(inc));
}

/** Decrease piksi_system_t.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           System tick value to be decreased.
 */
void piksi_systime_dec_internal(piksi_systime_t *t, u64 dec) {
  assert(t);

  if (0 == dec) {
    return;
  }

  /* TODO: modify function to support larger decrements */
  assert(dec <= TIME_INFINITE);

  if (dec > t->systime) {
    assert(0 < t->rollover_cnt);
    t->rollover_cnt--;
    dec -= (t->systime + 1);
    t->systime = TIME_INFINITE;
  }

  t->systime -= dec;
}

/** Decrease piksi_system_t with microsecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Microsecond value to be decreased.
 */
void piksi_systime_dec_us(piksi_systime_t *t, u64 dec) {
  piksi_systime_dec_internal(t, us2st(dec));
}

/** Decrease piksi_system_t with millisecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Millisecond value to be decreased.
 */
void piksi_systime_dec_ms(piksi_systime_t *t, u64 dec) {
  piksi_systime_dec_internal(t, ms2st(dec));
}

/** Decrease piksi_system_t with second value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] inc           Second value to be decreased.
 */
void piksi_systime_dec_s(piksi_systime_t *t, u64 dec) {
  piksi_systime_dec_internal(t, s2st(dec));
}

/** Increment/decrement piksi_system_t with signed microsecond value.
 *
 * \note time -> system tick conversion result is rounded upward to the next
 * tick boundary.
 *
 * \param[in,out] t         Pointer to piksi_systime_t variable.
 * \param[in] delta         Microsecond value to be added.
 */
void piksi_systime_add_us(piksi_systime_t *t, s64 delta) {
  if (delta > 0) {
    piksi_systime_inc_internal(t, us2st(delta));
  } else if (delta < 0) {
    piksi_systime_dec_internal(t, us2st(-delta));
  }
}

/** Compare a and b.
 *
 * \param[in] a           1st variable to compare.
 * \param[in] b           2nd variable to compare.
 *
 * \return -1 if a > b; 0 if a == b; 1 if a < b
 */
s8 piksi_systime_cmp(const piksi_systime_t *a, const piksi_systime_t *b) {
  assert(NULL != a && NULL != b);

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
 * \note This is an S-Class API,
 *  this function can be invoked from within a system lock zone by threads only.
 *
 * \param[in] len   Sleep length [system ticks].
 */
void piksi_systime_sleep_s_internal(systime_t len) { chThdSleepS(len); }

void piksi_systime_sleep_us_s(u32 len_us) {
  u64 len_st = us2st(len_us);

  assert(len_st <= TIME_INFINITE);

  piksi_systime_sleep_s_internal(len_st);
}

/** Suspends the invoking thread for the specified time.
 * \note Normal API, this function can be invoked by regular system threads
 *  but not from within a lock zone.
 *
 * \param[in] len   Sleep length [system ticks].
 */
void piksi_systime_sleep_internal(systime_t len) { chThdSleep(len); }

void piksi_systime_sleep_ms(u32 len_ms) {
  u64 len_st = ms2st(len_ms);

  assert(len_st <= TIME_INFINITE);

  piksi_systime_sleep_internal(len_st);
}

/** Suspends the invoking thread until specified moment is reached.
 *
 * \param[in] t   Time to wake up.
 *
 * \return Time spent in sleep [us].
 */
u32 piksi_systime_sleep_until_us(const piksi_systime_t *t) {
  chSysLock();
  piksi_systime_t now;
  piksi_systime_get_x(&now);
  s64 sleep = piksi_systime_sub_internal(t, &now);
  if (sleep > 0) {
    chThdSleepS(sleep);
  } else {
    sleep = 0;
  }
  chSysUnlock();
  return st2us(sleep);
}

/** Suspends the invoking thread until the specified window closes.
 *
 * \param[in] t            Window start.
 * \param[in] window_len   Window length [system ticks].
 *
 * \return Time spent in sleep [system ticks].
 */
systime_t piksi_systime_sleep_until_windowed_internal(const piksi_systime_t *t,
                                                      systime_t window_len) {
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
                                          u32 window_len_us) {
  systime_t ret =
      piksi_systime_sleep_until_windowed_internal(t, us2st(window_len_us));

  return st2us(ret);
}

u32 piksi_systime_sleep_until_windowed_ms(const piksi_systime_t *t,
                                          u32 window_len_ms) {
  systime_t ret =
      piksi_systime_sleep_until_windowed_internal(t, ms2st(window_len_ms));

  return st2ms(ret);
}
