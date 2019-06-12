#include <math.h>
#include <swiftnav/gnss_time.h>

#include "gtest/gtest.h"
#include "piksi_systime.h"

TEST(piksi_systime_tests, init) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  EXPECT_EQ(st.systime, 0);
  EXPECT_EQ(st.rollover_cnt, 0);
}

TEST(piksi_systime_tests, to_us) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  u64 us = piksi_systime_to_us(&st);
  // should be the smallest positive step in us
  EXPECT_EQ(us, ceil((double)SECS_US / CH_CFG_ST_FREQUENCY));

  st.systime = 1;
  st.rollover_cnt = 1;
  us = piksi_systime_to_us(&st);
  // should be one positive rollover + one min step in us
  EXPECT_EQ(us,
            ceil(((s64)TIME_INFINITE + 1 + 1) * (double)SECS_US /
                 CH_CFG_ST_FREQUENCY));

  st.rollover_cnt = -1;
  // result should overflow
  EXPECT_DEATH(piksi_systime_to_us(&st), "");
}

TEST(piksi_systime_tests, to_ms) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  u64 ms = piksi_systime_to_ms(&st);
  // should be the smallest positive step in ms
  EXPECT_EQ(ms, ceil((double)SECS_MS / CH_CFG_ST_FREQUENCY));

  st.systime = 1;
  st.rollover_cnt = 1;
  ms = piksi_systime_to_ms(&st);
  // should be one positive rollover + one min step in ms
  EXPECT_EQ(ms,
            ceil(((s64)TIME_INFINITE + 1 + 1) * (double)SECS_MS /
                 CH_CFG_ST_FREQUENCY));

  st.rollover_cnt = -1;
  // result should overflow
  EXPECT_DEATH(piksi_systime_to_ms(&st), "");
}

TEST(piksi_systime_tests, to_s) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  u64 s = piksi_systime_to_s(&st);
  // should be the smallest positive step in s
  EXPECT_EQ(s, ceil(1.0 / CH_CFG_ST_FREQUENCY));

  st.systime = 1;
  st.rollover_cnt = 1;
  s = piksi_systime_to_s(&st);
  // should be one positive rollover + one min step in s
  EXPECT_EQ(s, ceil(((s64)TIME_INFINITE + 1 + 1) * 1.0 / CH_CFG_ST_FREQUENCY));

  st.rollover_cnt = -1;
  s = piksi_systime_to_s(&st);
  // should be 4,294,967,295 positive rollovers + one min step in s
  u64 expected_ticks = ((u32)(-1) * ((u64)TIME_INFINITE + 1) + 1);
  EXPECT_EQ(s,
            (expected_ticks + (CH_CFG_ST_FREQUENCY - 1)) / CH_CFG_ST_FREQUENCY);
}

TEST(piksi_systime_tests, inc_us) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  // check null pointer
  EXPECT_DEATH(piksi_systime_inc_us(NULL, 1), "");

  // zero increment
  piksi_systime_inc_us(&st, 0);
  EXPECT_EQ(st.systime, 1);
  EXPECT_EQ(st.rollover_cnt, 0);

  // add one
  u64 inc = 1;
  piksi_systime_inc_us(&st, inc);
  EXPECT_EQ(st.systime, ceil(1 + inc * (double)CH_CFG_ST_FREQUENCY / SECS_US));
  EXPECT_EQ(st.rollover_cnt, 0);

  // rollover
  inc = 1;
  st.systime = TIME_INFINITE;
  st.rollover_cnt = 0;
  piksi_systime_inc_us(&st, inc);
  EXPECT_EQ(st.systime, ceil((double)CH_CFG_ST_FREQUENCY / SECS_US - 1));
  EXPECT_EQ(st.rollover_cnt, 1);

  // add minus one -> overflow
  inc = -1;
  EXPECT_DEATH(piksi_systime_inc_us(&st, inc), "");

  // piksi_systime_inc_internal is currently limited to TIME_INFINITE ticks,
  // test TIME_INFINITE + 1
  inc = ceil(1 + TIME_INFINITE * (double)SECS_US / CH_CFG_ST_FREQUENCY);
  EXPECT_DEATH(piksi_systime_inc_us(&st, inc), "");
}

TEST(piksi_systime_tests, inc_ms) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  // check null pointer
  EXPECT_DEATH(piksi_systime_inc_ms(NULL, 1), "");

  // zero increment
  piksi_systime_inc_ms(&st, 0);
  EXPECT_EQ(st.systime, 1);
  EXPECT_EQ(st.rollover_cnt, 0);

  // add one
  u64 inc = 1;
  piksi_systime_inc_ms(&st, inc);
  EXPECT_EQ(st.systime, ceil(1 + inc * (double)CH_CFG_ST_FREQUENCY / SECS_MS));
  EXPECT_EQ(st.rollover_cnt, 0);

  // rollover
  inc = 1;
  st.systime = TIME_INFINITE;
  st.rollover_cnt = 0;
  piksi_systime_inc_ms(&st, inc);
  EXPECT_EQ(st.systime, ceil((double)CH_CFG_ST_FREQUENCY / SECS_MS - 1));
  EXPECT_EQ(st.rollover_cnt, 1);

  // add minus one -> overflow
  inc = -1;
  EXPECT_DEATH(piksi_systime_inc_ms(&st, inc), "");

  // piksi_systime_inc_internal is currently limited to TIME_INFINITE ticks,
  // test TIME_INFINITE + 1
  inc = ceil(1 + TIME_INFINITE * (double)SECS_MS / CH_CFG_ST_FREQUENCY);
  EXPECT_DEATH(piksi_systime_inc_ms(&st, inc), "");
}

TEST(piksi_systime_tests, inc_s) {
  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  // check null pointer
  EXPECT_DEATH(piksi_systime_inc_s(NULL, 1), "");

  // zero increment
  piksi_systime_inc_s(&st, 0);
  EXPECT_EQ(st.systime, 1);
  EXPECT_EQ(st.rollover_cnt, 0);

  // add one
  u64 inc = 1;
  piksi_systime_inc_s(&st, inc);
  EXPECT_EQ(st.systime, ceil(1 + inc * (double)CH_CFG_ST_FREQUENCY));
  EXPECT_EQ(st.rollover_cnt, 0);

  // rollover
  inc = 1;
  st.systime = TIME_INFINITE;
  st.rollover_cnt = 0;
  piksi_systime_inc_s(&st, inc);
  EXPECT_EQ(st.systime, ceil((double)CH_CFG_ST_FREQUENCY) - 1);
  EXPECT_EQ(st.rollover_cnt, 1);

  // add minus one -> overflow
  inc = -1;
  EXPECT_DEATH(piksi_systime_inc_s(&st, inc), "");

  // piksi_systime_inc_internal is currently limited to TIME_INFINITE ticks,
  // test TIME_INFINITE + 1
  inc = ceil(1 + TIME_INFINITE * (double)SECS_MS);
  EXPECT_DEATH(piksi_systime_inc_s(&st, inc), "");
}

TEST(piksi_systime_tests, dec_us) {
  // one tick doesn't equal one micro
  const double min_step = (double)SECS_US / CH_CFG_ST_FREQUENCY;

  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  // check null pointer
  EXPECT_DEATH(piksi_systime_dec_us(NULL, 1), "");

  // zero decrement
  piksi_systime_dec_us(&st, 0);
  EXPECT_EQ(st.systime, 1);
  EXPECT_EQ(st.rollover_cnt, 0);

  // dec one
  u64 dec = 1 * min_step;
  st.systime = 1;
  piksi_systime_dec_us(&st, dec);
  EXPECT_EQ(st.systime, 0);
  EXPECT_EQ(st.rollover_cnt, 0);

  // dec one from 0
  dec = 1 * min_step;
  st.systime = 0;
  st.rollover_cnt = 0;
  EXPECT_DEATH(piksi_systime_dec_us(&st, dec), "");

  // dec one from rollover
  dec = 1 * min_step;
  st.systime = 0;
  st.rollover_cnt = 1;
  piksi_systime_dec_us(&st, dec);
  EXPECT_EQ(st.rollover_cnt, 0);
  EXPECT_EQ(st.systime, TIME_INFINITE);

  // dec two from rollover + 1
  // make sure we take two ticks (one tick reprepsents multiple us increments)
  dec = 2 * min_step;
  st.systime = 1;
  st.rollover_cnt = 1;
  piksi_systime_dec_us(&st, dec);
  EXPECT_EQ(st.rollover_cnt, 0);
  EXPECT_EQ(st.systime, TIME_INFINITE);

  // overflow
  dec = -1;
  EXPECT_DEATH(piksi_systime_dec_us(&st, dec), "");

  // piksi_systime_dec_internal is currently limited to TIME_INFINITE ticks,
  // test TIME_INFINITE + 1
  dec = ceil(1 + TIME_INFINITE * (double)SECS_US / CH_CFG_ST_FREQUENCY);
  EXPECT_DEATH(piksi_systime_dec_us(&st, dec), "");
}

TEST(piksi_systime_tests, dec_ms) {
  // one tick doesn't equal one milli
  const double min_step = (double)CH_CFG_ST_FREQUENCY / SECS_MS;

  piksi_systime_t ref = PIKSI_SYSTIME_INIT;

  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  // check null pointer
  EXPECT_DEATH(piksi_systime_dec_ms(NULL, 1), "");

  // zero decrement
  piksi_systime_dec_ms(&st, 0);
  EXPECT_EQ(st.systime, 1);
  EXPECT_EQ(st.rollover_cnt, 0);

  // dec one
  u64 dec = 1;
  st.systime = 1 * min_step;
  piksi_systime_dec_ms(&st, dec);
  EXPECT_EQ(st.systime, 0);
  EXPECT_EQ(st.rollover_cnt, 0);

  // dec one from 0
  dec = 1;
  st.systime = 0;
  st.rollover_cnt = 0;
  EXPECT_DEATH(piksi_systime_dec_ms(&st, dec), "");

  // dec one ms from rollover
  dec = 1;
  st.systime = 0;
  st.rollover_cnt = 1;
  ref = st;
  piksi_systime_dec_ms(&st, dec);
  EXPECT_EQ(st.rollover_cnt, 0);
  // reference should be larger by the amount of decrement
  EXPECT_EQ(dec, piksi_systime_sub_ms(&ref, &st));

  // dec two from rollover + 1
  // make sure we take two ticks (one tick reprepsents multiple ms increments)
  dec = 2;
  st.systime = 1 * min_step;
  st.rollover_cnt = 1;
  ref = st;
  piksi_systime_dec_ms(&st, dec);
  EXPECT_EQ(st.rollover_cnt, 0);
  // reference should be larger by the amount of decrement
  EXPECT_EQ(dec, piksi_systime_sub_ms(&ref, &st));

  // overflow
  dec = -1;
  EXPECT_DEATH(piksi_systime_dec_ms(&st, dec), "");

  // piksi_systime_dec_internal is currently limited to TIME_INFINITE ticks,
  // test TIME_INFINITE + 1
  dec = ceil(1 + TIME_INFINITE * (double)SECS_MS / CH_CFG_ST_FREQUENCY);
  EXPECT_DEATH(piksi_systime_dec_ms(&st, dec), "");
}

TEST(piksi_systime_tests, dec_s) {
  // one tick doesn't equal one second
  const double min_step = (double)CH_CFG_ST_FREQUENCY;

  piksi_systime_t ref = PIKSI_SYSTIME_INIT;

  piksi_systime_t st = PIKSI_SYSTIME_INIT;
  st.systime = 1;

  // check null pointer
  EXPECT_DEATH(piksi_systime_dec_s(NULL, 1), "");

  // zero decrement is always approved
  piksi_systime_dec_s(&st, 0);
  EXPECT_EQ(st.systime, 1);
  EXPECT_EQ(st.rollover_cnt, 0);

  // dec one
  u64 dec = 1;
  st.systime = 1 * min_step;
  piksi_systime_dec_s(&st, dec);
  EXPECT_EQ(st.systime, 0);
  EXPECT_EQ(st.rollover_cnt, 0);

  // dec one from 0
  dec = 1;
  st.systime = 0;
  st.rollover_cnt = 0;
  EXPECT_DEATH(piksi_systime_dec_s(&st, dec), "");

  // dec one s from rollover
  dec = 1;
  st.systime = 0;
  st.rollover_cnt = 1;
  ref = st;
  piksi_systime_dec_s(&st, dec);
  EXPECT_EQ(st.rollover_cnt, 0);
  // reference should be larger by the amount of decrement
  EXPECT_EQ(dec, piksi_systime_sub_s(&ref, &st));

  // dec two from rollover + 1
  // make sure we take two ticks
  dec = 2;
  st.systime = 1 * min_step;
  st.rollover_cnt = 1;
  ref = st;
  piksi_systime_dec_s(&st, dec);
  EXPECT_EQ(st.rollover_cnt, 0);
  // reference should be larger by the amount of decrement
  EXPECT_EQ(dec, piksi_systime_sub_s(&ref, &st));

  // overflow
  dec = -1;
  EXPECT_DEATH(piksi_systime_dec_s(&st, dec), "");

  // piksi_systime_dec_internal is currently limited to TIME_INFINITE ticks,
  // test TIME_INFINITE + 1
  dec = ceil(1 + TIME_INFINITE * 1.0 / CH_CFG_ST_FREQUENCY);
  EXPECT_DEATH(piksi_systime_dec_s(&st, dec), "");
}

TEST(piksi_systime_tests, sub_us) {
  piksi_systime_t st1 = PIKSI_SYSTIME_INIT;
  st1.systime = 10;
  piksi_systime_t st2 = PIKSI_SYSTIME_INIT;
  st2.systime = 11;

  s64 diff = piksi_systime_sub_us(&st2, &st1);
  // diff should be the smallest positive step in us
  EXPECT_EQ(diff, ceil((double)SECS_US / CH_CFG_ST_FREQUENCY));

  diff = piksi_systime_sub_us(&st1, &st2);
  // diff should be the smallest negative step in us
  EXPECT_EQ(diff, -ceil((double)SECS_US / CH_CFG_ST_FREQUENCY));

  st2.systime = 10;
  st2.rollover_cnt = 1;

  diff = piksi_systime_sub_us(&st2, &st1);
  // diff should be one positive rollover in us
  EXPECT_EQ(diff, ((s64)TIME_INFINITE + 1) * SECS_US / CH_CFG_ST_FREQUENCY);

  diff = piksi_systime_sub_us(&st1, &st2);
  // diff should be one negative rollover in us
  EXPECT_EQ(diff, (-(s64)TIME_INFINITE - 1) * SECS_US / CH_CFG_ST_FREQUENCY);
}

TEST(piksi_systime_tests, sub_ms) {
  piksi_systime_t st1 = PIKSI_SYSTIME_INIT;
  st1.systime = 10;
  piksi_systime_t st2 = PIKSI_SYSTIME_INIT;
  st2.systime = 11;

  s64 diff = piksi_systime_sub_ms(&st2, &st1);
  // diff should be the smallest positive step in ms
  EXPECT_EQ(diff, ceil((double)SECS_MS / CH_CFG_ST_FREQUENCY));

  diff = piksi_systime_sub_ms(&st1, &st2);
  // diff should be the smallest negative step in ms
  EXPECT_EQ(diff, -ceil((double)SECS_MS / CH_CFG_ST_FREQUENCY));

  st2.systime = 10;
  st2.rollover_cnt = 1;

  diff = piksi_systime_sub_ms(&st2, &st1);
  u64 freq_hz = (u64)CH_CFG_ST_FREQUENCY;
  s64 expected_ticks = (s64)TIME_INFINITE + 1;
  s64 expected_ms = (expected_ticks * SECS_MS + (freq_hz - 1)) / freq_hz;
  // diff should be one positive rollover in ms
  EXPECT_EQ(diff, expected_ms);

  diff = piksi_systime_sub_ms(&st1, &st2);
  expected_ticks = -(s64)TIME_INFINITE - 1;
  expected_ms = (expected_ticks * SECS_MS - (s64)(freq_hz - 1)) / (s64)freq_hz;
  // diff should be one negative rollover in ms
  EXPECT_EQ(diff, expected_ms);
}

TEST(piksi_systime_tests, sub_s) {
  piksi_systime_t st1 = PIKSI_SYSTIME_INIT;
  st1.systime = 10;
  piksi_systime_t st2 = PIKSI_SYSTIME_INIT;
  st2.systime = 11;

  s64 diff = piksi_systime_sub_s(&st2, &st1);
  // diff should be the smallest positive step in s
  EXPECT_EQ(diff, ceil(1.0 / CH_CFG_ST_FREQUENCY));

  diff = piksi_systime_sub_s(&st1, &st2);
  // diff should be the smallest negative step in s
  EXPECT_EQ(diff, -ceil(1.0 / CH_CFG_ST_FREQUENCY));

  st2.systime = 10;
  st2.rollover_cnt = 1;

  diff = piksi_systime_sub_s(&st2, &st1);
  // diff should be one positive rollover in s
  EXPECT_EQ(diff, ceil(((s64)TIME_INFINITE + 1) / (double)CH_CFG_ST_FREQUENCY));

  diff = piksi_systime_sub_s(&st1, &st2);
  // diff should be one negative rollover in s
  EXPECT_EQ(diff,
            -ceil(((s64)TIME_INFINITE - 1) / (double)CH_CFG_ST_FREQUENCY));
}

TEST(piksi_systime_tests, cmp) {
  piksi_systime_t st1 = PIKSI_SYSTIME_INIT;
  st1.systime = 10;
  piksi_systime_t st2 = PIKSI_SYSTIME_INIT;
  st2.systime = 11;

  s8 res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res > 0);

  res = piksi_systime_cmp(&st2, &st1);
  EXPECT_TRUE(res < 0);

  st1.systime = 11;
  res = piksi_systime_cmp(&st2, &st1);
  EXPECT_TRUE(res == 0);

  st2.rollover_cnt = 1;
  res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res > 0);

  res = piksi_systime_cmp(&st2, &st1);
  EXPECT_TRUE(res < 0);

  EXPECT_DEATH(piksi_systime_cmp(NULL, &st2), "");
  EXPECT_DEATH(piksi_systime_cmp(&st1, NULL), "");
  EXPECT_DEATH(piksi_systime_cmp(NULL, NULL), "");
}

TEST(piksi_systime_tests, misc) {
  piksi_systime_t st1 = PIKSI_SYSTIME_INIT;
  piksi_systime_t st2 = PIKSI_SYSTIME_INIT;

  piksi_systime_inc_us(&st1, 1);
  s8 res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res < 0);
  s64 diff = piksi_systime_sub_us(&st1, &st2);
  piksi_systime_dec_us(&st1, diff);
  res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res == 0);

  piksi_systime_inc_ms(&st1, 1);
  res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res < 0);
  diff = piksi_systime_sub_ms(&st1, &st2);
  piksi_systime_dec_ms(&st1, diff);
  res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res == 0);

  piksi_systime_inc_s(&st1, 1);
  res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res < 0);
  diff = piksi_systime_sub_s(&st1, &st2);
  piksi_systime_dec_s(&st1, diff);
  res = piksi_systime_cmp(&st1, &st2);
  EXPECT_TRUE(res == 0);
}
