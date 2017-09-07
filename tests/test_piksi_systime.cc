#include <math.h>

#include "gtest/gtest.h"

#include "piksi_systime.h"

#include <libswiftnav/time.h>

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
  EXPECT_EQ(us, ((s64)TIME_INFINITE + 1 + 1) * SECS_US / CH_CFG_ST_FREQUENCY);
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
  // diff should be one positive rollover in ms
  EXPECT_EQ(diff, ((s64)TIME_INFINITE + 1) * SECS_MS / CH_CFG_ST_FREQUENCY);

  diff = piksi_systime_sub_ms(&st1, &st2);
  // diff should be one negative rollover in ms
  EXPECT_EQ(diff, (-(s64)TIME_INFINITE - 1) * SECS_MS / CH_CFG_ST_FREQUENCY);
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
  EXPECT_EQ(diff, -ceil(((s64)TIME_INFINITE - 1) / (double)CH_CFG_ST_FREQUENCY));
}

