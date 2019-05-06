#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "gtest/gtest.h"

#include "cn0_est/cn0_est_common.h"

#define CN0_0 40
#define CUTOFF_FREQ 0.1
#define LOOP_DT_MS 1

static s8* generate_input(u32 length, u32 value) {
  s8* input;
  u32 ii = 0;

  input = (s8*)malloc(length);
  if (NULL == input) {
    return NULL;
  }

  for (ii = 0; ii < length; ii++) {
    input[ii] = value;
  }

  return input;
}

TEST(cn0_test, test_cn0_mm_init) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, LOOP_DT_MS);
  cn0_est_mm_state_t cn0;
  cn0_est_mm_init(&cn0, 40.f);
  EXPECT_FLOAT_EQ(cn0.cn0_dbhz, 40.f);
  EXPECT_FLOAT_EQ(p.log_bw, 30.f);
  EXPECT_FLOAT_EQ(cn0.M2, 0.0f);
  EXPECT_FLOAT_EQ(cn0.M4, 0.0f);
  cn0_est_mm_update(&cn0, &p, -0.5, 0.f);
  EXPECT_FLOAT_EQ(cn0.M2, 0.25);
  EXPECT_FLOAT_EQ(cn0.M4, 0.25f * 0.25f);
}

TEST(cn0_test, test_cn0_mm) {
  cn0_est_mm_state_t s;
  cn0_est_params_t p;
  s8* signal_I;
  s8* signal_Q;
  u32 ii = 0;
  u32 test_length = 1000;
  float cn0 = 0.0;

  signal_I = generate_input(test_length, 100);
  ASSERT_NE((s8*)NULL, signal_I) << "Could not allocate I data";
  signal_Q = generate_input(test_length, 50);
  ASSERT_NE((s8*)NULL, signal_Q) << "Could not allocate Q data";

  cn0_est_compute_params(&p, LOOP_DT_MS);
  cn0_est_mm_init(&s, CN0_0);

  for (ii = 0; ii < test_length; ii++) {
    cn0_est_mm_update(&s, &p, signal_I[ii], signal_Q[ii]);
    cn0 = s.cn0_dbhz;
  }

  EXPECT_GT(cn0, 30.0);

  free(signal_I);
  free(signal_Q);
}
