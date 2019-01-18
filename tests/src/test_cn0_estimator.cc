#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "gtest/gtest.h"

#include "cn0_est/cn0_est_common.h"

#define CN0_0 40
#define CUTOFF_FREQ 0.1
#define LOOP_FREQ 1000

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

TEST(cn0_test, test_cn0_bl_init) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  EXPECT_FLOAT_EQ(p.log_bw, 30.f);

  cn0_est_bl_state_t cn0;
  cn0_est_bl_init(&cn0, &p, 0.1f);
  EXPECT_FLOAT_EQ(cn0.nsr, 977.237f);
  EXPECT_FLOAT_EQ(cn0.I_prev_abs, -1.f);
  EXPECT_FLOAT_EQ(cn0.Q_prev_abs, -1.f);
  cn0_est_bl_update(&cn0, &p, -1.f, 0.f);  // I=-1; Q=0; RES=CN0_0(40)
  EXPECT_FLOAT_EQ(cn0.I_prev_abs, 1.f);
  EXPECT_FLOAT_EQ(cn0.Q_prev_abs, -1.f);
}

TEST(cn0_test, test_cn0_bl) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_bl_state_t s;
  s8* signal_I;
  s8* signal_Q;
  u32 ii = 0;
  u32 test_length = 1000;
  float cn0 = 0.0;

  signal_I = generate_input(test_length, 100);
  ASSERT_NE((s8*)NULL, signal_I) << "Could not allocate I data";
  signal_Q = generate_input(test_length, 50);
  ASSERT_NE((s8*)NULL, signal_Q) << "Could not allocate Q data";

  cn0_est_bl_init(&s, &p, CN0_0);

  for (ii = 0; ii < test_length; ii++) {
    cn0 = cn0_est_bl_update(&s, &p, signal_I[ii], signal_Q[ii]);
  }

  EXPECT_GT(cn0, 30.0);

  free(signal_I);
  free(signal_Q);
}

TEST(cn0_test, test_cn0_snv_init) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_snv_state_t cn0;
  cn0_est_snv_init(&cn0, &p, 40.f);
  EXPECT_FLOAT_EQ(cn0.cn0_db, 40.f);
  EXPECT_FLOAT_EQ(p.log_bw, 30.f);
  EXPECT_FLOAT_EQ(cn0.I_sum, 0.f);
  EXPECT_FLOAT_EQ(cn0.P_tot, 0.f);
  cn0_est_snv_update(&cn0, &p, -1.f, 0.f);  // I=-1; Q=0; RES=CN0_0(40)
  EXPECT_FLOAT_EQ(cn0.I_sum, 1.f);
  EXPECT_FLOAT_EQ(cn0.P_tot, 1.f);
}

TEST(cn0_test, test_cn0_snv) {
  cn0_est_snv_state_t s;
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

  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_snv_init(&s, &p, CN0_0);

  for (ii = 0; ii < test_length; ii++) {
    cn0 = cn0_est_snv_update(&s, &p, signal_I[ii], signal_Q[ii]);
  }

  EXPECT_GT(cn0, 30.0);

  free(signal_I);
  free(signal_Q);
}

TEST(cn0_test, test_cn0_nwpr_init) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_nwpr_state_t cn0;
  cn0_est_nwpr_init(&cn0, &p, 40.f);
  EXPECT_FLOAT_EQ(cn0.cn0_db, 40.f);
  EXPECT_FLOAT_EQ(p.log_bw, 30.f);
  EXPECT_FLOAT_EQ(cn0.WBP, 0.f);
  EXPECT_FLOAT_EQ(cn0.NBP_I, 0.f);
  EXPECT_FLOAT_EQ(cn0.NBP_Q, 0.f);
  EXPECT_FLOAT_EQ(cn0.mu, 0.f);
  cn0_est_nwpr_update(&cn0, &p, -0.5, 0.f);
  EXPECT_FLOAT_EQ(cn0.WBP, 0.25);
  EXPECT_FLOAT_EQ(cn0.NBP_I, -0.5);
  EXPECT_FLOAT_EQ(cn0.NBP_Q, -0.f);
}

TEST(cn0_test, test_cn0_nwpr) {
  cn0_est_nwpr_state_t s;
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

  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_nwpr_init(&s, &p, CN0_0);

  for (ii = 0; ii < test_length; ii++) {
    cn0 = cn0_est_nwpr_update(&s, &p, signal_I[ii], signal_Q[ii]);
  }

  EXPECT_GT(cn0, 30.0);

  free(signal_I);
  free(signal_Q);
}

TEST(cn0_test, test_cn0_rscn_init) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_rscn_state_t cn0;
  cn0_est_rscn_init(&cn0, &p, 40.f);
  EXPECT_FLOAT_EQ(cn0.cn0_db, 40.f);
  EXPECT_FLOAT_EQ(p.log_bw, 30.f);
  EXPECT_FLOAT_EQ(cn0.Q_sum, 0.f);
  EXPECT_FLOAT_EQ(cn0.P_tot, 0.f);
  cn0_est_rscn_update(&cn0, &p, -0.5, 0.f);
  EXPECT_FLOAT_EQ(cn0.Q_sum, 0.f);
  EXPECT_FLOAT_EQ(cn0.P_tot, 0.25);
}

TEST(cn0_test, test_cn0_rscn) {
  cn0_est_rscn_state_t s;
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

  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_rscn_init(&s, &p, CN0_0);

  for (ii = 0; ii < test_length; ii++) {
    cn0 = cn0_est_rscn_update(&s, &p, signal_I[ii], signal_Q[ii]);
  }

  EXPECT_GT(cn0, 30.0);

  free(signal_I);
  free(signal_Q);
}

TEST(cn0_test, test_cn0_basic_init) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  EXPECT_FLOAT_EQ(p.log_bw, 30.f);

  cn0_est_basic_state_t cn0;
  cn0_est_basic_init(&cn0, &p, 40.f, 1);
  EXPECT_FLOAT_EQ(cn0.cn0_db, 40.f);
  EXPECT_FLOAT_EQ(cn0.noise_Q_abs, 1.f);
}

TEST(cn0_test, test_cn0_basic) {
  cn0_est_params_t p;
  cn0_est_compute_params(&p, CUTOFF_FREQ, LOOP_FREQ, 1, 0);
  cn0_est_basic_state_t s;
  s8* signal_I;
  s8* signal_Q;
  s8* noise_Q;
  u32 ii = 0;
  u32 test_length = 1000;
  float cn0 = 0.0;

  signal_I = generate_input(test_length, 100);
  ASSERT_NE((s8*)NULL, signal_I) << "Could not allocate I data";
  signal_Q = generate_input(test_length, 50);
  ASSERT_NE((s8*)NULL, signal_Q) << "Could not allocate Q data";
  noise_Q = generate_input(test_length, 2);
  ASSERT_NE((s8*)NULL, noise_Q) << "Could not allocate Q data";

  cn0_est_basic_init(&s, &p, CN0_0, 8);
  p.t_int = 1000;

  for (ii = 0; ii < test_length; ii++) {
    cn0 = cn0_est_basic_update(&s,
                               &p,
                               signal_I[ii],
                               signal_Q[ii],
                               /*Noise_I is not used in Basic Estimator,
                                * just put noise_Q instead */
                               noise_Q[ii],
                               noise_Q[ii]);
  }
  EXPECT_GT(cn0, 30.0);

  free(signal_I);
  free(signal_Q);
  free(noise_Q);
}
