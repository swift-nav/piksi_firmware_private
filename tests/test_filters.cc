#include "gtest/gtest.h"

#include "filter/filter_common.h"

#define BW 1000
#define CN0_0 40
#define CUTOFF_FREQ 0.5
#define LOOP_FREQ 1000

TEST(filter_tests, test_lp1_params) {
  lp1_filter_params_t params;
  lp1_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  EXPECT_TRUE(params.a != 0.f);
  EXPECT_TRUE(params.b != 0.f);
}

TEST(filter_tests, test_lp1_init1) {
  lp1_filter_params_t params;
  lp1_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  lp1_filter_t filter;
  lp1_filter_init(&filter, &params, 0.f);

  EXPECT_TRUE(filter.xn == 0.f);
  EXPECT_TRUE(filter.yn == 0.f);
}

/*TEST(filter_tests, test_lp1_init2)
{
  lp1_filter_params_t params;
  lp1_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  lp1_filter_t filter;
  lp1_filter_init(&filter, &params, 1.f);

  EXPECT_TRUE(filter.xn == 1.f * params.b);
  EXPECT_TRUE(filter.yn == 1.f);
}

TEST(filter_tests, test_lp1_update1)
{
  lp1_filter_params_t params;
  lp1_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  lp1_filter_t filter;
  lp1_filter_init(&filter, &params, 1.f);
  float r;

  r = lp1_filter_update(&filter, &params, 1.);
  EXPECT_TRUE(r == 1.f);
}*/

TEST(filter_tests, test_lp1_update2) {
  lp1_filter_params_t params;
  lp1_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  lp1_filter_t filter;
  lp1_filter_init(&filter, &params, 1.f);
  float r;

  r = lp1_filter_update(&filter, &params, 0.f);
  EXPECT_TRUE(r < 1.f);
}

/*TEST(filter_tests, test_lp1_update3)
{
  lp1_filter_params_t params;
  lp1_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  lp1_filter_t filter;
  lp1_filter_init(&filter, &params, 1.f);
  float r;

  r = lp1_filter_update(&filter, &params, 1.1f);
  EXPECT_TRUE(r > 1.f);
}*/

TEST(filter_tests, test_bw2_params) {
  bw2_filter_params_t params;
  bw2_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  EXPECT_TRUE(params.a2 != 0.f);
  EXPECT_TRUE(params.a3 != 0.f);
  EXPECT_TRUE(params.b != 0.f);
}

TEST(filter_tests, test_bw2_init1) {
  bw2_filter_params_t params;
  bw2_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  bw2_filter_t filter;
  bw2_filter_init(&filter, &params, 0.f);

  EXPECT_TRUE(filter.xn == 0.f);
  EXPECT_TRUE(filter.xn_prev == 0.f);
  EXPECT_TRUE(filter.yn == 0.f);
  EXPECT_TRUE(filter.yn_prev == 0.f);
}

/*TEST(filter_tests, test_bw2_init2)
{
  bw2_filter_params_t params;
  bw2_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  bw2_filter_t filter;
  bw2_filter_init(&filter, &params, 1.f);

  EXPECT_TRUE(filter.xn == 1.f);
  EXPECT_TRUE(filter.xn_prev == 1.f);
  EXPECT_TRUE(filter.yn == 1.f);
  EXPECT_TRUE(filter.yn_prev == 1.f);
}*/

/*TEST(filter_tests, test_bw2_update1)
{
  bw2_filter_params_t params;
  bw2_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  bw2_filter_t filter;
  float r;

  bw2_filter_init(&filter, &params, 1.f);
  r = bw2_filter_update(&filter, &params, 1.f);

  EXPECT_TRUE(fabsf(r - 1.f) < 1e-6);
}*/

TEST(filter_tests, test_bw2_update2) {
  bw2_filter_params_t params;
  bw2_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  bw2_filter_t filter;
  float r;

  bw2_filter_init(&filter, &params, 1.f);
  r = bw2_filter_update(&filter, &params, .9);

  EXPECT_TRUE(r < 1.f);
}

/*TEST(filter_tests, test_bw2_update3)
{
  bw2_filter_params_t params;
  bw2_filter_compute_params(&params, CUTOFF_FREQ, LOOP_FREQ);
  bw2_filter_t filter;
  float r;

  bw2_filter_init(&filter, &params, 1.f);
  r = bw2_filter_update(&filter, &params, 1.1f);

  EXPECT_TRUE(r > 1.f);
}*/
