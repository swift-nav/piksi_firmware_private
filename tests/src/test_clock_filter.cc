#include "clock_filter/clock_filter.h"
#include "gtest/gtest.h"

// arbitrary clock tick frequency
#define TICK_FREQ 10000000
#define RX_DT_NOMINAL (1.0 / TICK_FREQ)

#define T1 \
  { 1000.0, 1939 }
#define TC1 1000
#define CLOCK_STATE_1                   \
  {                                     \
    TC1, T1, RX_DT_NOMINAL, 1 + 1e-6, { \
      {1e-18, 0}, { 0, 1e-24 }          \
    }                                   \
  }

TEST(clock_filter_tests, propagation) {
  clock_est_state_t init_clock_state = CLOCK_STATE_1;

  // test over a range of increments for numerical problems
  s64 tc_increments[] = {
      0L,
      100L,
      TICK_FREQ / 100,               // 100th of a second
      TICK_FREQ / 10,                // 10th of a second
      TICK_FREQ,                     // second
      600L * TICK_FREQ,              // 10 min
      3600L * TICK_FREQ,             // hour
      7L * 24L * 3600L * TICK_FREQ,  // week
      -10000L                        // negative
  };

  for (u8 i = 0; i < ARRAY_SIZE(tc_increments); i++) {
    clock_est_state_t cs = CLOCK_STATE_1;

    // propagate the clock state for some amount of ticks
    u64 tc = TC1 + tc_increments[i];
    propagate_clock_state(&cs, tc);

    // check the expected change in TOW
    double dt = (tc - TC1) * RX_DT_NOMINAL * cs.clock_rate;
    gps_time_t t2 = T1;
    t2.tow += dt;
    EXPECT_DOUBLE_EQ(cs.t_gps.tow, t2.tow);

    // check that the state uncertainty does not decrease
    EXPECT_GE(cs.P[0][0], init_clock_state.P[0][0]);
    EXPECT_GE(cs.P[1][1], init_clock_state.P[1][1]);
    // symmetry preserved
    EXPECT_DOUBLE_EQ(cs.P[0][1], cs.P[1][0]);
    // for significant time deltas, expect covariance to grow
    if (dt > 0.05) {
      EXPECT_GT(cs.P[0][0], init_clock_state.P[0][0]);
      EXPECT_GT(cs.P[1][1], init_clock_state.P[1][1]);
    }
  }
}

TEST(clock_filter_tests, update_tow) {
  struct testcase {
    double prior_tow, meas_tow, expected_tow;
    double prior_std, meas_std;
  } testcases[] = {
      /* equal prior and measurement variances: estimate is average of the
         prior and measurement */
      {1000, 1000.002, 1000.001, 1e-3, 1e-3},
      /* prior variance very small: estimate is equal to prior */
      {1000, 1000.002, 1000.000, 1e-9, 1},
      /* meas variance very small: estimate is equal to measurement */
      {1000, 1000.002, 1000.002, 1, 1e-9},
      /* zero prior variance: estimate is equal to prior */
      {1000, 1000.002, 1000.000, 0, 1e-9},
      /* zero meas variance: estimate is equal to measurement */
      {1000, 1000.002, 1000.002, 1e-9, 0},
      /* large tow */
      {7 * 24 * 3600 - 2, 7 * 24 * 3600, 7 * 24 * 3600 - 1, 1e-3, 1e-3},
      /* pathologically small noises */
      {1000, 1000.002, 1000.001, 1e-16, 1e-16},
      /* pathologically large noises */
      {1000, 1000.002, 1000.001, 1e50, 1e50},
  };

  for (u8 i = 0; i < ARRAY_SIZE(testcases); i++) {
    clock_est_state_t prior_clock_state = CLOCK_STATE_1;
    prior_clock_state.t_gps.tow = testcases[i].prior_tow;
    prior_clock_state.P[0][0] = pow(testcases[i].prior_std, 2);

    clock_est_state_t cs = prior_clock_state;

    gnss_solution sol;
    sol.time = cs.t_gps;
    sol.time.tow = testcases[i].meas_tow;
    sol.clock_offset_var = pow(testcases[i].meas_std, 2);
    sol.clock_drift = -1e-7;
    sol.clock_drift_var = (1e-12) * (1e-12);

    update_clock_state(&cs, &sol);

    EXPECT_DOUBLE_EQ(cs.t_gps.tow, testcases[i].expected_tow);

    // symmetry preserved
    EXPECT_DOUBLE_EQ(cs.P[0][1], cs.P[1][0]);

    // posterior covariance should be smaller (or equal to) prior
    EXPECT_LE(cs.P[0][0], prior_clock_state.P[0][0]);
    EXPECT_LE(cs.P[1][1], prior_clock_state.P[1][1]);
  }
}

TEST(clock_filter_tests, update_full) {
  // test updating both TOW and drift at the same time
  double prior_drift = -1e-7;
  clock_est_state_t prior_clock_state = CLOCK_STATE_1;
  prior_clock_state.t_gps.tow = 1000;
  prior_clock_state.clock_rate = 1 - prior_drift;
  prior_clock_state.P[0][0] = pow(1e-8, 2);
  prior_clock_state.P[1][1] = pow(1e-11, 2);
  prior_clock_state.P[0][1] = -pow(1e-10, 2);
  prior_clock_state.P[1][0] = -pow(1e-10, 2);

  clock_est_state_t cs = prior_clock_state;

  gnss_solution sol;
  sol.time = cs.t_gps;
  sol.time.tow = 1000.0;
  sol.clock_offset_var = pow(1e-8, 2);
  sol.clock_drift = -2e-7;
  sol.clock_drift_var = pow(1e-12, 2);

  update_clock_state(&cs, &sol);

  // meas tow is same as prior tow, but drift correction should cause also tow
  // to be updated
  EXPECT_NE(cs.t_gps.tow, 1000.0);

  // symmetry preserved
  EXPECT_FLOAT_EQ(cs.P[0][1], cs.P[1][0]);

  // posterior covariance should be smaller (or equal to) prior
  EXPECT_LT(cs.P[0][0], prior_clock_state.P[0][0]);
  EXPECT_LT(cs.P[1][1], prior_clock_state.P[1][1]);
}
