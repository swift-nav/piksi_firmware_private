#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "gtest/gtest.h"

#include "track_loop/trk_loop_common.h"

#define LOW_TOL 1e-6
#define HIGH_TOL 1e-1

TEST(tracking_loop_test, test_costas_discriminator) {
  const struct {
    float i, q;
    float res;
    bool valid;
  } test_cases[] = {
      {.i = 0.0, .q = 0, .res = 0, .valid = true},
      {.i = 0.1, .q = 0, .res = 0, .valid = true},
      {.i = 1.0, .q = M_PI, .res = 0.20095336902385319, .valid = true},
      {.i = 2.0, .q = -M_PI, .res = -0.20095336902385319, .valid = false},
      {.i = 1.0, .q = -M_PI, .res = -0.20095336902385319, .valid = true},
  };

  for (u32 i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
    float res = costas_discriminator(test_cases[i].i, test_cases[i].q);
    /*
    Result is expected to be close to the predicted value only when
    the test is expected to pass.
    */
    EXPECT_TRUE(
        ((fabs(test_cases[i].res - res) < 1e-6) && test_cases[i].valid) ||
        ((fabs(test_cases[i].res - res) > 1e-6) && !test_cases[i].valid));
  }
}

TEST(tracking_loop_test, test_aided) {
  tl_rates_t rates;
  const float code_freq = 6000.3f;
  const float carr_freq = 150000.56f;
  const float acceleration = 5.0f;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  tl_config_t config;
  const float dll_loop_freq = 1000.4f;
  const float fll_loop_freq = 1000.4f;
  const float fll_discr_freq = 1000.4f;
  const float code_bw = 2.0f;
  const float code_zeta = 0.6f;
  const float code_k = 1.0f;
  const float carr_to_code = 1540.0f;
  const float carr_bw = 30.0f;
  const float carr_zeta = 0.7f;
  const float carr_k = 1.0f;
  const float fll_bw = 35.0f;
  const float freq_err = 5.67f;
  config.dll_loop_freq = dll_loop_freq;
  config.fll_loop_freq = fll_loop_freq;
  config.fll_discr_freq = fll_discr_freq;
  config.code_bw = code_bw;
  config.code_zeta = code_zeta;
  config.code_k = code_k;
  config.carr_to_code = carr_to_code;
  config.carr_bw = carr_bw;
  config.carr_zeta = carr_zeta;
  config.carr_k = carr_k;
  config.fll_bw = fll_bw;
  correlation_t cs[3] = {{2.3, 3.4}, {0.0, 0.0}, {7.8, 9.9}};

  tl_pll3_state_t stl_f2p3;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  tl_pll3_init(&stl_f2p3, &rates, &config);
  EXPECT_LT(fabsf(stl_f2p3.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.T_DLL - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.T_FLL - 1.f / fll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.discr_sum_hz), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.discr_period_s - 0.001f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.freq_c1 - 92.452835f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2p3.freq_c2 - 4360.983398f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_c1 - 91.7782058716f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_c2 - 1608.6046142578f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_c3 - 55922.28515625f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_vel), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  tl_pll3_get_rates(&stl_f2p3, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration - acceleration), LOW_TOL);

  tl_pll3_retune(&stl_f2p3, &config);
  EXPECT_LT(fabsf(stl_f2p3.T_FLL - 1.f / fll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.T_DLL - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.freq_c1 - 92.452835f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2p3.freq_c2 - 4360.983398f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2p3.discr_period_s - 0.001f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_c1 - 91.7782058716f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_c2 - 1608.6046142578f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_c3 - 55922.28515625f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  tl_pll3_update_dll(&stl_f2p3, cs, true);
  EXPECT_LT(fabsf(stl_f2p3.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_vel - 0.0039352775f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_freq - 98.6056671143f), LOW_TOL);

  tl_pll3_adjust(&stl_f2p3, freq_err);
  EXPECT_LT(fabsf(stl_f2p3.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_vel - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_freq - 98.6093521118f), LOW_TOL);

  EXPECT_LT(
      fabsf(tl_pll3_get_dll_error(&stl_f2p3) -
            (stl_f2p3.code_freq - stl_f2p3.carr_to_code * stl_f2p3.carr_freq)),
      LOW_TOL);

  tl_pll3_discr_update(&stl_f2p3, cs[1].I, cs[1].Q, true, false);
  EXPECT_LT(fabsf(stl_f2p3.discr_sum_hz), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.prev_Q), LOW_TOL);
}
