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
  aided_lf_state_t s;
  float y = -.4f;
  float b0 = -.5f;
  float b1 = -.6f;
  float ai = 100.f;

  aided_lf_init(&s, y, b0, b1, ai);
  EXPECT_LT(fabsf(s.aiding_igain - ai), LOW_TOL);
  EXPECT_LT(fabsf(s.b0 - b0), LOW_TOL);
  EXPECT_LT(fabsf(s.b1 - b1), LOW_TOL);
  EXPECT_LT(fabsf(s.y - y), LOW_TOL);

  float p_i_error = 0.6f;
  float aiding_error = 0.5f;
  s.prev_error = 0.5f;

  aided_lf_update(&s, p_i_error, aiding_error);
  EXPECT_LT(fabsf(s.y - 49.0f), LOW_TOL);
  EXPECT_LT(fabsf(s.prev_error - p_i_error), LOW_TOL);

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

  aided_tl_state_t stl;
  memcpy(&stl.carr_filt, &s, sizeof(s));
  aided_tl_init(&stl,
                dll_loop_freq,
                code_freq,
                code_bw,
                code_zeta,
                code_k,
                carr_to_code,
                carr_freq,
                carr_bw,
                carr_zeta,
                carr_k,
                fll_bw);
  EXPECT_LT(fabsf(stl.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl.code_freq - code_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(stl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_to_code - carr_to_code), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_filt.aiding_igain - fll_bw), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_filt.prev_error), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_filt.y - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl.code_filt.prev_error), LOW_TOL);
  EXPECT_LT(fabsf(stl.code_filt.y), LOW_TOL);

  aided_tl_update(&stl, cs, false);
  EXPECT_LT(fabsf(stl.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl.code_freq - 98.6056671143f), LOW_TOL);

  aided_tl_retune(&stl,
                  dll_loop_freq,
                  code_bw,
                  code_zeta,
                  code_k,
                  carr_to_code,
                  carr_bw,
                  carr_zeta,
                  carr_k,
                  fll_bw);
  EXPECT_LT(fabsf(stl.carr_filt.aiding_igain - fll_bw), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_to_code - carr_to_code), LOW_TOL);

  aided_tl_adjust(&stl, freq_err);
  EXPECT_LT(fabsf(stl.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl.carr_filt.y - (carr_freq + freq_err)), LOW_TOL);

  EXPECT_LT(fabsf(aided_tl_get_dll_error(&stl) - stl.code_filt.y), LOW_TOL);

  aided_tl_state_fll1_pll2_t stl_f1p2;
  aided_tl_fll1_pll2_init(&stl_f1p2, &rates, &config);
  EXPECT_LT(fabsf(stl_f1p2.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.discr_mul - 159.2185974121f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.freq_a0 - 0.0676067173f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_c1 - 79.4594573975f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_c2 - 3221.3293457031f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_vel), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_fll1_pll2_get_rates(&stl_f1p2, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration), LOW_TOL);

  aided_tl_fll1_pll2_retune(&stl_f1p2, &config);
  EXPECT_LT(fabsf(stl_f1p2.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.freq_a0 - 0.0676067173f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.discr_mul - 159.2185974121f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_c1 - 79.4594573975f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_c2 - 3221.3293457031f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_fll1_pll2_update_fll(&stl_f1p2);
  EXPECT_LT(fabsf(stl_f1p2.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_vel - carr_freq), LOW_TOL);

  aided_tl_fll1_pll2_update_dll(&stl_f1p2, cs);
  EXPECT_LT(fabsf(stl_f1p2.carr_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_vel - 0.0039352775f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_freq - 98.607635498f), LOW_TOL);

  aided_tl_fll1_pll2_adjust(&stl_f1p2, freq_err);
  EXPECT_LT(fabsf(stl_f1p2.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.carr_vel - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.code_freq - 98.6113204956f), LOW_TOL);

  EXPECT_LT(
      fabsf(aided_tl_fll1_pll2_get_dll_error(&stl_f1p2) -
            (stl_f1p2.code_freq - stl_f1p2.carr_to_code * stl_f1p2.carr_freq)),
      LOW_TOL);

  aided_tl_fll1_pll2_discr_update(&stl_f1p2, cs[1].I, cs[1].Q, true, false);
  EXPECT_LT(fabsf(stl_f1p2.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1p2.prev_Q), LOW_TOL);

  aided_tl_state_fll2_pll3_t stl_f2p3;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  aided_tl_fll2_pll3_init(&stl_f2p3, &rates, &config);
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

  aided_tl_fll2_pll3_get_rates(&stl_f2p3, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration - acceleration), LOW_TOL);

  aided_tl_fll2_pll3_retune(&stl_f2p3, &config);
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

  aided_tl_fll2_pll3_update_fll(&stl_f2p3);
  EXPECT_LT(fabsf(stl_f2p3.discr_sum_hz), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_vel - carr_freq), LOW_TOL);

  aided_tl_fll2_pll3_update_dll(&stl_f2p3, cs, true);
  EXPECT_LT(fabsf(stl_f2p3.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_vel - 0.0039352775f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_freq - 98.6056671143f), LOW_TOL);

  aided_tl_fll2_pll3_adjust(&stl_f2p3, freq_err);
  EXPECT_LT(fabsf(stl_f2p3.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.carr_vel - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.code_freq - 98.6093521118f), LOW_TOL);

  EXPECT_LT(
      fabsf(aided_tl_fll2_pll3_get_dll_error(&stl_f2p3) -
            (stl_f2p3.code_freq - stl_f2p3.carr_to_code * stl_f2p3.carr_freq)),
      LOW_TOL);

  aided_tl_fll2_pll3_discr_update(&stl_f2p3, cs[1].I, cs[1].Q, true, false);
  EXPECT_LT(fabsf(stl_f2p3.discr_sum_hz), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2p3.prev_Q), LOW_TOL);

  aided_tl_state_fll1_t stl_f1;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  aided_tl_fll1_init(&stl_f1, &rates, &config);
  EXPECT_LT(fabsf(stl_f1.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.discr_mul - 159.2185974121f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.freq_a0 - 0.0676067173f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.freq_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_prev), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_vel), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_fll1_get_rates(&stl_f1, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration), LOW_TOL);

  aided_tl_fll1_retune(&stl_f1, &config);
  EXPECT_LT(fabsf(stl_f1.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.freq_a0 - 0.0676067173f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.discr_mul - 159.2185974121f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_fll1_update_fll(&stl_f1);
  EXPECT_LT(fabsf(stl_f1.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.freq_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.carr_freq - carr_freq), LOW_TOL);

  aided_tl_fll1_update_dll(&stl_f1, cs);
  EXPECT_LT(fabsf(stl_f1.code_vel - 0.0039352775f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_freq - 98.607635498f), LOW_TOL);

  aided_tl_fll1_adjust(&stl_f1, freq_err);
  EXPECT_LT(fabsf(stl_f1.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.freq_vel - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.code_freq - 98.6113204956f), LOW_TOL);

  EXPECT_LT(fabsf(aided_tl_fll1_get_dll_error(&stl_f1) -
                  (stl_f1.code_freq - stl_f1.carr_to_code * stl_f1.carr_freq)),
            LOW_TOL);

  aided_tl_fll1_discr_update(&stl_f1, cs[1].I, cs[1].Q, true, false);
  EXPECT_LT(fabsf(stl_f1.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl_f1.prev_Q), LOW_TOL);

  aided_tl_state_fll2_t stl_f2;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  aided_tl_fll2_init(&stl_f2, &rates, &config);
  EXPECT_LT(fabsf(stl_f2.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.T_DLL - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.T_FLL - 1.f / fll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.discr_mul - 159.2185974121f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_prev), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_a0 - 44.1823005676f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_a1 - 998.1625366211f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_prev), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_vel), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_fll2_get_rates(&stl_f2, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration - acceleration), LOW_TOL);

  aided_tl_fll2_retune(&stl_f2, &config);
  EXPECT_LT(fabsf(stl_f2.T_FLL - 1.f / fll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.T_DLL - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_a0 - 44.1823005676f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_a1 - 998.1625366211f), HIGH_TOL);
  EXPECT_LT(fabsf(stl_f2.discr_mul - 159.2185974121f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_c1 - 4.7213115692f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_c2 - 15.4797105789f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_fll2_update_fll(&stl_f2);
  EXPECT_LT(fabsf(stl_f2.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.carr_freq - carr_freq), LOW_TOL);

  aided_tl_fll2_update_dll(&stl_f2, cs);
  EXPECT_LT(fabsf(stl_f2.code_vel - 0.0039352775f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_freq - 98.607635498f), LOW_TOL);

  aided_tl_fll2_adjust(&stl_f2, freq_err);
  EXPECT_LT(fabsf(stl_f2.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.freq_vel - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_freq - 98.6113204956f), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.code_prev), LOW_TOL);

  EXPECT_LT(fabsf(aided_tl_fll2_get_dll_error(&stl_f2) -
                  (stl_f2.code_freq - stl_f2.carr_to_code * stl_f2.carr_freq)),
            LOW_TOL);

  aided_tl_fll2_discr_update(&stl_f2, cs[1].I, cs[1].Q, true, false);
  EXPECT_LT(fabsf(stl_f2.discr_sum), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(stl_f2.prev_Q), LOW_TOL);

  aided_tl_state3_t s3tl;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  aided_tl_init3(&s3tl, &rates, &config);
  EXPECT_LT(fabsf(s3tl.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.freq_c1 - 92.452835083f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.freq_c2 - 4360.9833984375f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_c1 - 91.7782058716f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_c2 - 1608.6046142578f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_c3 - 55922.28515625f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_c1 - 4.5283021927f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_c2 - 14.239944458f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_vel), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_get_rates3(&s3tl, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration - acceleration), LOW_TOL);

  aided_tl_retune3(&s3tl, &config);
  EXPECT_LT(fabsf(s3tl.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.freq_c1 - 92.452835083f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.freq_c2 - 4360.9833984375f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_c1 - 91.7782058716f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_c2 - 1608.6046142578f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_c3 - 55922.28515625f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_c1 - 4.5283021927f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_c2 - 14.239944458f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_update_dll3(&s3tl, cs, true);
  EXPECT_LT(fabsf(s3tl.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_vel - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_acc - acceleration), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_freq - 98.556427002f), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_vel - 0.0036201021f), LOW_TOL);

  aided_tl_adjust3(&s3tl, freq_err);
  EXPECT_LT(fabsf(s3tl.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.phase_vel - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(s3tl.code_vel - 0.0036201021f), LOW_TOL);

  EXPECT_LT(fabsf(aided_tl_get_dll_error3(&s3tl) -
                  (s3tl.code_freq - s3tl.carr_to_code * s3tl.carr_freq)),
            LOW_TOL);

  aided_tl_state3b_t s3btl;
  rates.code_freq = code_freq;
  rates.carr_freq = carr_freq;
  rates.acceleration = acceleration;
  aided_tl_init3b(&s3btl, &rates, &config);
  EXPECT_LT(fabsf(s3btl.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_freq - code_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.freq_prev), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.freq_b0 - 94.8941192627f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.freq_b1 - -90.5112915039f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_prev0), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_prev1), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_b0 - 93.4420471191f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_b1 - -185.1643676758f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_b2 - 91.7782058716f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_sum_a - 0.0049980008f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_sum_b - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_prev), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_b0 - 4.7290487289f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_b1 - -4.7135748863f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_sum - code_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_get_rates3b(&s3btl, &rates);
  EXPECT_LT(fabsf(rates.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.code_freq - code_freq), LOW_TOL);
  EXPECT_LT(fabsf(rates.acceleration - acceleration), LOW_TOL);

  aided_tl_retune3b(&s3btl, &config);
  EXPECT_LT(fabsf(s3btl.freq_prev), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_prev - 0.0f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_prev0), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_prev1), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_sum_a - 0.0049980008f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.prev_I - 1.0f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.T - 1.f / dll_loop_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.freq_b0 - 94.8941192627f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.freq_b1 - -90.5112915039f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_b0 - 93.4420471191f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_b1 - -185.1643676758f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_b2 - 91.7782058716f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_b0 - 4.7290487289f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_b1 - -4.7135748863f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.carr_to_code - 1.f / carr_to_code), LOW_TOL);

  aided_tl_update_dll3b(&s3btl, cs, false);
  EXPECT_LT(fabsf(s3btl.prev_I), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.prev_Q), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_sum_a - 0.0049980008f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_sum_b - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_prev1), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_prev0), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.freq_prev), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.carr_freq - carr_freq), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_prev - 0.2543233335f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_sum - 6001.5024414062f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_freq - 6098.9052734375f), LOW_TOL);

  aided_tl_adjust3b(&s3btl, freq_err);
  EXPECT_LT(fabsf(s3btl.carr_freq - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.phase_sum_b - (carr_freq + freq_err)), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_sum - 14733.302734375f), LOW_TOL);
  EXPECT_LT(fabsf(s3btl.code_prev), LOW_TOL);

  EXPECT_LT(fabsf(aided_tl_get_dll_error3b(&s3btl) - s3btl.code_sum), LOW_TOL);
}
