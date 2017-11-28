#include "gtest/gtest.h"

#include "sbas_select.h"

#undef log_info
#define log_info(...) printf(__VA_ARGS__)

void log_(u8 level, const char *msg, ...) {
  (void)level;
  (void)msg;
  return;
}

TEST(sbas_select_tests, masks) {
  EXPECT_EQ(sbas_select_prn_mask(SBAS_UNKNOWN), 0);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_WAAS), 0x4e804);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_EGNOS), 0x10009);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_MSAS), 0x20200);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_GAGAN), 0x1180);

  EXPECT_DEATH(sbas_select_prn_mask(SBAS_COUNT), "");
}

TEST(sbas_select_tests, provider) {
  last_good_fix_t lgf;
  // check UNKNOWN user position
  lgf.position_quality = POSITION_UNKNOWN;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_UNKNOWN);

  // check WAAS range and test that latitude does not affect
  lgf.position_quality = POSITION_FIX;
  lgf.position_solution.pos_llh[0] = 30;
  lgf.position_solution.pos_llh[1] = -100;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = -30;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // check EGNOS range
  lgf.position_solution.pos_llh[1] = 10;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check GAGAN range
  lgf.position_solution.pos_llh[1] = 50;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);

  // check MSAS range
  lgf.position_solution.pos_llh[1] = 120;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);

  // check out of any range
  lgf.position_solution.pos_llh[1] = 170;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_UNKNOWN);

  // check Borders
  lgf.position_solution.pos_llh[1] = -180;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[1] = -50;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[1] = 40;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 100;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  lgf.position_solution.pos_llh[1] = 160;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);
  lgf.position_solution.pos_llh[1] = 180;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // check border crossing
  lgf.position_solution.pos_llh[1] = 0.f;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  // move through EGNOS-GAGAN border
  for (lgf.position_solution.pos_llh[1] = 39.f;
       lgf.position_solution.pos_llh[1] <= 41.f;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] + .1) {
    sbas_type_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_EGNOS);
  }
  lgf.position_solution.pos_llh[1] = 41.1f;  // set to GAGAN
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  for (lgf.position_solution.pos_llh[1] = 41.f;
       lgf.position_solution.pos_llh[1] >= 39.0f;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] - .1) {
    sbas_type_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_GAGAN);
  }
  lgf.position_solution.pos_llh[1] = 38.9f;  // set to GAGAN
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check debouncing
  lgf.position_solution.pos_llh[1] = 0.f;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 40.f;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 39.9f;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 40.1f;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 39.9f;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
}
