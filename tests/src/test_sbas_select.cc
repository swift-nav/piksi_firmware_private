#include "gtest/gtest.h"

#include <libswiftnav/coord_system.h>
#include <libswiftnav/linear_algebra.h>
#include "sbas_select/sbas_select.h"

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

#define LAT_DEG_PER_KM (360 / (2 * WGS84_A * M_PI))

TEST(sbas_select_tests, provider) {
  last_good_fix_t lgf;
  // check UNKNOWN user position
  lgf.position_quality = POSITION_UNKNOWN;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_UNKNOWN);

  // check WAAS range and test that latitude does not affect
  // if it is not too close to a pole
  lgf.position_quality = POSITION_FIX;
  lgf.position_solution.pos_llh[0] = 30 * D2R;
  lgf.position_solution.pos_llh[1] = -100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = -30 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // check EGNOS range
  lgf.position_solution.pos_llh[1] = 10 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check GAGAN range
  lgf.position_solution.pos_llh[1] = 50 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);

  // check MSAS range
  lgf.position_solution.pos_llh[1] = 120 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);

  // check out of any range
  lgf.position_solution.pos_llh[1] = 170 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_UNKNOWN);

  // check Borders
  lgf.position_solution.pos_llh[1] = -180 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[1] = -50 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[1] = 40 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  lgf.position_solution.pos_llh[1] = 160 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);
  lgf.position_solution.pos_llh[1] = 180 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // check border crossing
  lgf.position_solution.pos_llh[1] = 0.f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  // move through EGNOS-GAGAN border
  for (lgf.position_solution.pos_llh[1] = 39.f * D2R;
       lgf.position_solution.pos_llh[1] <= 41.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] + .1 * D2R) {
    sbas_type_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_EGNOS);
  }
  lgf.position_solution.pos_llh[1] = 41.1f * D2R;  // set to GAGAN
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  for (lgf.position_solution.pos_llh[1] = 41.f * D2R;
       lgf.position_solution.pos_llh[1] >= 39.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] - .1 * D2R) {
    sbas_type_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_GAGAN);
  }
  lgf.position_solution.pos_llh[1] = 38.9f * D2R;  // set to GAGAN
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check longitude debouncing
  lgf.position_solution.pos_llh[1] = 0.f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 40.f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 39.9f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 40.1f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[1] = 39.9f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check poles vicinity areas
  const u8 distance_to_pole_deg = 25 * LAT_DEG_PER_KM;
  lgf.position_solution.pos_llh[1] = 39.9f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check north pole
  lgf.position_solution.pos_llh[0] = (90 - distance_to_pole_deg) * D2R;
  lgf.position_solution.pos_llh[1] = -100 * D2R;  // set to WAAS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check south pole
  lgf.position_solution.pos_llh[0] = (-90 + distance_to_pole_deg) * D2R;
  lgf.position_solution.pos_llh[1] = -100 * D2R;  // set to WAAS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
}
