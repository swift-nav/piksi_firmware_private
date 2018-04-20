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

  EXPECT_EQ(sbas_select_prn_mask(SBAS_WAAS), 0x48800);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_EGNOS), 0x10009);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_MSAS), 0x20200);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_GAGAN), 0x1180);

  EXPECT_DEATH(sbas_select_prn_mask(SBAS_COUNT), "");
}

#define LAT_DEG_PER_KM (360 / (2 * WGS84_A * M_PI))

TEST(sbas_select_tests, provider_static) {
  last_good_fix_t lgf;
  // check UNKNOWN user position
  lgf.position_quality = POSITION_UNKNOWN;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_UNKNOWN);

  // check WAAS range and test that latitude does not affect
  // if it is not too close to a pole
  lgf.position_quality = POSITION_FIX;
  lgf.position_solution.pos_llh[0] = 60 * D2R;
  lgf.position_solution.pos_llh[1] = -100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = 20 * D2R;
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
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);

  // check out of any range
  lgf.position_solution.pos_llh[1] = 175 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);

  // check Borders
  lgf.position_solution.pos_llh[0] = 80 * D2R;
  lgf.position_solution.pos_llh[1] = -180 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = 0 * D2R;
  lgf.position_solution.pos_llh[1] = -50 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = 0 * D2R;
  lgf.position_solution.pos_llh[1] = 40 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[0] = 80 * D2R;
  lgf.position_solution.pos_llh[1] = -50 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[0] = 0 * D2R;
  lgf.position_solution.pos_llh[1] = 100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  lgf.position_solution.pos_llh[0] = 50 * D2R;
  lgf.position_solution.pos_llh[1] = 40 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  lgf.position_solution.pos_llh[0] = 0 * D2R;
  lgf.position_solution.pos_llh[1] = 160 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);
  lgf.position_solution.pos_llh[0] = 60 * D2R;
  lgf.position_solution.pos_llh[1] = 100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);
  lgf.position_solution.pos_llh[1] = 180 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // check poles vicinity areas
  const u8 distance_to_pole_deg = 25 * LAT_DEG_PER_KM;
  lgf.position_solution.pos_llh[1] = 39.9f * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // check north pole
  lgf.position_solution.pos_llh[0] = (90 - distance_to_pole_deg) * D2R;
  lgf.position_solution.pos_llh[1] = -100 * D2R;  // set to WAAS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);

  // check south pole
  lgf.position_solution.pos_llh[0] = (-90 + distance_to_pole_deg) * D2R;
  lgf.position_solution.pos_llh[1] = -100 * D2R;  // set to WAAS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
}

TEST(sbas_select_tests, provider_change) {
  last_good_fix_t lgf;
  lgf.position_quality = POSITION_FIX;

  // 1) Check border crossing between two systems

  // a. Move through EGNOS-GAGAN border, should stay within hysteresis of EGNOS
  lgf.position_solution.pos_llh[0] = 20.f * D2R;  // set to EGNOS
  lgf.position_solution.pos_llh[1] = 0.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  for (lgf.position_solution.pos_llh[1] = 39.f * D2R;
       lgf.position_solution.pos_llh[1] <= 41.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] + .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_EGNOS);
  }

  // b. Move through GAGAN-EGNOS border, should stay within hysteresis of GAGAN
  lgf.position_solution.pos_llh[1] = 41.1f * D2R;  // set to GAGAN
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  for (lgf.position_solution.pos_llh[1] = 41.f * D2R;
       lgf.position_solution.pos_llh[1] >= 39.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] - .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_GAGAN);
  }
  // c. Move over the hysteresis back to EGNOS
  lgf.position_solution.pos_llh[1] = 38.9f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // 2) Check border crossing between MSAS and NONE

  // a. Move through MSAS western border, should stay within hysteresis of MSAS
  lgf.position_solution.pos_llh[0] = 20.f * D2R;  // set to MSAS
  lgf.position_solution.pos_llh[1] = 150.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);
  for (lgf.position_solution.pos_llh[1] = 159.f * D2R;
       lgf.position_solution.pos_llh[1] <= 161.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] + .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_MSAS);
  }

  // b. Move back from NONE towards MSAS, still staying outside
  lgf.position_solution.pos_llh[1] = 161.1f * D2R;  // set to NONE
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
  for (lgf.position_solution.pos_llh[1] = 161.1f * D2R;
       lgf.position_solution.pos_llh[1] > 160.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] - .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_NONE);
  }
  // c. Should switch from NONE to MSAS at the border without hysteresis
  lgf.position_solution.pos_llh[1] = 160.0f * D2R;  // set to MSAS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);

  // 3) Check longitude debouncing

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

  // 4) Check border crossing through latitudes at north

  // a. Move towards north over the WAAS border, staying within hysteresis
  // of WAAS
  lgf.position_solution.pos_llh[0] = 70.f * D2R;  // set to WAAS
  lgf.position_solution.pos_llh[1] = -80.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  for (lgf.position_solution.pos_llh[0] = 79.f * D2R;
       lgf.position_solution.pos_llh[0] <= 81.f * D2R;
       lgf.position_solution.pos_llh[0] =
           lgf.position_solution.pos_llh[0] + .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_WAAS);
  }

  // b. Move towards south from NONE to WAAS, still staying outside
  lgf.position_solution.pos_llh[0] = 81.1f * D2R;  // set to NONE
  lgf.position_solution.pos_llh[1] = -80.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
  for (lgf.position_solution.pos_llh[0] = 81.f * D2R;
       lgf.position_solution.pos_llh[0] > 80.f * D2R;
       lgf.position_solution.pos_llh[0] =
           lgf.position_solution.pos_llh[0] - .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_NONE);
  }

  // b. Move to WAAS border, should switch without hysteresis
  lgf.position_solution.pos_llh[0] = 80.f * D2R;  // set to WAAS
  lgf.position_solution.pos_llh[1] = -80.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // 5) Check border crossing through latitudes at south

  // a. Move over the equator on WAAS border, staying within hysteresis
  // of WAAS
  lgf.position_solution.pos_llh[0] = 10.f * D2R;  // set to WAAS
  lgf.position_solution.pos_llh[1] = -80.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  for (lgf.position_solution.pos_llh[0] = 1.f * D2R;
       lgf.position_solution.pos_llh[0] <= -1.f * D2R;
       lgf.position_solution.pos_llh[0] =
           lgf.position_solution.pos_llh[0] - .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_WAAS);
  }

  // b. Move over the equator from NONE to WAAS, still staying outside
  lgf.position_solution.pos_llh[0] = -1.1f * D2R;  // set to NONE
  lgf.position_solution.pos_llh[1] = -80.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
  for (lgf.position_solution.pos_llh[0] = -1.f * D2R;
       lgf.position_solution.pos_llh[0] < 0.f * D2R;
       lgf.position_solution.pos_llh[0] =
           lgf.position_solution.pos_llh[0] + .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_NONE);
  }

  // b. Move to equator, should switch to WAAS without hysteresis
  lgf.position_solution.pos_llh[0] = 0.f * D2R;  // set to WAAS
  lgf.position_solution.pos_llh[1] = -80.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // 6) TODO: Diagonal crossing changing both lat and lon
}
