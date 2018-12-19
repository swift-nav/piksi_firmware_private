#include "gtest/gtest.h"

#include <swiftnav/coord_system.h>
#include <swiftnav/linear_algebra.h>
#include "sbas_select/sbas_select.h"

TEST(sbas_select_tests, masks) {
  EXPECT_EQ(sbas_select_prn_mask(SBAS_NONE), 0);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_WAAS), 0x48800);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_EGNOS), 0x10009);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_MSAS), 0x20200);

  EXPECT_EQ(sbas_select_prn_mask(SBAS_GAGAN), 0x1180);

  EXPECT_DEATH(sbas_select_prn_mask(SBAS_COUNT), "");
}

#define LAT_DEG_PER_KM (360 / (2 * (WGS84_A / 1000.) * M_PI))

#define SBAS_SELECT_LAT_POLAR_REGION_KM 50

TEST(sbas_select_tests, provider_static) {
  last_good_fix_t lgf;
  // check UNKNOWN user position
  lgf.position_quality = POSITION_UNKNOWN;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);

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

  // check corners
  lgf.position_solution.pos_llh[0] = 90 * D2R;
  lgf.position_solution.pos_llh[1] = -180 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = 0 * D2R;
  lgf.position_solution.pos_llh[1] = -50 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  lgf.position_solution.pos_llh[0] = 0 * D2R;
  lgf.position_solution.pos_llh[1] = 40 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[0] = 90 * D2R;
  lgf.position_solution.pos_llh[1] = -50 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);
  lgf.position_solution.pos_llh[0] = -20 * D2R;
  lgf.position_solution.pos_llh[1] = 100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  lgf.position_solution.pos_llh[0] = 50 * D2R;
  lgf.position_solution.pos_llh[1] = 40 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  lgf.position_solution.pos_llh[0] = 20 * D2R;
  lgf.position_solution.pos_llh[1] = 160 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);
  lgf.position_solution.pos_llh[0] = 60 * D2R;
  lgf.position_solution.pos_llh[1] = 100 * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_MSAS);

  // check poles vicinity areas
  // First, set system to NONE to clear previous system
  const u8 distance_to_pole_deg =
      (SBAS_SELECT_LAT_POLAR_REGION_KM / 2) * LAT_DEG_PER_KM;
  lgf.position_quality = POSITION_UNKNOWN;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
  lgf.position_quality = POSITION_FIX;

  // check north pole, should default to WAAS if previously NONE
  lgf.position_solution.pos_llh[0] = (90 - distance_to_pole_deg) * D2R;
  lgf.position_solution.pos_llh[1] = 10 * D2R;  // set to EGNOS
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);

  // check south pole, should report NONE
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

  // a. Move towards north over the GAGAN border, staying within hysteresis
  // of GAGAN
  lgf.position_solution.pos_llh[0] = 40.f * D2R;  // set to GAGAN
  lgf.position_solution.pos_llh[1] = 60.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);
  for (lgf.position_solution.pos_llh[0] = 49.f * D2R;
       lgf.position_solution.pos_llh[0] <= 51.f * D2R;
       lgf.position_solution.pos_llh[0] =
           lgf.position_solution.pos_llh[0] + .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_GAGAN);
  }

  // b. Move towards south from NONE to GAGAN, still staying outside
  lgf.position_solution.pos_llh[0] = 51.1f * D2R;  // set to NONE
  lgf.position_solution.pos_llh[1] = 60.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
  for (lgf.position_solution.pos_llh[0] = 51.f * D2R;
       lgf.position_solution.pos_llh[0] > 50.f * D2R;
       lgf.position_solution.pos_llh[0] =
           lgf.position_solution.pos_llh[0] - .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_NONE);
  }

  // b. Move to GAGAN border, should switch without hysteresis
  lgf.position_solution.pos_llh[0] = 50.f * D2R;  // set to WAAS
  lgf.position_solution.pos_llh[1] = 60.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_GAGAN);

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

  // 6) Make system transition within both hysteresis regions

  // a. Make a loop through both hysteresis to another system
  // Set to WAAS, close to equator
  lgf.position_solution.pos_llh[0] = 0.1f * D2R;
  lgf.position_solution.pos_llh[1] = -50.1f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  // Enter WAAS lat hysteresis region
  lgf.position_solution.pos_llh[0] = -0.1f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  // Enter also lon hysteresis region
  lgf.position_solution.pos_llh[1] = -49.9f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  //  Exit lat hysteresis, still within lon hysteresis
  lgf.position_solution.pos_llh[0] = 0.1f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  //  Exit lon hysteresis to EGNOS
  lgf.position_solution.pos_llh[1] = -48.9f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_EGNOS);

  // b. Go through both hysteresis to NONE
  // Reset to WAAS
  lgf.position_solution.pos_llh[0] = 10.f * D2R;
  lgf.position_solution.pos_llh[1] = -100.f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  // Move close to equator
  lgf.position_solution.pos_llh[0] = 0.1f * D2R;
  lgf.position_solution.pos_llh[1] = -50.1f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  // Enter WAAS lat hysteresis region
  lgf.position_solution.pos_llh[0] = -0.1f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  // Enter to lon hysteresis region
  lgf.position_solution.pos_llh[1] = -49.9f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_WAAS);
  // Exit lon hysteresis
  lgf.position_solution.pos_llh[1] = -48.9f * D2R;
  EXPECT_EQ(sbas_select_provider(&lgf), SBAS_NONE);
}

TEST(sbas_select_tests, polar_region) {
  last_good_fix_t lgf;
  lgf.position_quality = POSITION_FIX;
  lgf.position_solution.pos_llh[1] = -80.f * D2R;

  // a. Move towards north entering the polar region from WAAS
  double distance_to_pole_km;
  for (distance_to_pole_km = (SBAS_SELECT_LAT_POLAR_REGION_KM + 1);
       distance_to_pole_km >= (SBAS_SELECT_LAT_POLAR_REGION_KM - 2);
       distance_to_pole_km -= 0.1) {
    lgf.position_solution.pos_llh[0] =
        (90 - distance_to_pole_km * LAT_DEG_PER_KM) * D2R;
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_WAAS);
  }

  // b. Move over to EGNOS within polar region, should stay in WAAS
  for (lgf.position_solution.pos_llh[1] = -51.f * D2R;
       lgf.position_solution.pos_llh[1] <= -48.f * D2R;
       lgf.position_solution.pos_llh[1] =
           lgf.position_solution.pos_llh[1] + .1 * D2R) {
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_WAAS);
  }

  // c. Move out of the polar region into EGNOS
  // First move to EGNOS within polar region
  for (distance_to_pole_km = (SBAS_SELECT_LAT_POLAR_REGION_KM - 1);
       distance_to_pole_km >= SBAS_SELECT_LAT_POLAR_REGION_KM;
       distance_to_pole_km += 0.1) {
    lgf.position_solution.pos_llh[0] =
        (90 - distance_to_pole_km * LAT_DEG_PER_KM) * D2R;
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_WAAS);
  }
  // Then step out of the polar region
  distance_to_pole_km = (SBAS_SELECT_LAT_POLAR_REGION_KM + 0.1);
  lgf.position_solution.pos_llh[0] =
      (90 - distance_to_pole_km * LAT_DEG_PER_KM) * D2R;
  sbas_system_t s = sbas_select_provider(&lgf);
  EXPECT_EQ(s, SBAS_EGNOS);

  // d. Move all the way to polar
  for (distance_to_pole_km = SBAS_SELECT_LAT_POLAR_REGION_KM;
       distance_to_pole_km >= 0.;
       distance_to_pole_km -= 1) {
    lgf.position_solution.pos_llh[0] =
        (90 - distance_to_pole_km * LAT_DEG_PER_KM) * D2R;
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_EGNOS);
  }

  // d. Move on the other side of the polar, finally exiting the polar region
  // First stay within the polar region
  lgf.position_solution.pos_llh[1] = 132.f * D2R;
  for (distance_to_pole_km = 0;
       distance_to_pole_km >= SBAS_SELECT_LAT_POLAR_REGION_KM;
       distance_to_pole_km += 1) {
    lgf.position_solution.pos_llh[0] =
        (90 - distance_to_pole_km * LAT_DEG_PER_KM) * D2R;
    sbas_system_t s = sbas_select_provider(&lgf);
    EXPECT_EQ(s, SBAS_EGNOS);
  }
  // Then step out of the polar region
  distance_to_pole_km = (SBAS_SELECT_LAT_POLAR_REGION_KM + 0.1);
  lgf.position_solution.pos_llh[0] =
      (90 - distance_to_pole_km * LAT_DEG_PER_KM) * D2R;
  s = sbas_select_provider(&lgf);
  EXPECT_EQ(s, SBAS_NONE);  // None of SBAS systems has coverage here
}
