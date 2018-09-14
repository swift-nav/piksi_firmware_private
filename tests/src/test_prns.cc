#include "gtest/gtest.h"
#include "soft_macq/prns.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <swiftnav/constants.h>

TEST(test_prns_sid_to_init_g1, init) {
  u32 g1;
  me_gnss_signal_t mesid;

  mesid = construct_mesid(CODE_GPS_L2CM, 25);
  g1 = mesid_to_lfsr0_init(mesid, 0);
  EXPECT_EQ(0700274134, g1);

  mesid = construct_mesid(CODE_GPS_L2CM, 32);
  g1 = mesid_to_lfsr0_init(mesid, 0);
  EXPECT_EQ(0050172213, g1);

  mesid = construct_mesid(CODE_GPS_L1CA, 25);
  g1 = mesid_to_lfsr0_init(mesid, 0);
  EXPECT_EQ(0x3ff, g1);

  mesid = construct_mesid(CODE_GPS_L2CL, 1);
  g1 = mesid_to_lfsr0_init(mesid, 0);
  EXPECT_EQ(0624145772, g1);

  mesid = construct_mesid(CODE_GPS_L2CL, 2);
  g1 = mesid_to_lfsr0_init(mesid, 1);
  EXPECT_EQ(0243255237, g1);
}

TEST(test_prns_ca_code, code) {
  const u8* ptr;
  me_gnss_signal_t mesid;

  for (u8 k = 0; k < 32; k++) {
    mesid = construct_mesid(CODE_GPS_L1CA, k + GPS_FIRST_PRN);
    ptr = ca_code(mesid);
    EXPECT_TRUE(NULL != ptr);
  }

  for (u8 k = 0; k < 14; k++) {
    mesid = construct_mesid(CODE_GLO_L1OF, k + GLO_FIRST_PRN);
    ptr = ca_code(mesid);
    EXPECT_TRUE(NULL != ptr);
  }

  for (u8 k = 0; k < 14; k++) {
    mesid = construct_mesid(CODE_GLO_L2OF, k + GLO_FIRST_PRN);
    ptr = ca_code(mesid);
    EXPECT_TRUE(NULL != ptr);
  }

  for (u8 k = 0; k < 19; k++) {
    mesid = construct_mesid(CODE_SBAS_L1CA, k + SBAS_FIRST_PRN);
    ptr = ca_code(mesid);
    EXPECT_TRUE(NULL != ptr);
  }

  for (u8 k = 0; k < 37; k++) {
    mesid = construct_mesid(CODE_BDS2_B1, k + BDS_FIRST_PRN);
    ptr = ca_code(mesid);
    EXPECT_TRUE(NULL != ptr);
  }

  for (u8 k = 0; k < 50; k++) {
    mesid = construct_mesid(CODE_GAL_E1X, k + GAL_FIRST_PRN);
    ptr = ca_code(mesid);
    EXPECT_TRUE(NULL != ptr);
  }
}

TEST(test_prns_get_chip, get_chip) {
  u8 code = 0xFF;

  s8 chip;

  chip = get_chip(&code, 0);
  EXPECT_EQ(-1, chip);
}
