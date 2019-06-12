#include "gtest/gtest.h"
#include "nav_msg/cnav_msg_storage.h"
#include "shm/shm.h"

TEST(shm, test_shm_signal_healthy) {
  std::vector<gnss_signal_t> sids;
  sids.push_back(construct_sid(CODE_GPS_L1CA, 17));
  sids.push_back(construct_sid(CODE_GPS_L1P, 17));
  sids.push_back(construct_sid(CODE_GPS_L2CM, 17));
  sids.push_back(construct_sid(CODE_GPS_L2P, 17));

  // All SHIs empty
  for (auto &sid : sids) {
    EXPECT_FALSE(shm_signal_healthy(sid));
    EXPECT_FALSE(shm_signal_unhealthy(sid));
  }

  // LNAV summary unhealthy
  shm_gps_set_shi_ephemeris(17, 0x20);
  for (auto &sid : sids) {
    if (CODE_GPS_L1CA == sid.code || CODE_GPS_L1P == sid.code) {
      EXPECT_FALSE(shm_signal_healthy(sid));
      EXPECT_TRUE(shm_signal_unhealthy(sid));
    } else {
      EXPECT_FALSE(shm_signal_healthy(sid));
      EXPECT_FALSE(shm_signal_unhealthy(sid));
    }
  }
  // Clear summary bit
  shm_gps_set_shi_ephemeris(17, 0x00);

  // CNAV health flags unhealthy
  cnav_msg_type_10_t msg10 = {
      .l1_health = false, .l2_health = false, .l5_health = true};

  cnav_msg_t msg = {.prn = 17,
                    .msg_id = CNAV_MSG_TYPE_10,
                    .tow = 0,
                    .alert = false,
                    .bit_polarity = 0,
                    .data = {.type_10 = msg10}};

  cnav_msg_put(&msg);

  for (auto &sid : sids) {
    EXPECT_FALSE(shm_signal_healthy(sid));
    EXPECT_TRUE(shm_signal_unhealthy(sid));
  }

  // Navigation OK
  msg10 = {.l1_health = true, .l2_health = true, .l5_health = true};
  msg.data.type_10 = msg10;
  cnav_msg_put(&msg);

  shm_gps_set_shi_lnav_how_alert(17, true);
  shm_gps_set_shi_cnav_alert(17, true);

  for (auto &sid : sids) {
    EXPECT_TRUE(shm_signal_healthy(sid));
    EXPECT_FALSE(shm_signal_unhealthy(sid));
  }
}
