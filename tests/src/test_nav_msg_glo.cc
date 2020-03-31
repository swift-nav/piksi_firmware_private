#include <stdio.h>
#include <string.h>
#include <swiftnav/logging.h>

#include <cmath>

#include "gtest/gtest.h"
#include "nav_msg/nav_msg_glo.h"
#undef log_error
#define log_error(...)

#define LOW_TOL 1e-6
#define HIGH_TOL 1e-1

nav_msg_glo_t n;
/*This input strings were taken from collected data,
 * refer to libswiftnav/tests/data/gloframesstream/raw_glo_frame_ascii.log
 * #GLORAWFRAMEA,USB1,0,73.0,SATTIME,1892,300827.000,00000000,8792,13498;
 * 3,55,4,1892,300827.077,52,52,15,
 * 01074396999b05c3a850b5,0,021760a5256204d9c15f66,0,0380269d60899a6d0e3123,0,
 * 04865d1cc0000000344918,0,050d100000000340000895,0,06ab81d85b1019f107b83c,0,
 * 0705d3179ea697fc554079,0,082c00148c06153f8e133e,0,0972222bcd4e97ff14be12,0,
 * 0aad8090a54019cb035d3f,0,0b3e2240201e97fc34fc39,0,0cae940cdc3c1e2786da9b,0,
 * 0d68bf54a4c697f115320b,0,0eaf8449b38c1e228932d8,0,0f815b653eee981314802a,0*f3837a1c
 */
glo_string_t strings_in[6] = {
    {1, 1, 1}, /* dummy words used in test_nav_msg_update_glo only */
    {0xc3a850b5, 0x96999b05, 0x010743}, /* 01074396999b05c3a850b5 */
    {0xd9c15f66, 0xa5256204, 0x021760}, /* 021760a5256204d9c15f66 */
    {0x6d0e3123, 0x9d60899a, 0x038026}, /* 0380269d60899a6d0e3123 */
    {0x00344918, 0x1cc00000, 0x04865d}, /* 04865d1cc0000000344918 */
    {0x40000895, 0x3, 0x050d10}         /* 050d100000000340000895 */
};

/* RAW strings above correspond to following data from same file:
 * #GLOEPHEMERISA,USB1,0,73.5,SATTIME,1892,300617.000,00000000,8d29,13498;
 * 55,4,1,4,1892,301517000,10783,104,0,0,59,0,
 * -1.4453039062500000e+07,-6.9681713867187500e+06,1.9873773925781250e+07, <--
 * X, Y, Z
 * -1.4125013351440430e+03,-2.3216266632080078e+03,-1.8360681533813477e+03, <--
 * Vx, Vy, Vz
 * 0.00000000000000000,0.00000000000000000,-2.79396772384643555e-06, <-- Ax, Ay,
 * Az
 * -9.71024855971336365e-05, <-- tau
 * 5.587935448e-09,
 * 1.81898940354585648e-12, <-- gamma
 * 52200,3,0,0,13*955c64e9
 */
double X = -1.4453039062500000e+07;
double Y = -6.9681713867187500e+06;
double Z = 1.9873773925781250e+07;
double VX = -1.4125013351440430e+03;
double VY = -2.3216266632080078e+03;
double VZ = -1.8360681533813477e+03;
double AX = 0;
double AY = 0;
double AZ = -2.79396772384643555e-06;
double GAMMA = 1.81898940354585648e-12;
double TAU = -9.71024855971336365e-05;

void e_out(void) {
  log_debug("GLO Ephemeris:\n");
  log_debug("\tSID: %u (code %u)\n", n.eph.sid.sat, n.eph.sid.code);
  log_debug("\tGPS time: TOE %f, WN %d\n", n.eph.toe.tow, n.eph.toe.wn);
  log_debug("\tURA: %f\n", n.eph.ura);
  log_debug("\tFit interval: %u\n", n.eph.fit_interval);
  log_debug("\tValid: %u\n", n.eph.valid);
  log_debug("\tHealth bits: 0x%02x\n", n.eph.health_bits);
  log_debug("\tgamma: %25.18f\n", n.eph.glo.gamma);
  log_debug("\ttau: %25.18f\n", n.eph.glo.tau);
  log_debug("\tX, Y, Z: %25.18f, %25.18f, %25.18f\n",
            n.eph.glo.pos[0],
            n.eph.glo.pos[1],
            n.eph.glo.pos[2]);
  log_debug("\tVX, VY, VZ: %25.18f, %25.18f, %25.18f\n",
            n.eph.glo.vel[0],
            n.eph.glo.vel[1],
            n.eph.glo.vel[2]);
  log_debug("\tAX, AY, AZ: %25.18f, %25.18f, %25.18f\n",
            n.eph.glo.acc[0],
            n.eph.glo.acc[1],
            n.eph.glo.acc[2]);
  EXPECT_LT(std::abs(n.eph.glo.pos[0] - X), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.pos[1] - Y), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.pos[2] - Z), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.vel[0] - VX), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.vel[1] - VY), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.vel[2] - VZ), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.acc[0] - AX), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.acc[1] - AY), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.acc[2] - AZ), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.tau - TAU), LOW_TOL);
  EXPECT_LT(std::abs(n.eph.glo.gamma - GAMMA), LOW_TOL);
}

TEST(nav_msg_glo_tests, process_string_glo) {
  u32 time_tag_ms = 0;
  me_gnss_signal_t mesid;
  mesid.sat = 1;
  mesid.code = CODE_GLO_L1OF;
  memset(&n, 0, sizeof(n));
  nav_msg_init_glo(&n, mesid);
  for (u8 i = 1; i < sizeof(strings_in) / sizeof(strings_in[1]); i++) {
    n.string = strings_in[i];
    time_tag_ms += GLO_STR_LEN_S * SECS_MS;
    process_string_glo(&n, time_tag_ms);
  }
  e_out();
}

void msg_update_test(bool inverted) {
  u32 time_tag_ms = 0;
  me_gnss_signal_t mesid;
  mesid.sat = 1;
  mesid.code = CODE_GLO_L1OF;
  memset(&n, 0, sizeof(n));
  /* the unit test encodes strings_in to generate either normal or
   * inverted glo bitstream , calls nav_msg_update_glo to receive and finally
   * decodes received string */
  nav_msg_init_glo(&n, mesid);
  /* get string one by one */
  for (u8 i = 0; i < sizeof(strings_in) / sizeof(strings_in[0]); i++) {
    u8 manchester = 0;
    u8 meander = inverted ? 2 : 1;
    nav_msg_glo_t a;
    nav_msg_status_t ret;
    u8 j;
    nav_msg_init_glo(&a, mesid);
    u8 relcode_state = 0;
    /* write test string to temporary buffer */
    a.string = strings_in[i];
    /* transmit data bits, 85 bits */
    for (j = GLO_STR_LEN; j > 0; j--) {
      bool one_bit = extract_word_glo(&a, j, 1); /* get bit to be transmitted */
      relcode_state ^= one_bit;                  /* apply relative code phase */
      manchester = ((relcode_state << 1) | relcode_state) ^
                   meander; /* transform to line code */
      /* now pass it to receiver MSB first, receiver must return -1 */
      ret = nav_msg_update_glo(&n, (manchester >> 1) & 1);
      EXPECT_EQ(GLO_STRING_NOT_READY, ret);
      /* now LSB, receiver must return -1 */
      ret = nav_msg_update_glo(&n, manchester & 1);
    }
    /* try to decode the string */
    if (GLO_STRING_READY == ret) {
      EXPECT_EQ(memcmp(a.string, n.string, sizeof(a.string)), 0);

      time_tag_ms += GLO_STR_LEN_S * SECS_MS;
      if (process_string_glo(&n, time_tag_ms) == 1) {
        e_out();
      }
    }
    /* now pass time mark bit by bit to receiver (MSB first),
     * no line code needed */
    for (u8 k = 30; k > 0; k--) {
      if (inverted) {
        ret = nav_msg_update_glo(&n, (GLO_TM_INV >> (k - 1)) & 1);
      } else {
        ret = nav_msg_update_glo(&n, (GLO_TM >> (k - 1)) & 1);
      }
      EXPECT_TRUE((GLO_STRING_NOT_READY == ret) ||
                  (GLO_TIME_MARK_DECODED == ret));
    }
  }
}

TEST(nav_msg_glo_tests, nav_msg_update_glo) {
  msg_update_test(false);
  msg_update_test(true);
}

TEST(nav_msg_glo_tests, get_tow_glo) {
  u32 time_tag_ms = 0;
  me_gnss_signal_t mesid;
  mesid.sat = 1;
  mesid.code = CODE_GLO_L1OF;
  memset(&n, 0, sizeof(n));
  nav_msg_init_glo(&n, mesid);
  gps_time_t t = glo2gps(&n.toe, /* utc_params = */ NULL);
  EXPECT_EQ(t.tow, TOW_UNKNOWN);
  for (u8 i = 1; i < sizeof(strings_in) / sizeof(strings_in[1]); i++) {
    n.string = strings_in[i];
    time_tag_ms += GLO_STR_LEN_S * SECS_MS;
    process_string_glo(&n, time_tag_ms);
  }
  t = glo2gps(&n.toe, /* utc_params = */ NULL);
  EXPECT_EQ(t.tow, n.eph.toe.tow);
}
