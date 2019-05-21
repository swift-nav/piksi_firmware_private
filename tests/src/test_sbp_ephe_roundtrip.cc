#include "gtest/gtest.h"

#include "sbp/sbp_utils.h"

// real ephemeris for SV 15 at 8:00, 30-May-2016
// tow and toc modified to fit the dates below
static const ephemeris_t e_in = {
    {15,                            // sid.sat
     CODE_GPS_L1CA},                // sid.code
    {205200.0,                      // toe.tow
     1899},                         // toe.wn
    1.0,                            // ura
    14400,                          // fit_interval
    1,                              // valid
    0,                              // health_bits
    0,                              // source
    {{{{-0.00000001071020960808}},  // kepler.tgd_gps_s
      222.34375000000000000000,     // kepler.crc
      62.87500000000000000000,      // kepler.crs
      0.00000356882810592651,       // kepler.cuc
      0.00000734068453311920,       // kepler.cus
      0.00000002793967723846,       // kepler.cic
      0.00000012852251529694,       // kepler.cis
      0.00000000562166273625,       // kepler.dn
      1.32453305390110598339,       // kepler.m0
      0.00812081422191113234,       // kepler.ecc
      5153.59741973876953125000,    // kepler.sqrta
      -0.25515026323261330576,      // kepler.omega0
      -0.00000000872250618454,      // kepler.omegadot
      0.48344690815566465636,       // kepler.w
      0.93105119838528094256,       // kepler.inc
      0.00000000008536069847,       // kepler.inc_dot
      -0.00031310319900512695,      // kepler.af0
      -0.00000000000170530257,      // kepler.af1
      0.00000000000000000000,       // kepler.af2
      {205200.0,                    // kepler.toc.tow
       1899},                       // kepler.toc.wn
      40,                           // kepler.iodc
      40}}                          // kepler.iode
};

TEST(test_ephemeris_sbp, sbp_roundtrip) {
  gps_time_t t_start = e_in.toe;
  t_start.tow += -(double)e_in.fit_interval / 2.0;
  normalize_gps_time(&t_start);

  gps_time_t t = t_start;

  // check the positions every second during the ephemeris fix interval
  while (gpsdifftime(&t, &e_in.toe) < (double)e_in.fit_interval / 2) {
    t.tow += 1.0;
    normalize_gps_time(&t);
    double _[3];
    double sat_pos[3];
    double sat_vel[3];
    double sbp_sat_pos[3];
    double sbp_sat_vel[3];
    u8 iode;
    u16 iodc;

    ephemeris_t e_sbp;

    // convert the ephemeris into sbp and back
    msg_ephemeris_t sbp_msg;
    pack_ephemeris(&e_in, &sbp_msg);
    unpack_ephemeris(&sbp_msg, &e_sbp);

    calc_sat_state(&e_in, &t, sat_pos, sat_vel, _, _, _, &iodc, &iode);
    calc_sat_state(&e_sbp, &t, sbp_sat_pos, sbp_sat_vel, _, _, _, &iodc, &iode);

    // Compare the original ephemeris to the one converted to SBP and back
    const double POS_THRESHOLD = 1e-8;   // m
    const double VEL_THRESHOLD = 1e-12;  // m/s

    EXPECT_LT(fabs(sat_pos[0] - sbp_sat_pos[0]), POS_THRESHOLD);
    EXPECT_LT(fabs(sat_pos[1] - sbp_sat_pos[1]), POS_THRESHOLD);
    EXPECT_LT(fabs(sat_pos[2] - sbp_sat_pos[2]), POS_THRESHOLD);
    EXPECT_LT(fabs(sat_vel[0] - sbp_sat_vel[0]), VEL_THRESHOLD);
    EXPECT_LT(fabs(sat_vel[1] - sbp_sat_vel[1]), VEL_THRESHOLD);
    EXPECT_LT(fabs(sat_vel[2] - sbp_sat_vel[2]), VEL_THRESHOLD);
  }
}
