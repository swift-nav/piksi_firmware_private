#include "gtest/gtest.h"

#include <math.h>
#include <stdio.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>

#include "pvt.h"

static navigation_measurement_t nm1 = {
    .raw_pseudorange = 23946993.888943646,
    .pseudorange = 23946993.888943646,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-19477278.087422125, -7649508.9457812719, 16674633.163554827},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {9, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm1_no_doppler = {
    .raw_pseudorange = 23946993.888943646,
    .pseudorange = 23946993.888943646,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-19477278.087422125, -7649508.9457812719, 16674633.163554827},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {9, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID};

static navigation_measurement_t nm2 = {
    .raw_pseudorange = 22932174.156858064,
    .pseudorange = 22932174.156858064,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-9680013.5408340245, -15286326.354385279, 19429449.383770257},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {1, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm3 = {
    .raw_pseudorange = 24373231.648055989,
    .pseudorange = 24373231.648055989,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-19858593.085281931, -3109845.8288993631, 17180320.439503901},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {2, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm4 = {
    .raw_pseudorange = 24779663.252316438,
    .pseudorange = 24779663.252316438,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {6682497.8716542246, -14006962.389166718, 21410456.275678463},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {3, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm5 = {
    .raw_pseudorange = 26948717.022331879,
    .pseudorange = 26948717.022331879,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {7415370.9916331079, -24974079.044485383, -3836019.0262199985},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {4, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm6 = {
    .raw_pseudorange = 23327405.435463827,
    .pseudorange = 23327405.435463827,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-2833466.1648670658, -22755197.793894723, 13160322.082875408},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {5, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm7 = {
    .raw_pseudorange = 27371419.016328193,
    .pseudorange = 27371419.016328193,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {14881660.383624561, -5825253.4316490609, 21204679.68313824},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {6, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm8 = {
    .raw_pseudorange = 26294221.697782904,
    .pseudorange = 26294221.697782904,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {12246530.477279386, -22184711.955107089, 7739084.2855069181},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {7, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm9 = {
    .raw_pseudorange = 25781999.479948733,
    .pseudorange = 25781999.479948733,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-25360766.249484103, -1659033.490658124, 7821492.0398916304},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {8, CODE_GPS_L1CA},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

static navigation_measurement_t nm10 = {
    .raw_pseudorange = 25781999.479948733,
    .pseudorange = 25781999.479948733,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-25360766.249484103, -1659033.490658124, 7821492.0398916304},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {8, CODE_GPS_L2CM},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

/* broken measurement */
static navigation_measurement_t nm10b = {
    .raw_pseudorange = 25781999.479948733,
    .pseudorange = 25781999.479948733 + 30000,
    .raw_carrier_phase = 0,
    .carrier_phase = 0,
    .raw_measured_doppler = 0,
    .measured_doppler = 0,
    .raw_computed_doppler = 0,
    .computed_doppler = 0,
    .computed_doppler_dt = 0,
    .sat_pos = {-25360766.249484103, -1659033.490658124, 7821492.0398916304},
    .sat_vel = {0, 0, 0},
    .IODE = 0,
    .sat_clock_err = 0,
    .sat_clock_err_rate = 0,
    .IODC = 0,
    .cn0 = 0,
    .lock_time = 5,
    .time_in_track = 0,
    .elevation = 0,
    .tot = {1000000, 1939},
    .sid = {8, CODE_GPS_L2CM},
    .flags = NAV_MEAS_FLAG_CODE_VALID | NAV_MEAS_FLAG_MEAS_DOPPLER_VALID |
             NAV_MEAS_FLAG_PHASE_VALID};

TEST(pvt_tests, test_pvt_failed_repair) {
  u8 n_used = 5;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;

  navigation_measurement_t nms[9] = {nm1, nm2, nm3, nm4, nm5, nm6, nm7, nm8};

  calc_PVT(n_used, nms, false, true, &soln, &dops, &raim_removed_sids);
  /* PVT repair requires at least 6 measurements. */
  EXPECT_EQ(soln.valid, 0) << "Solution should be invalid!";
}

TEST(pvt_tests, test_pvt_repair) {
  u8 n_used = 6;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;
  gnss_signal_t expected_removed_sid = {.sat = 9, .code = CODE_GPS_L1CA};

  navigation_measurement_t nms[9] = {
      nm1, nm2, nm3, nm4, nm5, nm6, nm7, nm8, nm9};

  s8 code =
      calc_PVT(n_used, nms, false, true, &soln, &dops, &raim_removed_sids);
  EXPECT_EQ(code, 1) << "Return code should be 1 (pvt repair)";
  EXPECT_EQ(soln.n_sigs_used, n_used - 1);  //  "n_sigs_used should be %u. Saw:
                                            //  %u\n", n_used - 1,
                                            //  soln.n_sigs_used);
  EXPECT_EQ(soln.n_sats_used, n_used - 1);  //   "n_sats_used should be %u. Saw:
                                            //   %u\n", n_used - 1,
                                            //   soln.n_sats_used);
  EXPECT_TRUE(sid_set_contains(
      &raim_removed_sids,
      expected_removed_sid));  //   "Unexpected RAIM removed SID!\n");
}

TEST(pvt_tests, test_pvt_raim_singular) {
  /* test the case of bug 946 where extreme pseudorange errors lead to singular
   * geometry */
  u8 n_used = 9;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;

  navigation_measurement_t nm1_broken = nm1;
  navigation_measurement_t nm2_broken = nm2;
  nm1_broken.pseudorange += 5e8;
  nm2_broken.pseudorange -= 2e7;

  navigation_measurement_t nms[9] = {
      nm1_broken, nm2_broken, nm3, nm4, nm5, nm6, nm7, nm9, nm10};

  s8 code =
      calc_PVT(n_used, nms, false, true, &soln, &dops, &raim_removed_sids);

  EXPECT_EQ(code,
            -4);  // "Return code should be -4 (RAIM failed). Saw: %d\n", code);
}

TEST(pvt_tests, test_pvt_vel_repair) {
  u8 n_used = 6;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;
  gnss_signal_t expected_removed_sid = {5, CODE_GPS_L1CA};

  /* add a Doppler outlier */
  navigation_measurement_t nm6_broken = nm6;
  nm6_broken.cn0 = 40;
  nm6_broken.measured_doppler += 10000;

  navigation_measurement_t nms[6] = {nm2, nm3, nm4, nm5, nm6_broken, nm7};

  s8 code =
      calc_PVT(n_used, nms, false, false, &soln, &dops, &raim_removed_sids);
  EXPECT_EQ(code,
            1);  // "Return code should be 1 (pvt repair). Saw: %d\n", code);
  EXPECT_EQ(soln.n_sigs_used, n_used - 1);  //"n_sigs_used should be %u. Saw:
                                            //%u\n", n_used - 1,
                                            // soln.n_sigs_used);
  EXPECT_EQ(soln.n_sats_used, n_used - 1);  //"n_sats_used should be %u. Saw:
                                            //%u\n", n_used - 1,
                                            // soln.n_sats_used);
  EXPECT_TRUE(sid_set_contains(
      &raim_removed_sids,
      expected_removed_sid));  // "Unexpected RAIM removed SID!\n");
}

TEST(pvt_tests, test_pvt_repair_multifailure) {
  u8 n_used = 7;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;
  gnss_signal_t expected_removed_sid = {.sat = 9, .code = CODE_GPS_L1CA};

  navigation_measurement_t nms[8] = {nm1, nm2, nm3, nm7, nm10b, nm5, nm6, nm7};

  s8 code =
      calc_PVT(n_used, nms, false, false, &soln, &dops, &raim_removed_sids);
  EXPECT_EQ(code, 1) << "Return code should be 1 (pvt repair)";
  EXPECT_EQ(soln.n_sigs_used, n_used - 2);  //    "n_sigs_used should be %u.
                                            //    Saw: %u\n", n_used - 2,
                                            //    soln.n_sigs_used);
  EXPECT_EQ(soln.n_sats_used, n_used - 2);  //    "n_sats_used should be %u.
                                            //    Saw: %u\n", n_used - 2,
                                            //    soln.n_sats_used);
  EXPECT_TRUE(sid_set_contains(
      &raim_removed_sids,
      expected_removed_sid));  //    "Unexpected RAIM removed SID!\n");
}

TEST(pvt_tests, test_disable_pvt_raim) {
  u8 n_used = 6;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;

  navigation_measurement_t nms[9] = {
      nm1, nm2, nm3, nm4, nm5, nm6, nm7, nm8, nm9};

  /* disable raim check */
  s8 code = calc_PVT(n_used, nms, true, true, &soln, &dops, &raim_removed_sids);
  EXPECT_EQ(code, 2) << "Return code should be 2 (raim not used)";
  EXPECT_EQ(soln.valid, 1) << "Solution should be valid!";
}

TEST(pvt_tests, test_disable_pvt_velocity) {
  u8 n_used = 6;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;

  navigation_measurement_t nms[9] = {
      nm1_no_doppler, nm2, nm3, nm4, nm5, nm6, nm7, nm8, nm9};

  s8 code =
      calc_PVT(n_used, nms, false, true, &soln, &dops, &raim_removed_sids);
  EXPECT_GE(code, 0) << "Return code should be >=0 (success)";
  EXPECT_EQ(soln.valid, 1) << "Solution should be valid!";
  EXPECT_TRUE((soln.vel_ned[0] == 0.0) && (soln.vel_ned[1] == 0.0) &&
              (soln.vel_ned[2] == 0.0))
      << "Velocity NED was not zero.";
  EXPECT_TRUE((soln.vel_ecef[0] == 0.0) && (soln.vel_ecef[1] == 0.0) &&
              (soln.vel_ecef[2] == 0.0))
      << "Velocity ECEF was not zero.";
}

TEST(pvt_tests, test_count_sats) {
  u8 n_used = 10;
  gnss_solution soln;
  dops_t dops;
  gnss_sid_set_t raim_removed_sids;

  navigation_measurement_t nms[10] = {
      nm1, nm2, nm3, nm4, nm5, nm6, nm7, nm8, nm9, nm10};

  /* disable raim check */
  s8 code =
      calc_PVT(n_used, nms, true, false, &soln, &dops, &raim_removed_sids);
  EXPECT_GE(code, 0) << "Return code should be >=0 (success)";
  EXPECT_EQ(soln.valid, 1) << "Solution should be valid!";
  EXPECT_EQ(soln.n_sigs_used, 10) << "n_sigs_used should be 10";
  EXPECT_EQ(soln.n_sats_used, 9) << "n_sats_used should be 9";
}

TEST(pvt_tests, test_dops) {
  u8 n_used = 6;
  gnss_solution soln;
  dops_t dops = {.pdop = 22, .gdop = 22, .tdop = 22, .hdop = 22, .vdop = 22};
  dops_t truedops = {.pdop = 2.69955,
                     .gdop = 3.07696,
                     .tdop = 1.47652,
                     .hdop = 1.76157,
                     .vdop = 2.04559};
  gnss_sid_set_t raim_removed_sids;

  const double dop_tol = 1e-3;

  navigation_measurement_t nms[6] = {nm1, nm2, nm3, nm4, nm5, nm6};

  /* disable raim check */
  s8 code =
      calc_PVT(n_used, nms, false, true, &soln, &dops, &raim_removed_sids);
  EXPECT_GE(code, 0) << "Return code should be >=0 (success)";
  EXPECT_EQ(soln.valid, 1) << "Solution should be valid!";
  EXPECT_TRUE(fabs(dops.pdop * dops.pdop -
                   (dops.vdop * dops.vdop + dops.hdop * dops.hdop)) < dop_tol)
      << "HDOP^2 + VDOP^2 != PDOP^2.";
  double dop_err =
      fabs(dops.pdop - truedops.pdop) + fabs(dops.gdop - truedops.gdop) +
      fabs(dops.tdop - truedops.tdop) + fabs(dops.hdop - truedops.hdop) +
      fabs(dops.vdop - truedops.vdop);
  EXPECT_LT(dop_err, dop_tol) << "DOPs don't match hardcoded correct values.";
}

TEST(pvt_tests, iono_tropo_usage_test) {
  /*NOTE: this unit test checks calc_iono_tropo function usage only.
   * Find iono and tropo correction unit tests in check_ionosphere.c and
   * check_troposphere.c accordingly*/
  u8 n_ready_tdcp = 1;
  navigation_measurement_t nav_meas_tdcp = {
      .raw_pseudorange = 0,
      .pseudorange = 22932174.156858064,
      .raw_carrier_phase = 0,
      .carrier_phase = 0,
      .raw_measured_doppler = 0,
      .measured_doppler = 1000,
      .raw_computed_doppler = 0,
      .computed_doppler = 1000,
      .computed_doppler_dt = 0,
      .sat_pos = {-9680013.5408340245, -15286326.354385279, 19429449.383770257},
      .sat_vel = {-1000, 3000, 2000},
      .IODE = 0,
      .sat_clock_err = 0,
      .sat_clock_err_rate = 0,
      .IODC = 0,
      .cn0 = 0,
      .lock_time = 0,
      .time_in_track = 0,
      .elevation = 0,
      .tot = {479820, 1875},
      .sid = {1, CODE_GPS_L1CA},
      .flags = 0};
  double pos_ecef[3];
  double pos_llh[3] = {20.3 * D2R, 149.1 * D2R, 0};

  wgsllh2ecef(pos_llh, pos_ecef);

  double pr_tropo_corrected = 22932123.831141;
  double pr_iono_tropo_corrected = 22932118.957291;
  double doppler_tropo_corrected = 999.521223;
  double doppler_iono_tropo_corrected = 999.517450;

  ionosphere_t i = {.toa = GPS_TIME_UNKNOWN,
                    .a0 = 0.1583e-7,
                    .a1 = -0.7451e-8,
                    .a2 = -0.5960e-7,
                    .a3 = 0.1192e-6,
                    .b0 = 0.1290e6,
                    .b1 = -0.2130e6,
                    .b2 = 0.6554e5,
                    .b3 = 0.3277e6};

  calc_iono_tropo(n_ready_tdcp, &nav_meas_tdcp, pos_ecef, pos_llh, &i);

  EXPECT_LT(fabs(nav_meas_tdcp.pseudorange - pr_iono_tropo_corrected), 1e-3)
      << "Pseudorange Iono and Tropo corrected " << nav_meas_tdcp.pseudorange
      << " differs from expected " << pr_iono_tropo_corrected;

  EXPECT_LT(fabs(nav_meas_tdcp.measured_doppler - doppler_iono_tropo_corrected),
            1e-6)
      << "Measured Doppler Iono and Tropo corrected "
      << nav_meas_tdcp.measured_doppler << " differs from expected "
      << doppler_iono_tropo_corrected;

  EXPECT_LT(fabs(nav_meas_tdcp.computed_doppler - doppler_iono_tropo_corrected),
            1e-6)
      << "Computed Doppler Iono and Tropo corrected "
      << nav_meas_tdcp.computed_doppler << " differs from expected "
      << doppler_iono_tropo_corrected;

  nav_meas_tdcp.pseudorange = 22932174.156858064;
  nav_meas_tdcp.measured_doppler = 1000;
  nav_meas_tdcp.computed_doppler = 1000;

  calc_iono_tropo(
      n_ready_tdcp, &nav_meas_tdcp, pos_ecef, pos_llh, (ionosphere_t*)NULL);

  EXPECT_LT(fabs(nav_meas_tdcp.pseudorange - pr_tropo_corrected), 1e-3)
      << "Pseudorange Tropo corrected " << nav_meas_tdcp.pseudorange
      << " differs from expected " << pr_tropo_corrected;

  EXPECT_LT(fabs(nav_meas_tdcp.measured_doppler - doppler_tropo_corrected),
            1e-6)
      << "Measured Doppler Tropo corrected " << nav_meas_tdcp.measured_doppler
      << " differs from expected " << doppler_tropo_corrected;

  EXPECT_LT(fabs(nav_meas_tdcp.computed_doppler - doppler_tropo_corrected),
            1e-6)
      << "Computed Doppler Tropo corrected " << nav_meas_tdcp.computed_doppler
      << " differs from expected " << doppler_tropo_corrected;
}
