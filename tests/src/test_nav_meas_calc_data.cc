/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>
#include <stdio.h>

#include <libswiftnav/ch_meas.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/coord_system.h>
#include <libswiftnav/correct_iono_tropo.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/nav_meas.h>
#include <libswiftnav/observation.h>

#include "calc_nav_meas.h"

#include "gtest/gtest.h"

/* real ephemeris for SV 15 at 8:00, 30-May-2016 */
/* tow and toc modified to fit the dates below */
static const ephemeris_t e_in = {
    {15,                         /* sid.sat */
     CODE_GPS_L1CA},             /* sid.code */
    {205200.0,                   /* toe.tow */
     1899},                      /* toe.wn */
    1.0,                         /* ura */
    14400,                       /* fit_interval */
    1,                           /* valid */
    0,                           /* health_bits */
    {{{-0.00000001071020960808}, /* kepler.tgd_gps_s */
      222.34375000000000000000,  /* kepler.crc */
      62.87500000000000000000,   /* kepler.crs */
      0.00000356882810592651,    /* kepler.cuc */
      0.00000734068453311920,    /* kepler.cus */
      0.00000002793967723846,    /* kepler.cic */
      0.00000012852251529694,    /* kepler.cis */
      0.00000000562166273625,    /* kepler.dn */
      1.32453305390110598339,    /* kepler.m0 */
      0.00812081422191113234,    /* kepler.ecc */
      5153.59741973876953125000, /* kepler.sqrta */
      -0.25515026323261330576,   /* kepler.omega0 */
      -0.00000000872250618454,   /* kepler.omegadot */
      0.48344690815566465636,    /* kepler.w */
      0.93105119838528094256,    /* kepler.inc */
      0.00000000008536069847,    /* kepler.inc_dot */
      -0.00031310319900512695,   /* kepler.af0 */
      -0.00000000000170530257,   /* kepler.af1 */
      0.00000000000000000000,    /* kepler.af2 */
      {205200.0,                 /* kepler.toc.tow */
       1899},                    /* kepler.toc.wn */
      40,                        /* kepler.iodc */
      40}}                       /* kepler.iode */
};

/* Real measurements from tracking (Piksi v3 board) channel for L1C/A, SV 31. */
static const channel_measurement_t l1ca_meas_in = {
    {31,                         /* sid.sat */
     CODE_GPS_L1CA},             /* sid.code */
    1022.8769119973294437,       /* code_phase_chips */
    1023002.7708759307861328,    /* code_phase_rate */
    -111811276.6140588223934174, /* carrier_phase */
    4137.60058593750000005,      /* carrier_freq */
    206957325,                   /* time_of_week_ms */
    0,                           /* tow_residual_ns */
    -0.0007533685534591,         /* rec_time_delta */
    38.8603668212890625,         /* cn0 */
    0.0,                         /* lock_time */
    0.0,                         /* time_in_track */
    0.0,                         /* elevation */
    0                            /* flags */
};

/* Real measurements from tracking (Piksi v3 board) channel for L2CM, SV 12. */
static const channel_measurement_t l2cm_meas_in = {
    {12,                        /* .sid.sat */
     CODE_GPS_L2CM},            /* .sid.code */
    20459.9424161217175424,     /* .code_phase_chips */
    1023002.6959905624389648,   /* .code_phase_rate */
    -69199213.4344176054000854, /* .carrier_phase */
    3229.3356933593750000,      /* .carrier_freq */
    206957320,                  /* .time_of_week_ms */
    0,                          /* .tow_residual_ns */
    -0.0057533283018868,        /* .rec_time_delta */
    37.5970230102539062,        /* .cn0 */
    0.0,                        /* .lock_time */
    0.0,                        /* time_in_track */
    0.0,                        /* elevation */
    0};

TEST(test_nav_meas_calc_data, first_test) {
  navigation_measurement_t out_l1ca, out_l2cm;
  navigation_measurement_t *p_out_l1ca = &out_l1ca;
  navigation_measurement_t *p_out_l2cm = &out_l2cm;
  const ephemeris_t *e[1] = {&e_in}; /* Use one ephemeris
                                      for both calculations */

  gps_time_t rec_time = {
      206957.3995198214543052, /* .tow */
      1899                     /* .wn */
  };

  calc_nav_meas(1, &l1ca_meas_in, &p_out_l1ca, &rec_time);
  calc_sat_clock_corrections(1, &p_out_l1ca, e);
  log_debug(" ***** L1CA: *****\n");
  log_debug("raw_pseudorange = %30.20f\n", out_l1ca.raw_pseudorange);
  log_debug("pseudorange = %30.20f\n", out_l1ca.pseudorange);
  log_debug("carrier_phase = %30.20f\n", out_l1ca.carrier_phase);
  log_debug("raw_measured_doppler = %30.20f\n", out_l1ca.raw_measured_doppler);
  log_debug("measured_doppler = %30.20f\n", out_l1ca.measured_doppler);
  log_debug("sat_pos = %30.20f, %30.20f, %30.20f\n",
            out_l1ca.sat_pos[0],
            out_l1ca.sat_pos[1],
            out_l1ca.sat_pos[2]);
  log_debug("sat_vel = %30.20f, %30.20f, %30.20f\n",
            out_l1ca.sat_vel[0],
            out_l1ca.sat_vel[1],
            out_l1ca.sat_vel[2]);
  log_debug("cn0 = %30.20f\n", out_l1ca.cn0);
  log_debug("lock_time = %30.20f\n", out_l1ca.lock_time);
  log_debug("tow = %30.20f, wn = %d\n", out_l1ca.tot.tow, out_l1ca.tot.wn);
  log_debug("sat = %u, code = %u\n",
            (unsigned int)out_l1ca.sid.sat,
            (unsigned int)out_l1ca.sid.code);
  log_debug("TOR = %30.20f\n", out_l1ca.tot.tow + out_l1ca.pseudorange / GPS_C);

  calc_nav_meas(1, &l2cm_meas_in, &p_out_l2cm, &rec_time);
  calc_sat_clock_corrections(1, &p_out_l2cm, e);
  log_debug(" \n***** L2CM: *****\n");
  log_debug("raw_pseudorange = %30.20f\n", out_l2cm.raw_pseudorange);
  log_debug("pseudorange = %30.20f\n", out_l2cm.pseudorange);
  log_debug("carrier_phase = %30.20f\n", out_l2cm.carrier_phase);
  log_debug("raw_measured_doppler = %30.20f\n", out_l2cm.raw_measured_doppler);
  log_debug("measured_doppler = %30.20f\n", out_l2cm.measured_doppler);
  log_debug("sat_pos = %30.20f, %30.20f, %30.20f\n",
            out_l2cm.sat_pos[0],
            out_l2cm.sat_pos[1],
            out_l2cm.sat_pos[2]);
  log_debug("sat_vel = %30.20f, %30.20f, %30.20f\n",
            out_l2cm.sat_vel[0],
            out_l2cm.sat_vel[1],
            out_l2cm.sat_vel[2]);
  log_debug("cn0 = %30.20f\n", out_l2cm.cn0);
  log_debug("lock_time = %30.20f\n", out_l2cm.lock_time);
  log_debug("tow = %30.20f, wn = %d\n", out_l2cm.tot.tow, out_l2cm.tot.wn);
  log_debug("sat = %u, code = %u\n",
            (unsigned int)out_l2cm.sid.sat,
            (unsigned int)out_l2cm.sid.code);
  log_debug("TOR = %30.20f\n", out_l2cm.tot.tow + out_l2cm.pseudorange / GPS_C);

  double check_value =
      fabs(sqrt(pow(out_l1ca.sat_pos[0], 2) + pow(out_l1ca.sat_pos[1], 2) +
                pow(out_l1ca.sat_pos[2], 2)) -
           sqrt(pow(out_l2cm.sat_pos[0], 2) + pow(out_l2cm.sat_pos[1], 2) +
                pow(out_l2cm.sat_pos[2], 2)));
  EXPECT_LT(check_value, 0.02);

  /* L1 and L2 pseudoranges differ some meters because of ionosphere and
   * inter-signal delay */
  check_value = fabs(out_l1ca.pseudorange - out_l2cm.pseudorange);
  EXPECT_LT(check_value, 15);

  check_value = fabs(out_l1ca.measured_doppler / out_l2cm.measured_doppler -
                     GPS_L1_HZ / GPS_L2_HZ);
  EXPECT_LT(check_value, 0.003);

  check_value =
      fabs(out_l1ca.tot.tow + out_l1ca.pseudorange / GPS_C - rec_time.tow);
  EXPECT_LT(check_value, 5e-8);

  check_value =
      fabs(out_l2cm.tot.tow + out_l2cm.pseudorange / GPS_C - rec_time.tow);
  EXPECT_LT(check_value, 5e-8);
}

static navigation_measurement_t nm1 = {
    11,              /* raw_pseudorange */
    0.0,             /* pseudorange */
    12,              /* raw_carrier_phase */
    0.0,             /* carrier_phase */
    13,              /* raw_measured_doppler */
    0.0,             /* measured_doppler */
    14,              /* raw_computed_doppler */
    0.0,             /* computed_doppler */
    0.0,             /* computed_doppler_dt */
    {1,              /* sat_pos[0] */
     2,              /* sat_pos[1] */
     3},             /* sat_pos[2] */
    {4,              /* sat_vel[0] */
     5,              /* sat_vel[1] */
     6},             /* sat_vel[2] */
    {7,              /* sat_acc[0] */
     8,              /* sat_acc[1] */
     9},             /* sat_acc[2] */
    0,               /* IODE */
    0.0,             /* sat_clock_err */
    0.0,             /* sat_clock_err_rate */
    0,               /* IODC */
    15,              /* cn0 */
    16,              /* lock_time */
    0.0,             /* elevation */
    {0.0,            /* tot.tow */
     0},             /* tot.wn */
    {1,              /* sid.sat */
     CODE_GPS_L1CA}, /* sid.code */
    0xAAAA           /* nav_meas_flags_t */
};
static navigation_measurement_t nm1_2 = {
    111,             /* raw_pseudorange */
    0.0,             /* pseudorange */
    112,             /* raw_carrier_phase */
    0.0,             /* carrier_phase */
    113,             /* raw_measured_doppler */
    0.0,             /* measured_doppler */
    114,             /* raw_computed_doppler */
    0.0,             /* computed_doppler */
    0.0,             /* computed_doppler_dt */
    {7,              /* sat_pos[0] */
     8,              /* sat_pos[1] */
     9},             /* sat_pos[2] */
    {10,             /* sat_vel[0] */
     11,             /* sat_vel[1] */
     12},            /* sat_vel[2] */
    {13,             /* sat_acc[0] */
     14,             /* sat_acc[1] */
     15},            /* sat_acc[2] */
    0,               /* IODE */
    0.0,             /* sat_clock_err */
    0.0,             /* sat_clock_err_rate */
    0,               /* IODC */
    115,             /* cn0 */
    116,             /* lock_time */
    0.0,             /* elevation */
    {0.0,            /* tot.tow */
     0},             /* tot.wn */
    {1,              /* sid.sat */
     CODE_GPS_L1CA}, /* sid.code */
    0xAAAA           /* nav_meas_flags_t */
};
static navigation_measurement_t nm2 = {
    21,              /* raw_pseudorange */
    0.0,             /* pseudorange */
    22,              /* raw_carrier_phase */
    0.0,             /* carrier_phase */
    23,              /* raw_measured_doppler */
    0.0,             /* measured_doppler */
    24,              /* raw_computed_doppler */
    0.0,             /* computed_doppler */
    0.0,             /* computed_doppler_dt */
    {1,              /* sat_pos[0] */
     2,              /* sat_pos[1] */
     3},             /* sat_pos[2] */
    {4,              /* sat_vel[0] */
     5,              /* sat_vel[1] */
     6},             /* sat_vel[2] */
    {7,              /* sat_acc[0] */
     8,              /* sat_acc[1] */
     9},             /* sat_acc[2] */
    0,               /* IODE */
    0.0,             /* sat_clock_err */
    0.0,             /* sat_clock_err_rate */
    0,               /* IODC */
    25,              /* cn0 */
    26,              /* lock_time */
    0.0,             /* elevation */
    {0.0,            /* tot.tow */
     0},             /* tot.wn */
    {2,              /* sid.sat */
     CODE_GPS_L1CA}, /* sid.code */
    0xAAAA           /* nav_meas_flags_t */
};
static navigation_measurement_t nm2_2 = {
    221,             /* raw_pseudorange */
    0.0,             /* pseudorange */
    222,             /* raw_carrier_phase */
    0.0,             /* carrier_phase */
    223,             /* raw_measured_doppler */
    0.0,             /* measured_doppler */
    224,             /* raw_computed_doppler */
    0.0,             /* computed_doppler */
    0.0,             /* computed_doppler_dt */
    {13,             /* sat_pos[0] */
     14,             /* sat_pos[1] */
     15},            /* sat_pos[2] */
    {16,             /* sat_vel[0] */
     17,             /* sat_vel[1] */
     18},            /* sat_vel[2] */
    {19,             /* sat_acc[0] */
     20,             /* sat_acc[1] */
     21},            /* sat_acc[2] */
    0,               /* IODE */
    0.0,             /* sat_clock_err */
    0.0,             /* sat_clock_err_rate */
    0,               /* IODC */
    225,             /* cn0 */
    226,             /* lock_time */
    0.0,             /* elevation */
    {0.0,            /* tot.tow */
     0},             /* tot.wn */
    {2,              /* sid.sat */
     CODE_GPS_L1CA}, /* sid.code */
    0x5555           /* nav_meas_flags_t */
};
static navigation_measurement_t nm3 = {
    31,              /* raw_pseudorange */
    0.0,             /* pseudorange */
    32,              /* raw_carrier_phase */
    0.0,             /* carrier_phase */
    33,              /* raw_measured_doppler */
    0.0,             /* measured_doppler */
    34,              /* raw_computed_doppler */
    0.0,             /* computed_doppler */
    0.0,             /* computed_doppler_dt */
    {1,              /* sat_pos[0] */
     2,              /* sat_pos[1] */
     3},             /* sat_pos[2] */
    {4,              /* sat_vel[0] */
     5,              /* sat_vel[1] */
     6},             /* sat_vel[2] */
    {7,              /* sat_acc[0] */
     8,              /* sat_acc[1] */
     9},             /* sat_acc[2] */
    0,               /* IODE */
    0.0,             /* sat_clock_err */
    0.0,             /* sat_clock_err_rate */
    0,               /* IODC */
    35,              /* cn0 */
    36,              /* lock_time */
    0.0,             /* elevation */
    {0.0,            /* tot.tow */
     0},             /* tot.wn */
    {3,              /* sid.sat */
     CODE_GPS_L1CA}, /* sid.code */
    0xAAAA           /* nav_meas_flags_t */
};

TEST(test_tdcp_doppler, second_test) {
  /* test forming of tdcp doppler */
  navigation_measurement_t nav_meas[2];
  navigation_measurement_t nav_meas_tdcp[2];
  navigation_measurement_t nav_meas_old[2];

  u8 n_ready = 2;
  u8 n_ready_old = 2;
  double dt = 1.0;

  nav_meas_old[0] = nm1;
  nav_meas_old[1] = nm2;
  nav_meas[0] = nm1_2;
  nav_meas[1] = nm2_2;
  nav_meas_old[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas_old[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[0].raw_measured_doppler = 1;
  nav_meas[0].measured_doppler = 2;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[1].raw_measured_doppler = 3;
  nav_meas[1].measured_doppler = 5;
  nav_meas[1].flags = NAV_MEAS_FLAG_PHASE_VALID;

  u8 n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 2);
  EXPECT_EQ(nav_meas_tdcp[0].raw_computed_doppler, 100);
  EXPECT_EQ(nav_meas_tdcp[0].computed_doppler, 101);
  EXPECT_TRUE(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);
  EXPECT_EQ(nav_meas_tdcp[1].raw_computed_doppler, 200);
  EXPECT_EQ(nav_meas_tdcp[1].computed_doppler, 202);

  /* sort the new measurements the other way around */
  nav_meas_old[0] = nm1;
  nav_meas_old[1] = nm2;
  nav_meas[0] = nm2_2;
  nav_meas[1] = nm1_2;
  nav_meas_old[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas_old[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[1].flags = NAV_MEAS_FLAG_PHASE_VALID;

  n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 2);
  EXPECT_EQ(nav_meas_tdcp[0].raw_computed_doppler, 100);
  EXPECT_TRUE(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);
  EXPECT_EQ(nav_meas_tdcp[1].raw_computed_doppler, 200);
  EXPECT_TRUE(nav_meas_tdcp[1].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);

  /* old measurements only for the second satellite */
  nav_meas_old[0] = nm2;
  nav_meas_old[1] = nm3;
  nav_meas[0] = nm2_2;
  nav_meas[1] = nm1_2;
  nav_meas_old[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas_old[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[1].flags = NAV_MEAS_FLAG_PHASE_VALID;

  n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 2);
  EXPECT_TRUE(!(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID));
  EXPECT_EQ(nav_meas_tdcp[1].raw_computed_doppler, 200);
  EXPECT_TRUE(nav_meas_tdcp[1].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);

  /* new measurements only for one satellite */
  nav_meas_old[0] = nm1;
  nav_meas_old[1] = nm2;
  nav_meas_old[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas_old[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  n_ready_old = 2;
  nav_meas[0] = nm2_2;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  n_ready = 1;

  n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 1);
  EXPECT_EQ(nav_meas_tdcp[0].raw_computed_doppler, 200);
  EXPECT_TRUE(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);

  /* old measurements only for one band of the first satellite */
  nav_meas_old[0] = nm2;
  nav_meas_old[0].sid.sat = 2;
  nav_meas_old[0].sid.code = CODE_GPS_L2CM;
  nav_meas_old[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  n_ready_old = 1;
  nav_meas[0] = nm1_2;
  nav_meas[1] = nm2_2;
  nav_meas[0].sid.sat = 2;
  nav_meas[0].sid.code = CODE_GPS_L1CA;
  nav_meas[1].sid.sat = 2;
  nav_meas[1].sid.code = CODE_GPS_L2CM;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  n_ready = 2;

  n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 2);
  EXPECT_TRUE(!(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID));
  EXPECT_EQ(nav_meas_tdcp[1].raw_computed_doppler, 200);
  EXPECT_TRUE(nav_meas_tdcp[1].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);

  /* cycle slip on nm1, should not have valid calculated Doppler */
  n_ready = 2;
  n_ready_old = 2;
  dt = 1.0;

  nav_meas_old[0] = nm1;
  nav_meas_old[1] = nm2;
  nav_meas[0] = nm1_2;
  nav_meas[1] = nm2_2;
  nav_meas_old[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas_old[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[0].lock_time = 0;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[1].flags = NAV_MEAS_FLAG_PHASE_VALID;

  n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 2);
  EXPECT_TRUE(!(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID));
  EXPECT_EQ(nav_meas_tdcp[1].raw_computed_doppler, 200);
  EXPECT_TRUE(nav_meas_tdcp[1].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);

  /* invalid carrier phase on nm1, should not have valid calculated Doppler */
  n_ready = 2;
  n_ready_old = 2;
  dt = 1.0;

  nav_meas_old[0] = nm1;
  nav_meas_old[1] = nm2;
  nav_meas[0] = nm1_2;
  nav_meas[1] = nm2_2;
  nav_meas_old[0].flags = 0;
  nav_meas_old[1].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[0].flags = NAV_MEAS_FLAG_PHASE_VALID;
  nav_meas[1].flags = NAV_MEAS_FLAG_PHASE_VALID;

  n_ready_tdcp = tdcp_doppler(
      n_ready, nav_meas, n_ready_old, nav_meas_old, nav_meas_tdcp, dt);

  EXPECT_EQ(n_ready_tdcp, 2);
  EXPECT_TRUE(!(nav_meas_tdcp[0].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID));
  EXPECT_EQ(nav_meas_tdcp[1].raw_computed_doppler, 200);
  EXPECT_TRUE(nav_meas_tdcp[1].flags & NAV_MEAS_FLAG_COMP_DOPPLER_VALID);
}

TEST(iono_tropo_usage_test, iono_tropo_test) {
  /*NOTE: this unit test checks correct_iono/correct_tropo function usage only.
   * The iono and tropo correction unit tests are in LNSP */
  u8 n_ready_tdcp = 1;

  static navigation_measurement_t nav_meas_tdcp = {
      22932174.15685,  /* raw_pseudorange */
      22932174.15685,  /* pseudorange */
      32,              /* raw_carrier_phase */
      0.0,             /* carrier_phase */
      1000,            /* raw_measured_doppler */
      1000,            /* measured_doppler */
      1000,            /* raw_computed_doppler */
      1000,            /* computed_doppler */
      0.0,             /* computed_doppler_dt */
      {-9680013.54,    /* sat_pos[0] */
       -15286326.35,   /* sat_pos[1] */
       19429449.38},   /* sat_pos[2] */
      {-1000,          /* sat_vel[0] */
       3000,           /* sat_vel[1] */
       2000},          /* sat_vel[2] */
      {-10,            /* sat_acc[0] */
       30,             /* sat_acc[1] */
       20},            /* sat_acc[2] */
      0,               /* IODE */
      0.0,             /* sat_clock_err */
      0.0,             /* sat_clock_err_rate */
      0,               /* IODC */
      35,              /* cn0 */
      36,              /* lock_time */
      0.0,             /* elevation */
      {479820,         /* tot.tow */
       1875},          /* tot.wn */
      {1,              /* sid.sat */
       CODE_GPS_L1CA}, /* sid.code */
      0xAAAA           /* nav_meas_flags_t */
  };

  double pos_ecef[3];
  double pos_llh[3] = {20.3 * D2R, 149.1 * D2R, 0};
  wgsllh2ecef(pos_llh, pos_ecef);

  double pr_tropo_corrected = 22932123.831141;
  double pr_iono_tropo_corrected = 22932118.957291;
  double doppler_tropo_corrected = 999.521223;
  double doppler_iono_tropo_corrected = 999.517450;

  ionosphere_t i = {{479820, 1875},
                    0.1583e-7,
                    -0.7451e-8,
                    -0.5960e-7,
                    0.1192e-6,
                    0.1290e6,
                    -0.2130e6,
                    0.6554e5,
                    0.3277e6};

  correct_tropo(pos_ecef, n_ready_tdcp, &nav_meas_tdcp);
  correct_iono(pos_ecef, &i, n_ready_tdcp, &nav_meas_tdcp);

  EXPECT_FLOAT_EQ(nav_meas_tdcp.pseudorange, pr_iono_tropo_corrected);

  EXPECT_FLOAT_EQ(nav_meas_tdcp.measured_doppler, doppler_iono_tropo_corrected);

  EXPECT_FLOAT_EQ(nav_meas_tdcp.computed_doppler, doppler_iono_tropo_corrected);

  nav_meas_tdcp.pseudorange = 22932174.156858064;
  nav_meas_tdcp.measured_doppler = 1000;
  nav_meas_tdcp.computed_doppler = 1000;

  correct_tropo(pos_ecef, n_ready_tdcp, &nav_meas_tdcp);

  EXPECT_FLOAT_EQ(nav_meas_tdcp.pseudorange, pr_tropo_corrected);

  EXPECT_FLOAT_EQ(nav_meas_tdcp.measured_doppler, doppler_tropo_corrected);

  EXPECT_FLOAT_EQ(nav_meas_tdcp.computed_doppler, doppler_tropo_corrected);
}
