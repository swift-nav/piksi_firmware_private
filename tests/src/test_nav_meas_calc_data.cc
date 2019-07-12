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
#include <starling/observation.h>
#include <stdio.h>
#include <swiftnav/ch_meas.h>
#include <swiftnav/constants.h>
#include <swiftnav/coord_system.h>
#include <swiftnav/correct_iono_tropo.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/ionosphere.h>
#include <swiftnav/logging.h>
#include <swiftnav/nav_meas.h>

#include "board/me_max_channels.h"
#include "calc/calc_nav_meas.h"
#include "gtest/gtest.h"

/* Real measurements from tracking (Piksi v3 board) channel for L1C/A, SV 31. */
static const channel_measurement_t l1ca_meas_in = {
    .sid = {31,                  /* sid.sat */
            CODE_GPS_L1CA},      /* sid.code */
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
    .sid = {12,                 /* .sid.sat */
            CODE_GPS_L2CM},     /* .sid.code */
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
  obs_array_t obs_array_l1ca, obs_array_l2cm;
  starling_obs_t* out_l1ca = &obs_array_l1ca.observations[0];
  starling_obs_t* out_l2cm = &obs_array_l2cm.observations[0];

  gps_time_t rec_time = {
      206957.3995198214543052, /* .tow */
      1899                     /* .wn */
  };

  calc_navigation_measurements(1, &l1ca_meas_in, &obs_array_l1ca, &rec_time);
  log_debug(" ***** L1CA: *****\n");
  log_debug("pseudorange = %30.20f\n", out_l1ca->pseudorange);
  log_debug("carrier_phase = %30.20f\n", out_l1ca->carrier_phase);
  log_debug("measured_doppler = %30.20f\n", out_l1ca->doppler);
  log_debug("cn0 = %30.20f\n", out_l1ca->cn0);
  log_debug("lock_time = %30.20f\n", out_l1ca->lock_time);
  log_debug(
      "tow = %30.20f, wn = %d\n", obs_array_l1ca.t.tow, obs_array_l1ca.t.wn);
  log_debug("sat = %u, code = %u\n",
            (unsigned int)out_l1ca->sid.sat,
            (unsigned int)out_l1ca->sid.code);
  log_debug("TOR = %30.20f\n", obs_array_l1ca.t.tow);

  calc_navigation_measurements(1, &l2cm_meas_in, &obs_array_l2cm, &rec_time);
  log_debug(" \n***** L2CM: *****\n");
  log_debug("pseudorange = %30.20f\n", out_l2cm->pseudorange);
  log_debug("carrier_phase = %30.20f\n", out_l2cm->carrier_phase);
  log_debug("measured_doppler = %30.20f\n", out_l2cm->doppler);
  log_debug("cn0 = %30.20f\n", out_l2cm->cn0);
  log_debug("lock_time = %30.20f\n", out_l2cm->lock_time);
  log_debug(
      "tow = %30.20f, wn = %d\n", obs_array_l2cm.t.tow, obs_array_l2cm.t.wn);
  log_debug("sat = %u, code = %u\n",
            (unsigned int)out_l2cm->sid.sat,
            (unsigned int)out_l2cm->sid.code);
  log_debug("TOR = %30.20f\n", obs_array_l2cm.t.tow);

  double check_value;

  /* L1 and L2 pseudoranges differ some meters because of ionosphere and
   * inter-signal delay */
  check_value = fabs(out_l1ca->pseudorange - out_l2cm->pseudorange);
  EXPECT_LT(check_value, 15);

  check_value =
      fabs(out_l1ca->doppler / out_l2cm->doppler - GPS_L1_HZ / GPS_L2_HZ);
  EXPECT_LT(check_value, 0.003);

  check_value = fabs(obs_array_l1ca.t.tow - rec_time.tow);
  EXPECT_FLOAT_EQ(check_value, 0.0);

  check_value = fabs(obs_array_l2cm.t.tow - rec_time.tow);
  EXPECT_FLOAT_EQ(check_value, 0.0);
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
