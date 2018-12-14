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
#define DEBUG 0
#include <stdio.h>

#include <swiftnav/coord_system.h>
#include <swiftnav/logging.h>

#include "gtest/gtest.h"

#include "sv_visibility/sv_visibility.h"

static ephemeris_t ep;

typedef struct {
  double tow;          /* TOW when LGF made, sec */
  s16 wn;              /* week number when LGF made */
  float user_velocity; /* estimated maximum user speed during "offline" */
  u32 time_delta;      /* "offline" time */
  bool visible;        /* calculation result: visible parameter */
  bool known;          /* calculation result: known parameter */
} sv_vis_tests_case_t;

#define TOW 115200.0
#define WN 1899

static sv_vis_tests_case_t t_cases[8] = {
    /*LGF time is same as ephemeris time.
    * User did not move.Calc visibility at same place */
    {TOW, WN, 0., 0, true, true},  // 0
    /*LGF time is 1 year later than ephemeris time.
    * User did not move.Calc visibility at same place */
    {201600., WN + 52, 0., 0, true, true},  // 1
    /*LGF time is one week later than ephemeris time.
    * User did not move.Calc visibility at same place */
    {TOW, 1900, 0., 0, true, true},  // 2
    /*LGF time is nine days later than ephemeris time.
    * User did not move. Calc visibility at same place */
    {TOW + 172800., 1900, 0., 0, true, true},  // 3
    /*LGF time is nine days later than ephemeris time.
    * User moved 200 km/h during 1 hour. */
    {TOW + 172800., 1900, 200. * 1000. / 3600., 3600, true, true},  // 4
    /*LGF time is nine days later than ephemeris time.
    * User moved 850 km/h during 1 hour. */
    {TOW + 172800., 1900, 850. * 1000. / 3600., 3600, true, true},  // 5
    /*LGF time is nine days later than ephemeris time.
    * User moved 850 km/h during 12 hour. */
    {TOW + 172800., 1900, 850. * 1000. / 3600., 3600 * 12, false, false},  // 6
    /*LGF time is nine days later than ephemeris time.
    * User moved 850 km/h during 3 hour. */
    {TOW + 172800., 1900, 850. * 1000. / 3600., 3600 * 3, false, false},  // 7
};

TEST(sv_visibility_test, test_gps_sv_visibility) {
  /* real ephemeris for SV 15 at 8:00, 30-May-2016 */
  ep.sid.sat = 15;
  ep.sid.code = CODE_GPS_L1CA;
  ep.toe.tow = TOW;
  ep.toe.wn = WN;
  ep.ura = 1.0;
  ep.fit_interval = 14400;
  ep.valid = 1;
  ep.health_bits = 0;
  ep.kepler.tgd.gps_s = -0.00000001071020960808;
  ep.kepler.crc = 222.34375000000000000000;
  ep.kepler.crs = 62.87500000000000000000;
  ep.kepler.cuc = 0.00000356882810592651;
  ep.kepler.cus = 0.00000734068453311920;
  ep.kepler.cic = 0.00000002793967723846;
  ep.kepler.cis = 0.00000012852251529694;
  ep.kepler.dn = 0.00000000562166273625;
  ep.kepler.m0 = 1.32453305390110598339;
  ep.kepler.ecc = 0.00812081422191113234;
  ep.kepler.sqrta = 5153.59741973876953125000;
  ep.kepler.omega0 = -0.25515026323261330576;
  ep.kepler.omegadot = -0.00000000872250618454;
  ep.kepler.w = 0.48344690815566465636;
  ep.kepler.inc = 0.93105119838528094256;
  ep.kepler.inc_dot = 0.00000000008536069847;
  ep.kepler.af0 = -0.00031310319900512695;
  ep.kepler.af1 = -0.00000000000170530257;
  ep.kepler.af2 = 0.00000000000000000000;
  ep.kepler.toc.tow = 115200.0;
  ep.kepler.toc.wn = 1899;
  ep.kepler.iodc = 40;
  ep.kepler.iode = 40;

  sv_vis_config_t config;
  bool visible = false;
  bool known = false;

  memset(&config, 0, sizeof(config));

  config.e = &ep;
  double lgf_llh[3] = {61.4484385 * D2R, 23.8651107 * D2R, 0};
  wgsllh2ecef(lgf_llh, config.lgf_ecef);

  u8 i;
  for (i = 0; i < sizeof(t_cases) / sizeof(t_cases[0]); i++) {
    config.lgf_time.tow = t_cases[i].tow;
    config.lgf_time.wn = t_cases[i].wn;
    config.user_velocity = t_cases[i].user_velocity;
    config.time_delta = t_cases[i].time_delta;
    sv_visibility_status_get(&config, &visible, &known);
    log_debug("%u: VISIBLE = %u, KNOWN = %u\n", i, visible, known);
    EXPECT_TRUE(visible == t_cases[i].visible && known == t_cases[i].known);
  }

  /* Extra test case: LGF is the North pole, user does not move */
  memset(lgf_llh, 0, sizeof(lgf_llh));
  wgsllh2ecef(lgf_llh, config.lgf_ecef);
  config.lgf_time.tow = t_cases[0].tow;
  config.lgf_time.wn = t_cases[0].wn;
  config.user_velocity = t_cases[0].user_velocity;
  config.time_delta = t_cases[0].time_delta;
  sv_visibility_status_get(&config, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
  EXPECT_TRUE(visible && known);

  /* Extra test case: ephemeris is not provided (unknown) */
  i++;
  config.e = NULL;
  sv_visibility_status_get(&config, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
  EXPECT_TRUE(!visible && !known);

  /* Extra test case: config is not provided */
  i++;
  sv_visibility_status_get(NULL, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
  EXPECT_TRUE(!visible && !known);
}

#undef TOW
#undef WN
#define TOW 212400.0
#define WN 1945

static ephemeris_t ep_glo;

static sv_vis_tests_case_t t_cases_glo[7] = {
    /*LGF time is same as ephemeris time.
    * User did not move.Calc visibility at same place */
    {TOW, WN, 0., 0, true, true},  // 0
    /*LGF time is 15 mins later than ephemeris time.
    * User did not move.Calc visibility at same place */
    {TOW + 15 * 60, WN, 0., 0, true, true},  // 1
    /*LGF time is one GLO period later than ephemeris time.
    * User did not move. Calc visibility at same place */
    {TOW + 40544, WN, 0., 0, true, true},  // 2
    /*LGF time is 19 hours later than ephemeris time.
    * User's moving 100 km/h during 17 minutes  */
    {TOW + 280800, WN, 100. * 1000. / 3600., 60 * 17, true, true},  // 3
    /*LGF time is 15 mins later than ephemeris time.
    * User moved 850 km/h during 3 hour. */
    {TOW + 900, WN, 850. * 1000. / 3600., 3600 * 3, false, true},  // 4
    /*LGF time is one hour later than ephemeris time.
    * User moved 850 km/h during 1 hour. */
    {TOW + 3600, WN, 850. * 1000. / 3600., 3600, false, true},  // 5
    /*LGF time is 5 mins later than ephemeris time.
    * User moved 60 km/h during 5 mins. */
    {TOW + 300, WN, 60. * 1000. / 3600., 300, true, true},  // 6
};

TEST(sv_visibility_test, test_glo_sv_visibility) {
  /* real ephemeris for GLO SV 3 at 11:00, 18-Apr-2017 */
  ep_glo.sid.sat = 1;
  ep_glo.sid.code = CODE_GLO_L1OF;
  ep_glo.toe.tow = TOW;
  ep_glo.toe.wn = WN;
  ep_glo.ura = 1.0;
  ep_glo.fit_interval = 10000;
  ep_glo.valid = 1;
  ep_glo.health_bits = 0;
  ep_glo.glo.gamma = 1.81898940354585648e-12;
  ep_glo.glo.tau = -9.71024855971336365e-05;
  ep_glo.glo.pos[0] = -1.4453039062500000e+07;
  ep_glo.glo.pos[1] = -6.9681713867187500e+06;
  ep_glo.glo.pos[2] = 1.9873773925781250e+07;
  ep_glo.glo.vel[0] = -1.4125013351440430e+03;
  ep_glo.glo.vel[1] = -2.3216266632080078e+03;
  ep_glo.glo.vel[2] = -1.8360681533813477e+03;
  ep_glo.glo.acc[0] = 0;
  ep_glo.glo.acc[1] = 0;
  ep_glo.glo.acc[2] = -2.79396772384643555e-06;

  sv_vis_config_t config;
  bool visible = false;
  bool known = false;

  log_debug("GLO SV visibility test\n");

  memset(&config, 0, sizeof(config));

  config.e = &ep_glo;
  double lgf_llh[3] = {61.4484385 * D2R, 23.8651107 * D2R, 0};
  wgsllh2ecef(lgf_llh, config.lgf_ecef);

  u8 i;
  for (i = 0; i < sizeof(t_cases_glo) / sizeof(t_cases_glo[0]); i++) {
    config.lgf_time.tow = t_cases_glo[i].tow;
    config.lgf_time.wn = t_cases_glo[i].wn;
    config.user_velocity = t_cases_glo[i].user_velocity;
    config.time_delta = t_cases_glo[i].time_delta;
    sv_visibility_status_get(&config, &visible, &known);
    log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
    EXPECT_TRUE(visible == t_cases_glo[i].visible &&
                known == t_cases_glo[i].known);
  }

  /* Extra test case: LGF is the North pole, user does not move */
  memset(lgf_llh, 0, sizeof(lgf_llh));
  lgf_llh[0] = 90 * D2R;
  wgsllh2ecef(lgf_llh, config.lgf_ecef);
  config.lgf_time.tow = t_cases_glo[0].tow;
  config.lgf_time.wn = t_cases_glo[0].wn;
  config.user_velocity = t_cases_glo[0].user_velocity;
  config.time_delta = t_cases_glo[0].time_delta;
  sv_visibility_status_get(&config, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
  EXPECT_TRUE(visible && known);

  /* Extra test case: ephemeris is not provided (unknown) */
  i++;
  config.e = NULL;
  sv_visibility_status_get(&config, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u\n", i, visible, known);
  EXPECT_TRUE(!visible && !known);

  /* Extra test case: config is not provided */
  i++;
  sv_visibility_status_get(NULL, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u\n", i, visible, known);
  EXPECT_TRUE(!visible && !known);
}

#undef TOW
#undef WN
#define TOW 198100.0
#define WN 1976

static ephemeris_t ep_sbas;

static sv_vis_tests_case_t t_cases_sbas[7] = {
    /*LGF time is same as ephemeris time.
    * User did not move.Calc visibility at same place */
    {TOW, WN, 0., 0, true, true},  // 0
    /*LGF time is 15 mins later than ephemeris time.
    * User did not move.Calc visibility at same place */
    {TOW + 15 * 60, WN, 0., 0, true, true},  // 1
    /*LGF time is one GLO period later than ephemeris time.
    * User did not move. Calc visibility at same place */
    {TOW + 40544, WN, 0., 0, true, true},  // 2
    /*LGF time is 19 hours later than ephemeris time.
    * User's moving 100 km/h during 17 minutes */
    {TOW + 280800, WN, 100. * 1000. / 3600., 60 * 17, true, true},  // 3
    /*LGF time is 15 mins later than ephemeris time.
    * User moved 850 km/h during 3 hour. */
    {TOW + 900, WN, 850. * 1000. / 3600., 3600 * 3, true, true},  // 4
    /*LGF time is one hour later than ephemeris time.
    * User moved 850 km/h during 5 hours. */
    {TOW + 3600, WN, 850. * 1000. / 3600., 3600 * 5, false, false},  // 5
    /*LGF time is 5 mins later than ephemeris time.
    * User moved 60 km/h during 5 mins. */
    {TOW + 300, WN, 60. * 1000. / 3600., 300, true, true},  // 6
};

TEST(sv_visibility_test, test_sbas_sv_visibility) {
  /* real ephemeris for EGNOS SV 136 at 7:01:40, 21-Nov-2017
   * taken from here
   * ftp://ems.estec.esa.int/pub/PRN136/y2017/d325/h07.ems
   * MT 9 parsed */
  ep_sbas.sid.sat = 136;
  ep_sbas.sid.code = CODE_SBAS_L1CA;
  ep_sbas.toe.tow = TOW;
  ep_sbas.toe.wn = WN;
  ep_sbas.ura = 0xf;
  ep_sbas.fit_interval = 10000;
  ep_sbas.valid = 1;
  ep_sbas.health_bits = 0;
  ep_sbas.xyz.pos[0] = 42003688.f;
  ep_sbas.xyz.pos[1] = 3674846.96f;
  ep_sbas.xyz.pos[2] = 0.f;
  ep_sbas.xyz.vel[0] = 0.f;
  ep_sbas.xyz.vel[1] = 0.f;
  ep_sbas.xyz.vel[2] = 0.f;
  ep_sbas.xyz.acc[0] = 0.f;
  ep_sbas.xyz.acc[1] = 0.f;
  ep_sbas.xyz.acc[2] = 0.f;
  ep_sbas.xyz.a_gf0 = 0.f;
  ep_sbas.xyz.a_gf1 = 0.f;

  sv_vis_config_t config;
  bool visible = false;
  bool known = false;

  log_debug("SBAS SV visibility test\n");

  memset(&config, 0, sizeof(config));

  config.e = &ep_sbas;
  double lgf_llh[3] = {61.4484385 * D2R, 23.8651107 * D2R, 0};
  wgsllh2ecef(lgf_llh, config.lgf_ecef);

  u8 i;
  for (i = 0; i < sizeof(t_cases_sbas) / sizeof(t_cases_sbas[0]); i++) {
    config.lgf_time.tow = t_cases_sbas[i].tow;
    config.lgf_time.wn = t_cases_sbas[i].wn;
    config.user_velocity = t_cases_sbas[i].user_velocity;
    config.time_delta = t_cases_sbas[i].time_delta;
    sv_visibility_status_get(&config, &visible, &known);
    log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
    EXPECT_TRUE(visible == t_cases_sbas[i].visible &&
                known == t_cases_sbas[i].known);
  }

  /* Extra test case: LGF is the North pole, user does not move */
  memset(lgf_llh, 0, sizeof(lgf_llh));
  lgf_llh[0] = 90 * D2R;
  wgsllh2ecef(lgf_llh, config.lgf_ecef);
  config.lgf_time.tow = t_cases_sbas[0].tow;
  config.lgf_time.wn = t_cases_sbas[0].wn;
  config.user_velocity = t_cases_sbas[0].user_velocity;
  config.time_delta = t_cases_sbas[0].time_delta;
  sv_visibility_status_get(&config, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u", i, visible, known);
  EXPECT_TRUE(!visible && known);

  /* Extra test case: ephemeris is not provided (unknown) */
  i++;
  config.e = NULL;
  sv_visibility_status_get(&config, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u\n", i, visible, known);
  EXPECT_TRUE(!visible && !known);

  /* Extra test case: config is not provided */
  i++;
  sv_visibility_status_get(NULL, &visible, &known);
  log_debug("%u: VISIBLE = %u, KNOWN = %u\n", i, visible, known);
  EXPECT_TRUE(!visible && !known);
}
