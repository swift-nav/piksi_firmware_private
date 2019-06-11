#include <stdio.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "gtest/gtest.h"
#include "nav_msg/nav_msg_bds.h"

#undef log_error
#define log_error(...)

/* Reference BDS ephemeris */
static const ephemeris_t ref_eph = {
    {10,                      /* sid.sat */
     CODE_BDS2_B1},           /* sid.code */
    {2.01600000000E+05,       /* toe.tow */
     1998},                   /* toe.wn */
    2.00000000000E+00,        /* ura */
    BDS_FIT_INTERVAL_SECONDS, /* fit_interval [s] */
    1,                        /* valid */
    0,                        /* health_bits */
    0,                        /* source */
    {{.tgd = {{0}},           /* kepler.tgd */
      -3.47640625000E+02,     /* kepler.crc */
      -1.20656250000E+02,     /* kepler.crs */
      -3.98792326450E-06,     /* kepler.cuc */
      1.90609134734E-05,      /* kepler.cus */
      -2.18860805035E-08,     /* kepler.cic */
      -3.92086803913E-07,     /* kepler.cis */
      1.08933108930E-09,      /* kepler.dn */
      -1.43540489556E+00,     /* kepler.m0 */
      4.98019636143E-03,      /* kepler.ecc */
      6.49363065910E+03,      /* kepler.sqrta */
      1.69518318499E+00,      /* kepler.omega0 */
      -2.13366030409E-09,     /* kepler.omegadot */
      -2.75182959435E+00,     /* kepler.w */
      9.20638105072E-01,      /* kepler.inc */
      -9.93255658799E-10,     /* kepler.inc_dot */
      -1.81104638614E-04,     /* kepler.af0 */
      -2.85078627371E-11,     /* kepler.af1 */
      0.00000000000E+00,      /* kepler.af2 */
      {2.01600000000E+05,     /* kepler.toc.tow */
       1998},                 /* kepler.toc.wn */
      40,                     /* kepler.iodc */
      40}}                    /* kepler.iode */

};

/* Reference BDS subframe 1 */
static const u32 ref_subfr1[10] = {0x38901313,
                                   0x12380030,
                                   0x05046242,
                                   0x1c0380cc,
                                   0x1e0a1484,
                                   0x3a0b8ffe,
                                   0x0ef13020,
                                   0x20007494,
                                   0x08647f83,
                                   0x3053e192};
/* Reference BDS subframe 2 */
static const u32 ref_subfr2[10] = {0x38902311,
                                   0x12502f78,
                                   0x2af7a35a,
                                   0x0c5842f1,
                                   0x3c440ae4,
                                   0x0cc3a9fe,
                                   0x09fe5e27,
                                   0x2917f8a6,
                                   0x1d6cae2c,
                                   0x342e5c51};
/* Reference BDS subframe 3 */
static const u32 ref_subfr3[10] = {0x3890331e,
                                   0x126b13e2,
                                   0x204b051d,
                                   0x0d487f3a,
                                   0x3e8fff4f,
                                   0x1155fea1,
                                   0x16da91f5,
                                   0x28a22cf0,
                                   0x374c7ff3,
                                   0x02c09a96};

u16 bch_encoder(u16 input) {
  u16 output = 0;
  u8 gate1 = 1;
  u8 gate2 = 0;
  u8 d0 = 0;
  u8 d1 = 0;
  u8 d2 = 0;
  u8 d3 = 0;
  for (s8 i = 14; i >= 0; i--) {
    if (i < 4) {
      gate1 = 0;
      gate2 = 1;
    }
    /* Do computations */
    u8 x = ((input >> i) & 0x1);
    u8 mem1 = d3 ^ x;
    u8 out_bit = x | (gate2 & mem1);
    output |= (out_bit << i);
    /* Shift registers */
    d3 = d2;
    d2 = d1;
    d1 = (mem1 & gate1) ^ d0;
    d0 = (mem1 & gate1);
  }
  return output;
}

u32 interleave(u16 hi, u16 lo) {
  u32 ret_val = 0;
  for (s8 i = 14; i >= 0; i--) {
    ret_val |= (((hi >> i) & 0x1) << (2 * i + 1));
    ret_val |= (((lo >> i) & 0x1) << (2 * i));
  }
  return ret_val;
}

TEST(nav_msg_bds_tests, bch_decoder_real_data) {
  /* Initialize bit buffer with correct BDS subframe. */
  me_gnss_signal_t mesid = {12, CODE_BDS2_B1};
  nav_msg_bds_t nav_msg1;
  bds_nav_msg_init(&nav_msg1, &mesid);

  nav_msg1.subframe_bits[0] = 0x076FEA35;
  nav_msg1.subframe_bits[1] = 0x3B8B7723;
  nav_msg1.subframe_bits[2] = 0x3B32EBCB;
  nav_msg1.subframe_bits[3] = 0x122A87FA;
  nav_msg1.subframe_bits[4] = 0x1056B834;
  nav_msg1.subframe_bits[5] = 0x07567622;
  nav_msg1.subframe_bits[6] = 0x20809FC3;
  nav_msg1.subframe_bits[7] = 0x3DFFAB54;
  nav_msg1.subframe_bits[8] = 0x215E0A5A;
  nav_msg1.subframe_bits[9] = 0x051581CB;

  /* Make a copy of nav_msg1, since CRC check modifies the message.
   * nav_msg1 is re-used later for bit error test. */
  nav_msg_bds_t nav_msg1_result;
  memcpy(&nav_msg1_result, &nav_msg1, sizeof(nav_msg_bds_t));

  /* CRC should pass with correct data. */
  EXPECT_TRUE(0x3FF == crc_check(&nav_msg1_result));

  /* Initialize bit buffer with correct BDS subframe.
   * This time with opposite polarity. */
  nav_msg_bds_t nav_msg2;
  bds_nav_msg_init(&nav_msg2, &mesid);

  nav_msg2.subframe_bits[0] = 0x289515CA;
  nav_msg2.subframe_bits[1] = 0x0460888D;
  nav_msg2.subframe_bits[2] = 0x04CD1434;
  nav_msg2.subframe_bits[3] = 0x3DD52A5B;
  nav_msg2.subframe_bits[4] = 0x15040104;
  nav_msg2.subframe_bits[5] = 0x38B38962;
  nav_msg2.subframe_bits[6] = 0x177DE81E;
  nav_msg2.subframe_bits[7] = 0x020454EF;
  nav_msg2.subframe_bits[8] = 0x0BC77F50;
  nav_msg2.subframe_bits[9] = 0x22BC4A14;

  /* CRC should pass with correct data. */
  EXPECT_TRUE(0x3FF == crc_check(&nav_msg2));

  /* Test injection of a single bit error. */
  nav_msg_bds_t nav_msg3_result;
  memcpy(&nav_msg3_result, &nav_msg1, sizeof(nav_msg_bds_t));

  /* Loop through all bits injecting single bit error. */
  for (u8 i = 0; i < BDS_WORD_SUBFR; i++) {
    for (u8 j = 0; j < 30; j++) {
      nav_msg3_result.subframe_bits[i] ^= (0x1 << j);
      if ((0 == i) && (j > 14)) {
        /* These are the first 15 bits of the word.
         * They are not encoded, and thus not protected by crc check.
         * CRC check does not notice bit errors here. */
        EXPECT_TRUE(0x3FF == crc_check(&nav_msg3_result));
      } else {
        /* These are the usual bits which are protected by crc check.
         * CRC should catch these errors. */
        EXPECT_FALSE(0x3FF == crc_check(&nav_msg3_result));
      }
      memcpy(&nav_msg3_result, &nav_msg1, sizeof(nav_msg_bds_t));
    }
  }
}

TEST(nav_msg_bds_tests, bch_decoder_random_data) {
  me_gnss_signal_t mesid = {12, CODE_BDS2_B1};
  nav_msg_bds_t nav_msg;
  /* Generate all possible messages and see that decoding succeeds.
   * There are 11 information bits, i.e. 2^11 different messages. */
  u16 msg_max = 2048;
  u16 msg_per_word = 20;
  u8 rounds = ceilf((float)msg_max / msg_per_word);
  u16 rand_msg = 0;
  for (u8 i = 0; i < rounds; i++) {
    /* In first word only lo is encoded, and no interleaving. */
    bds_nav_msg_init(&nav_msg, &mesid);
    u32 hi = ((rand_msg++) << 4) & 0x7FF0;
    u32 lo = ((rand_msg++) << 4) & 0x7FF0;
    lo = bch_encoder(lo);
    nav_msg.subframe_bits[0] = (hi << 15) | lo;

    for (u8 j = 1; j < BDS_WORD_SUBFR; j++) {
      /* The following words are encoded and interleaved. */
      hi = ((rand_msg++) << 4) & 0x7FF0;
      lo = ((rand_msg++) << 4) & 0x7FF0;
      hi = bch_encoder(hi);
      lo = bch_encoder(lo);
      nav_msg.subframe_bits[j] = interleave(hi, lo);
    }

    /* CRC should pass with correct data. */
    EXPECT_TRUE(0x3FF == crc_check(&nav_msg));
  }
}

TEST(nav_msg_bds_tests, ephemeris_decoding) {
  me_gnss_signal_t mesid = {10, CODE_BDS2_B1};
  nav_msg_bds_t nav_msg;
  bds_nav_msg_init(&nav_msg, &mesid);

  /* Copy reference subframes 1-3 for processing. */
  memcpy(&nav_msg.page_words[0], &ref_subfr1, sizeof(ref_subfr1));
  memcpy(&nav_msg.page_words[10], &ref_subfr2, sizeof(ref_subfr2));
  memcpy(&nav_msg.page_words[20], &ref_subfr3, sizeof(ref_subfr3));
  /* Mark all words good to enable decoding. */
  nav_msg.goodwords_mask = -1;

  bds_d1_decoded_data_t dd_d1nav;
  memset(&dd_d1nav, 0, sizeof(bds_d1_decoded_data_t));
  bds_d1_processing(&nav_msg, &dd_d1nav);

  ephemeris_t tmp_eph;
  memcpy(&tmp_eph, &ref_eph, sizeof(ref_eph));
  /* Set TGD2 into copy of ref_eph. */
  tmp_eph.kepler.tgd.bds_s[0] = 5.6E-09;
  tmp_eph.kepler.tgd.bds_s[1] = 3.0E-09;
  /* Adjust ref_eph times to GPS time. */
  add_secs(&tmp_eph.toe, BDS_SECOND_TO_GPS_SECOND);
  add_secs(&tmp_eph.kepler.toc, BDS_SECOND_TO_GPS_SECOND);

  /* Introduce few helper variables to make following lines shorter. */
  ephemeris_t decoded_eph = dd_d1nav.ephemeris;
  ephemeris_kepler_t decoded_k = dd_d1nav.ephemeris.kepler;
  ephemeris_kepler_t tmp_k = tmp_eph.kepler;

  /* Check that decoded ephemeris matches with reference ephemeris. */
  /* Note that ephemeris_equal() function would fail due to float precision.
   */
  EXPECT_EQ(tmp_eph.sid.sat, decoded_eph.sid.sat);
  EXPECT_EQ(tmp_eph.sid.code, decoded_eph.sid.code);
  EXPECT_FLOAT_EQ(tmp_eph.toe.tow, decoded_eph.toe.tow);
  EXPECT_EQ(tmp_eph.toe.wn, decoded_eph.toe.wn);
  EXPECT_FLOAT_EQ(tmp_eph.ura, decoded_eph.ura);
  EXPECT_EQ(tmp_eph.fit_interval, decoded_eph.fit_interval);
  EXPECT_EQ(tmp_eph.valid, decoded_eph.valid);
  EXPECT_EQ(tmp_eph.health_bits, decoded_eph.health_bits);
  EXPECT_FLOAT_EQ(tmp_k.tgd.bds_s[0], decoded_k.tgd.bds_s[0]);
  EXPECT_FLOAT_EQ(tmp_k.tgd.bds_s[1], decoded_k.tgd.bds_s[1]);
  EXPECT_FLOAT_EQ(tmp_k.crc, decoded_k.crc);
  EXPECT_FLOAT_EQ(tmp_k.crs, decoded_k.crs);
  EXPECT_FLOAT_EQ(tmp_k.cuc, decoded_k.cuc);
  EXPECT_FLOAT_EQ(tmp_k.cus, decoded_k.cus);
  EXPECT_FLOAT_EQ(tmp_k.cic, decoded_k.cic);
  EXPECT_FLOAT_EQ(tmp_k.cis, decoded_k.cis);
  EXPECT_FLOAT_EQ(tmp_k.dn, decoded_k.dn);
  EXPECT_FLOAT_EQ(tmp_k.m0, decoded_k.m0);
  EXPECT_FLOAT_EQ(tmp_k.ecc, decoded_k.ecc);
  EXPECT_FLOAT_EQ(tmp_k.sqrta, decoded_k.sqrta);
  EXPECT_FLOAT_EQ(tmp_k.omega0, decoded_k.omega0);
  EXPECT_FLOAT_EQ(tmp_k.omegadot, decoded_k.omegadot);
  EXPECT_FLOAT_EQ(tmp_k.w, decoded_k.w);
  EXPECT_FLOAT_EQ(tmp_k.inc, decoded_k.inc);
  EXPECT_FLOAT_EQ(tmp_k.inc_dot, decoded_k.inc_dot);
  EXPECT_FLOAT_EQ(tmp_k.af0, decoded_k.af0);
  EXPECT_FLOAT_EQ(tmp_k.af1, decoded_k.af1);
  EXPECT_FLOAT_EQ(tmp_k.af2, decoded_k.af2);
  EXPECT_FLOAT_EQ(tmp_k.toc.tow, decoded_k.toc.tow);
  EXPECT_EQ(tmp_k.toc.wn, decoded_k.toc.wn);
  EXPECT_EQ(tmp_k.iodc, decoded_k.iodc);
  EXPECT_EQ(tmp_k.iode, decoded_k.iode);
}
