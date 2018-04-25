#include <stdio.h>
#include <string.h>
#include <cmath>

#include <libswiftnav/logging.h>

#include "gtest/gtest.h"
#include "nav_msg/nav_msg_bds.h"

#undef log_error
#define log_error(...)

#define LOW_TOL 1e-6
#define HIGH_TOL 1e-1

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
  me_gnss_signal_t mesid = {12, CODE_BDS2_B11};
  nav_msg_bds_t nav_msg1;
  bds_nav_msg_init(&nav_msg1, mesid.sat);

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
  EXPECT_TRUE(crc_check(&nav_msg1_result));

  /* Initialize bit buffer with correct BDS subframe.
   * This time with opposite polarity. */
  nav_msg_bds_t nav_msg2;
  bds_nav_msg_init(&nav_msg2, mesid.sat);

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
  EXPECT_TRUE(crc_check(&nav_msg2));

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
        EXPECT_TRUE(crc_check(&nav_msg3_result));
      } else {
        /* These are the usual bits which are protected by crc check.
         * CRC should catch these errors. */
        EXPECT_FALSE(crc_check(&nav_msg3_result));
      }
      memcpy(&nav_msg3_result, &nav_msg1, sizeof(nav_msg_bds_t));
    }
  }
}

TEST(nav_msg_bds_tests, bch_decoder_random_data) {
  me_gnss_signal_t mesid = {12, CODE_BDS2_B11};
  nav_msg_bds_t nav_msg;
  /* Generate all possible messages and see that decoding succeeds.
   * There are 11 information bits, i.e. 2^11 different messages. */
  u16 msg_max = 2048;
  u16 msg_per_word = 20;
  u8 rounds = ceilf((float)msg_max / msg_per_word);
  u16 rand_msg = 0;
  for (u8 i = 0; i < rounds; i++) {
    /* In first word only lo is encoded, and no interleaving. */
    bds_nav_msg_init(&nav_msg, mesid.sat);
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
    EXPECT_TRUE(crc_check(&nav_msg));
  }
}
