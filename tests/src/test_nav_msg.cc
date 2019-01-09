#include "gtest/gtest.h"
#include "nav_msg/nav_msg.h"

#include <stdio.h>
#include <string.h>

#define BITSTREAM_LENGTH_BITS (56 * 32)
#define FIRST_PREAMBLE_BIT_INDEX (55)
#define SECOND_PREAMBLE_BIT_INDEX (355)

/* Normal preamble found starting from zero-based bit index 55.
 * Repeats thereafter every 300 bits, until index 1550*/
const u32 normal_bit_stream[56] = {
    0xAF263520, 0x849B7116, 0x1DB102A4, 0x116A0000, 0x8496048D, 0xB8C5FFBE,
    0x4FAC270C, 0x2DBD381C, 0xCA9E61D9, 0xF5FF58DA, 0x7FCF8F15, 0x9161DB10,
    0x2A415828, 0xB6527B4C, 0x274ECA60, 0x575FE050, 0x8682920E, 0x2D2EDF86,
    0x9EEFCB09, 0xBACB1680, 0x16A1161D, 0xB102A419, 0xAC881555, 0x56555555,
    0x79555555, 0xE5555557, 0x9555555E, 0x55555579, 0x555555E5, 0x55555791,
    0x61DB102A, 0x41D331DB, 0x20007E71, 0x78AEF2CC, 0x3A530EDD, 0x1FA501EC,
    0x18903208, 0x4D801FFD, 0x697F6942, 0x79161DB1, 0x02A42151, 0x80DF1ED9,
    0x15BDBCDD, 0x596A2197, 0xCE781E54, 0xAA354DF4, 0xCD2F001A, 0xF2635208,
    0x49B71161, 0xDB102A42, 0x56080008, 0x496048DB, 0x8C5FFBE4, 0xFAC270C2,
    0xDBD381CC, 0xA9E61D9F};

/* Inverse preamble found starting from zero-based bit index 55. f
 * Repeats thereafter every 300 bits, until index 1550*/
const u32 inverse_bit_stream[56] = {
    0x4147B7DF, 0x7AC0B6E9, 0xE24EFD5B, 0xEE95FFF0, 0xBD96E7CB, 0x22E1FFFA,
    0x48265AE2, 0xCCBD2677, 0x55738A54, 0x0600B3EC, 0xDDA00BF6, 0xEE9E24EF,
    0xD5BEA7D7, 0x49AD8B5C, 0xE74EDD9F, 0xA8400850, 0x8682920E, 0x1C8A1F86,
    0x755C4202, 0xAF6D1680, 0x1606E9E2, 0x4EFD5BE6, 0x5377EAAA, 0xA9AAAAAA,
    0x86AAAAAA, 0x1AAAAAA8, 0x6AAAAAA1, 0xAAAAAA86, 0xAAAAAA1A, 0xAAAAA86E,
    0x9E24EFD5, 0xBE2CCE24, 0xDFFF818E, 0x87510D33, 0xC5ACF122, 0xE05AFE12,
    0x329DA5F7, 0xADBFE001, 0xE45076EC, 0x4EE9E24E, 0xFD5BDEAE, 0x7681A69A,
    0xE243CFEB, 0xE741A618, 0x5A57BB2A, 0x42AD7208, 0xD8D0FBE4, 0x147B7DF7,
    0xAC0B6E9E, 0x24EFD5BD, 0xA9F7FF0B, 0xD96E7CB2, 0x2E1FFFA4, 0x8265AE2C,
    0xCBD26775, 0x5738A540};

/* Additional L1CA bit streams, which can be used for unit tests. */
/*
const u32 bit_stream1[56] = {
    0x9E1BB520, 0x853E5116, 0x1DB102A4, 0x116A1FFC, 0x89CBBC4C, 0xB58FFFF3,
    0xB1D0E35D, 0xBFBC3B1D, 0x03DC6003, 0x29FF4E0E, 0x93487DE3, 0x1161DB10,
    0x2A415828, 0xB65274A3, 0x18B12260, 0x57BFF7AF, 0x797D6DF1, 0xE375E079,
    0x8AA3BDFD, 0x5092E97F, 0xE9F9161D, 0xB102A419, 0xAC881555, 0x56555555,
    0x79555555, 0xE5555557, 0x9555555E, 0x55555579, 0x555555E5, 0x55555791,
    0x61DB102A, 0x41D331DB, 0x20007E71, 0x78AEF2CC, 0x3A530EDD, 0x1FA501EC,
    0x0ABCB5F7, 0xA9800005, 0xF97D0942, 0xF9161DB1, 0x02A42151, 0x8D203214,
    0xE603D8F9, 0xE6E6E20F, 0xF597FEAD, 0xC2EFF1F8, 0xA4AF7C19, 0xE1BB5208,
    0x53E51161, 0xDB102A42, 0x5609FFC8, 0x9CBBC4CB, 0x58FFFF3B, 0x1D0E35DB,
    0xFBC3B1D0, 0x3DC60032};
*/
/*
const u32 bit_stream2[56] = {
    0x4B132FDF, 0x7AD93116, 0x1DB102A4, 0x116A0009, 0x6F8ADAE9, 0xBF87FF43,
    0xB0E18AAD, 0xFF3CA5E9, 0xF8451521, 0x91FF5140, 0x104F6B5B, 0x1161DB10,
    0x2A415828, 0xB65274A3, 0x18B12260, 0x57BFF7AF, 0x797D6DF1, 0xE375E079,
    0x8AA3BDFD, 0x5092E97F, 0xE9F9161D, 0xB102A419, 0xAC881555, 0x56555555,
    0x79555555, 0xE5555557, 0x9555555E, 0x55555579, 0x555555E5, 0x55555791,
    0x61DB102A, 0x41D331DB, 0x20007E71, 0x78AEF2CC, 0x3A530EDD, 0x1FA501ED,
    0xD7104A08, 0x52BFE000, 0xC685C5A0, 0x69161DB1, 0x02A42151, 0x813FFDFD,
    0xE64E9CF1, 0x34EEB82F, 0xFD67DAE7, 0xF1A2E7F7, 0x0DAF53E4, 0xB132FDF7,
    0xAD931161, 0xDB102A42, 0x56080096, 0xF8ADAE9B, 0xF87FF43B, 0x0E18AADF,
    0xF3CA5E9F, 0x84515219};
*/
/*
const u32 bit_stream3[56] = {
    0x53329DDF, 0x7AC0B6E9, 0xE24EFD5B, 0xEE95FFFA, 0x9666CC0D, 0xA0F3FF81,
    0xB4651EA4, 0x3FFC6F68, 0x66394E91, 0x0E00B99C, 0x42D88E12, 0xEE9E24EF,
    0xD5BEA7D7, 0x49AD8B5C, 0xE74EDD9F, 0xA8400850, 0x8682920E, 0x1C8A1F86,
    0x755C4202, 0xAF6D1680, 0x1606E9E2, 0x4EFD5BE6, 0x5377EAAA, 0xA9AAAAAA,
    0x86AAAAAA, 0x1AAAAAA8, 0x6AAAAAA1, 0xAAAAAA86, 0xAAAAAA1A, 0xAAAAA86E,
    0x9E24EFD5, 0xBE2CCE24, 0xDFFF818E, 0x87510D33, 0xC5ACF122, 0xE05AFE12,
    0x4BDD25F7, 0xAE3FE00D, 0xF18BE7A2, 0x1EE9E24E, 0xFD5BDEAE, 0x749FBB23,
    0xE1BBA3FE, 0xA73FA59F, 0xEDFFE615, 0x25F0E5F9, 0x05AF2BE5, 0x3329DDF7,
    0xAC0B6E9E, 0x24EFD5BD, 0xA9F7FFA9, 0x666CC0DA, 0x0F3FF81B, 0x4651EA43,
    0xFFC6F686, 0x6394E910};
*/
/*
const u32 bit_stream4[56] = {
    0x598A41DF, 0x7AD1CEE9, 0xE24EFD5B, 0xEE95FFF5, 0xD0FDA4E0, 0x998600C7,
    0xB213CA70, 0x9FBCA76C, 0x50B40ADA, 0x4A00B1F0, 0x208F99A2, 0x6E9E24EF,
    0xD5BEA7D7, 0x49AD8B5C, 0xE74EDD9F, 0xA8400850, 0x8682920E, 0x1C8A1F86,
    0x755C4202, 0xAF6D1680, 0x1606E9E2, 0x4EFD5BE6, 0x5377EAAA, 0xA9AAAAAA,
    0x86AAAAAA, 0x1AAAAAA8, 0x6AAAAAA1, 0xAAAAAA86, 0xAAAAAA1A, 0xAAAAA86E,
    0x9E24EFD5, 0xBE2CCE24, 0xDFFF818E, 0x87510D33, 0xC5ACF122, 0xE05AFE12,
    0x561F75F7, 0xAE001FFF, 0xFA8E52EA, 0xEEE9E24E, 0xFD5BDEAE, 0x7DC02F2B,
    0xE437B25B, 0x0DF9EA28, 0x0F27CAFD, 0xA0111E08, 0x8550BFE5, 0x98A41DF7,
    0xAD1CEE9E, 0x24EFD5BD, 0xA9F7FF5D, 0x0FDA4E09, 0x98600C7B, 0x213CA709,
    0xFBCA76C5, 0x0B40ADA4};
*/
/*
const u32 bit_stream5[56] = {
    0x8F8C19DF, 0x7AC0B6E9, 0xE24EFD5B, 0xEE95E002, 0x865571AD, 0x261A0035,
    0xB10FECDD, 0xB88504DF, 0x9C2E3B94, 0xD200BA91, 0x3E17EBA8, 0xEE9E24EF,
    0xD5BEA7D7, 0x49AD8B5C, 0xE74EDD9F, 0xA8400850, 0x8682920E, 0x1C8A1F86,
    0x755C4202, 0xAF6D1680, 0x1606E9E2, 0x4EFD5BE6, 0x5377EAAA, 0xA9AAAAAA,
    0x86AAAAAA, 0x1AAAAAA8, 0x6AAAAAA1, 0xAAAAAA86, 0xAAAAAA1A, 0xAAAAA86E,
    0x9E24EFD5, 0xBE2CCE24, 0xDFFF818E, 0x87510D33, 0xC5ACF122, 0xE05AFE13,
    0xDEE1EA08, 0x5580000E, 0xAF9840AD, 0x7EE9E24E, 0xFD5BDEAE, 0x785EE066,
    0x1A8690B5, 0x66BA72AF, 0xC0CFF23C, 0x647FCF02, 0xCFD0A418, 0xF8C19DF7,
    0xAC0B6E9E, 0x24EFD5BD, 0xA9F60028, 0x65571AD2, 0x61A0035B, 0x10FECDDB,
    0x88504DF9, 0xC2E3B94D};
*/
/*
const u32 bit_stream6[56] = {
    0x702428DF, 0x7AC1AEE9, 0xE24EFD5B, 0xEE95E000, 0x2B4FF3C1, 0x5C4800B0,
    0x4E12AD4F, 0xA2828DBE, 0xDD999527, 0x2A00ABED, 0x8DD84990, 0x6E9E24EF,
    0xD5BEA7D7, 0x49AD8B5C, 0xE74EDD9F, 0xA8400850, 0x8682920E, 0x1C8A1F86,
    0x755C4202, 0xAF6D1680, 0x1606E9E2, 0x4EFD5BE6, 0x5377EAAA, 0xA9AAAAAA,
    0x86AAAAAA, 0x1AAAAAA8, 0x6AAAAAA1, 0xAAAAAA86, 0xAAAAAA1A, 0xAAAAA86E,
    0x9E24EFD5, 0xBE2CCE24, 0xDFFF818E, 0x87510D33, 0xC5ACF122, 0xE05AFE12,
    0x385DDDF7, 0xAC401FFD, 0x540AB898, 0x6EE9E24E, 0xFD5BDEAE, 0x777E8558,
    0x171CAB67, 0x64D3DA4F, 0xA93FD931, 0x474FCF0B, 0xCB50CFE7, 0x02428DF7,
    0xAC1AEE9E, 0x24EFD5BD, 0xA9F60002, 0xB4FF3C15, 0xC4800B04, 0xE12AD4FA,
    0x2828DBED, 0xD9995272};
*/
/*
 const u32 bit_stream7[56] = {
    0x990B8120, 0x853EF6E9, 0xE24EFD5B, 0xEE95E000, 0x70FB6E8A, 0x897E0007,
    0xB01FC68E, 0xA17C912D, 0x4146382B, 0x0200B163, 0x9DE08BA2, 0x6E9E24EF,
    0xD5BEA7D7, 0x49AD8B5C, 0xE74EDD9F, 0xA8400850, 0x8682920E, 0x1C8A1F86,
    0x755C4202, 0xAF6D1680, 0x1606E9E2, 0x4EFD5BE6, 0x5377EAAA, 0xA9AAAAAA,
    0x86AAAAAA, 0x1AAAAAA8, 0x6AAAAAA1, 0xAAAAAA86, 0xAAAAAA1A, 0xAAAAA86E,
    0x9E24EFD5, 0xBE2CCE24, 0xDFFF818E, 0x87510D33, 0xC5ACF122, 0xE05AFE13,
    0xFD1DE5F7, 0xA8401FFC, 0x5F5DD4D4, 0xC6E9E24E, 0xFD5BDEAE, 0x77801990,
    0x1A9B77A7, 0xCE057118, 0x04D7FD67, 0xBA14BAF7, 0x57AF6019, 0x90B81208,
    0x53EF6E9E, 0x24EFD5BD, 0xA9F60007, 0x0FB6E8A8, 0x97E0007B, 0x01FC68EA,
    0x17C912D4, 0x146382B0};
*/

/* Utility function to get single bit from the bit_stream.
 * Input index is zero-based index.*/
u8 get_bit(const u32 *stream, u16 ind) {
  u8 bit = -1;
  if (ind >= BITSTREAM_LENGTH_BITS) {
    return bit;
  }

  u8 word_index = ind / 32;
  u8 bit_index = ind - word_index * 32;

  u32 word = stream[word_index];

  bit = (word >> (31 - bit_index)) & 0x1;

  return bit;
}

TEST(nav_msg_tests, init) {
  nav_msg_t n;
  nav_msg_init(&n);

  /* Check that every variable has been initialized to expected values. */
  EXPECT_EQ(n.subframe_bit_index, 0x0);
  EXPECT_FALSE(n.overrun);
  EXPECT_EQ(n.subframe_start_index, 0x0);
  EXPECT_EQ(n.next_subframe_id, 0x0);
  EXPECT_EQ(n.bit_polarity, BIT_POLARITY_UNKNOWN);
  EXPECT_EQ(n.alert, 0x0);
  for (u8 i = 0; i < NAV_MSG_SUBFRAME_WORDS_LEN; ++i) {
    EXPECT_EQ(n.subframe_bits[i], 0x0);
  }
  for (u8 i = 0; i < GPS_LNAV_SUBFRAME_CNT; ++i) {
    EXPECT_EQ(n.frame_age[i], GPS_LNAV_SUBFRAME_AGE_INVALID);
    for (u8 j = 0; j < 8; ++j) {
      EXPECT_EQ(n.frame_words[i][j], 0x0);
    }
  }
}

TEST(nav_msg_tests, subframe_ready) {
  nav_msg_t n;
  nav_msg_init(&n);
  bool ready = false;
  /* Subframe should not be ready after initialization. */
  ready = subframe_ready(&n);
  EXPECT_FALSE(ready);

  n.subframe_start_index = 1;
  /* Subframe should be ready with positive index. */
  ready = subframe_ready(&n);
  EXPECT_TRUE(ready);

  n.subframe_start_index = -1;
  /* Subframe should be ready with negative index. */
  ready = subframe_ready(&n);
  EXPECT_TRUE(ready);
}

TEST(nav_msg_tests, adjust_tow_trunc) {
  u32 TOW_trunc_s = 0;
  s32 TOW_ms = TOW_INVALID;

  /* Test all valid TOW_trunc values. */
  for (u32 i = 0; i < WEEK_SECS / GPS_TOW_MULTIPLIER; i++) {
    TOW_ms = adjust_tow(TOW_trunc_s + i);
    EXPECT_TRUE(TOW_ms >= 0 && TOW_ms < WEEK_MS);
  }

  /* Test invalid TOW_trunc value. */
  TOW_trunc_s = WEEK_SECS / GPS_TOW_MULTIPLIER;
  TOW_ms = adjust_tow(TOW_trunc_s);
  EXPECT_EQ(TOW_ms, TOW_INVALID);
}

TEST(nav_msg_tests, nav_msg_update_error_codes) {
  nav_msg_t n;
  nav_msg_init(&n);

  /* Select input stream to read. */
  const u32 *normal_stream = &normal_bit_stream[0];
  const u32 *inverse_stream = &inverse_bit_stream[0];

  /* Check that whole input streams are read without problems. */
  u8 bit = -1;
  u16 start_index = 0;
  u16 number_bits = BITSTREAM_LENGTH_BITS;
  u16 end_index = start_index + number_bits;
  for (u16 i = start_index; i < end_index; i++) {
    bit = get_bit(normal_stream, i);
    EXPECT_TRUE(bit == 1 || bit == 0);
    bit = get_bit(inverse_stream, i);
    EXPECT_TRUE(bit == 1 || bit == 0);
  }

  /* Check that buffer overrun is marked correctly.
   * Buffer gets overrun if:
   * 1) preamble is found,
   * 2) additional bits are read to the buffer,
   * 3) subframe is not processed before it gets overwritten. */
  s32 TOW = TOW_INVALID;
  for (u16 i = start_index; i < end_index; i++) {
    bit = get_bit(normal_stream, i);
    EXPECT_TRUE(bit == 1 || bit == 0);
    TOW = nav_msg_update(&n, (bool)bit);
    if (BUFFER_OVERRUN == TOW) {
      EXPECT_TRUE(n.overrun);
      break;
    }
  }

  /* Check that bit index does not go wild. */
  TOW = TOW_INVALID;
  for (u16 i = start_index; i < end_index; i++) {
    bit = get_bit(normal_stream, i);
    EXPECT_TRUE(bit == 1 || bit == 0);
    TOW = nav_msg_update(&n, (bool)bit);
    EXPECT_NE(BIT_INDEX_INVALID, TOW);
  }
}

TEST(nav_msg_tests, nav_msg_update_preamble_seek) {
  nav_msg_t n1;
  nav_msg_t n2;
  nav_msg_init(&n1);
  nav_msg_init(&n2);

  /* Select input stream to read. */
  const u32 *normal_stream = &normal_bit_stream[0];
  const u32 *inverse_stream = &inverse_bit_stream[0];

  /* Check that preamble is found in both streams with correct polarity. */
  u8 bit = -1;
  u16 start_index = 0;
  u16 number_bits = BITSTREAM_LENGTH_BITS;
  u16 end_index = start_index + number_bits;

  s32 TOW1 = TOW_INVALID;
  for (u16 i = start_index; i < end_index; i++) {
    bit = get_bit(normal_stream, i);
    EXPECT_TRUE(bit == 1 || bit == 0);
    TOW1 = nav_msg_update(&n1, (bool)bit);
    if (n1.subframe_start_index != 0) {
      break;
    }
  }
  /* Normal subframe should be found, i.e. subframe_start_index > 0. */
  EXPECT_TRUE(n1.subframe_start_index > 0 && TOW1 > 0 &&
              BIT_POLARITY_NORMAL == n1.bit_polarity);

  s32 TOW2 = TOW_INVALID;
  for (u16 i = start_index; i < end_index; i++) {
    bit = get_bit(inverse_stream, i);
    EXPECT_TRUE(bit == 1 || bit == 0);
    TOW2 = nav_msg_update(&n2, (bool)bit);
    if (n2.subframe_start_index != 0) {
      break;
    }
  }
  /* Inverted subframe should be found, i.e. subframe_start_index < 0.
   * TOW should have valid value. */
  EXPECT_TRUE(n2.subframe_start_index < 0 && TOW2 > 0 &&
              BIT_POLARITY_INVERTED == n2.bit_polarity);
}

TEST(nav_msg_tests, bit_polarity_seek) {
  nav_msg_t n1;
  nav_msg_t n2;
  nav_msg_init(&n1);
  nav_msg_init(&n2);

  /* Select input stream to read. */
  const u32 *normal_stream = &normal_bit_stream[0];
  const u32 *inverse_stream = &inverse_bit_stream[0];

  /* Check that TOW is not updated with bit_polarity_seek function.
   * Preamble starts at index 55,
   * and should be found when bit 114 (55 + 59) is processed.
   * We also need the 2 last parity bits from previous Word 10,
   * i.e. bits from indexes 53 & 54. */

  /* Let's loop through all possible starting bit indexes.
   * We should find the preamble when we start processing
   * from bit_index 0 to 53. */

  u8 bit = -1;
  s32 TOW1 = TOW_INVALID;
  u16 index_test_start = 0;
  /* This guarantees that we have parity bits decoded. */
  u16 index_test_end = FIRST_PREAMBLE_BIT_INDEX - 2;
  for (u16 i = index_test_start; i <= index_test_end; i++) {
    nav_msg_init(&n1);
    for (u16 j = 0; j <= FIRST_PREAMBLE_BIT_INDEX + 59; j++) {
      bit = get_bit(normal_stream, i + j);
      TOW1 = nav_msg_update(&n1, (bool)bit);
      if (TOW_INVALID != TOW1) {
        break;
      }
    }
    EXPECT_EQ(TOW_INVALID, TOW1);
  }

  s32 TOW2 = TOW_INVALID;
  index_test_start = 0;
  /* This guarantees that we have parity bits decoded. */
  index_test_end = FIRST_PREAMBLE_BIT_INDEX - 2;
  for (u16 i = index_test_start; i <= index_test_end; i++) {
    nav_msg_init(&n2);
    for (u16 j = 0; j <= FIRST_PREAMBLE_BIT_INDEX + 59; j++) {
      bit = get_bit(inverse_stream, i + j);
      TOW2 = nav_msg_update(&n2, (bool)bit);
      if (TOW_INVALID != TOW2) {
        break;
      }
    }
    EXPECT_EQ(TOW_INVALID, TOW2);
  }

  /* Verify that preamble is not found if we start reading stream
   * from point where there is only 1 last parity bit from Word 10,
   * followed by the preamble .*/
  u16 i = FIRST_PREAMBLE_BIT_INDEX - 1;
  nav_msg_init(&n1);
  for (u16 j = 0; j <= FIRST_PREAMBLE_BIT_INDEX + 59; j++) {
    bit = get_bit(normal_stream, i + j);
    TOW1 = nav_msg_update(&n1, (bool)bit);
    if (TOW_INVALID != TOW1) {
      break;
    }
  }
  EXPECT_EQ(TOW_INVALID, TOW1);

  nav_msg_init(&n2);
  for (u16 j = 0; j <= FIRST_PREAMBLE_BIT_INDEX + 59; j++) {
    bit = get_bit(inverse_stream, i + j);
    TOW2 = nav_msg_update(&n2, (bool)bit);
    if (TOW_INVALID != TOW2) {
      break;
    }
  }
  EXPECT_EQ(TOW_INVALID, TOW2);
}

TEST(nav_msg_tests, subframe_seek) {
  nav_msg_t n1;
  nav_msg_t n2;
  nav_msg_init(&n1);
  nav_msg_init(&n2);

  /* Select input stream to read. */
  const u32 *normal_stream = &normal_bit_stream[0];
  const u32 *inverse_stream = &inverse_bit_stream[0];

  /* Check that preamble is found with subframe_seek function.
   * First preamble starts at index 55, and the second preamble at index 355.
   * Subframe should be found when bit 414 (355 + 59) is processed.
   * We also need the 2 last parity bits from first Word 10,
   * i.e. bits from indexes 53 & 54. */

  /* Let's loop through all possible starting bit indexes.
   * We should find the subframe when we start processing
   * from bit_index 0 to 53. */

  u8 bit = -1;
  s32 TOW1 = TOW_INVALID;
  u16 index_test_start = 0;
  /* This guarantees that we have parity bits decoded. */
  u16 index_test_end = FIRST_PREAMBLE_BIT_INDEX - 2;
  for (u16 i = index_test_start; i <= index_test_end; i++) {
    nav_msg_init(&n1);
    for (u16 j = 0; j <= SECOND_PREAMBLE_BIT_INDEX + 59; j++) {
      bit = get_bit(normal_stream, i + j);
      TOW1 = nav_msg_update(&n1, (bool)bit);
      if (0 != n1.subframe_start_index) {
        break;
      }
    }
    EXPECT_NE(0, n1.subframe_start_index);
  }
  (void)TOW1;

  s32 TOW2 = TOW_INVALID;
  index_test_start = 0;
  /* This guarantees that we have parity bits decoded. */
  index_test_end = FIRST_PREAMBLE_BIT_INDEX - 2;
  for (u16 i = index_test_start; i <= index_test_end; i++) {
    nav_msg_init(&n2);
    for (u16 j = 0; j <= SECOND_PREAMBLE_BIT_INDEX + 59; j++) {
      bit = get_bit(inverse_stream, i + j);
      TOW2 = nav_msg_update(&n2, (bool)bit);
      if (0 != n2.subframe_start_index) {
        break;
      }
    }
    EXPECT_NE(0, n2.subframe_start_index);
  }
  (void)TOW2;

  /* Verify that subframe is not found if we start reading stream
   * from point where there is only 1 last parity bit from Word 10,
   * followed by the preamble .*/
  nav_msg_init(&n1);
  for (u16 i = FIRST_PREAMBLE_BIT_INDEX - 1;
       i <= SECOND_PREAMBLE_BIT_INDEX + 59;
       i++) {
    bit = get_bit(normal_stream, i);
    TOW1 = nav_msg_update(&n1, (bool)bit);
    if (0 != n1.subframe_start_index) {
      break;
    }
  }
  EXPECT_EQ(0, n1.subframe_start_index);

  nav_msg_init(&n2);
  for (u16 i = FIRST_PREAMBLE_BIT_INDEX - 1;
       i <= SECOND_PREAMBLE_BIT_INDEX + 59;
       i++) {
    bit = get_bit(inverse_stream, i);
    TOW2 = nav_msg_update(&n2, (bool)bit);
    if (0 != n2.subframe_start_index) {
      break;
    }
  }
  EXPECT_EQ(0, n2.subframe_start_index);
}

TEST(nav_msg_tests, extract_word) {
  nav_msg_t n1;
  nav_msg_t n2;
  nav_msg_init(&n1);
  nav_msg_init(&n2);

  /* Select input stream to read. */
  const u32 *normal_stream = &normal_bit_stream[0];
  const u32 *inverse_stream = &inverse_bit_stream[0];

  /* Check that preamble is correctly extracted.
   * First preamble starts at index 55, and the second preamble at index 355.
   * Subframe should be found when bit 414 (355 + 59) is processed.
   * We also need the 2 last parity bits from first Word 10,
   * i.e. bits from indexes 53 & 54. */

  /* Let's start processing bits from bit_index 53 to 414.
   * We should have all necessary data in buffer then. */
  u8 bit = -1;
  s32 TOW1 = TOW_INVALID;
  for (u16 i = FIRST_PREAMBLE_BIT_INDEX - 2;
       i <= SECOND_PREAMBLE_BIT_INDEX + 59;
       i++) {
    bit = get_bit(normal_stream, i);
    TOW1 = nav_msg_update(&n1, (bool)bit);
    if (0 != n1.subframe_start_index) {
      break;
    }
  }
  (void)TOW1;
  EXPECT_GT(n1.subframe_start_index, 0);

  /* Let's extract both preambles. */
  u32 first_preamble = extract_word(&n1, 0, 8, 0);
  u32 second_preamble = extract_word(&n1, 300, 8, 0);

  /* This was normal bit stream. */
  EXPECT_TRUE(first_preamble == second_preamble &&
              GPS_L1CA_PREAMBLE_NORMAL == first_preamble);

  /* Let's extract both TOWs. */
  u8 parity_bit1 = extract_word(&n1, 29, 1, 0);
  u8 parity_bit2 = extract_word(&n1, 329, 1, 0);

  u32 TOW_trunc1 = extract_word(&n1, 30, 17, parity_bit1);
  EXPECT_LT(TOW_trunc1 * GPS_TOW_MULTIPLIER, WEEK_SECS);

  u32 TOW_trunc2 = extract_word(&n1, 330, 17, parity_bit2);
  EXPECT_LT(TOW_trunc2 * GPS_TOW_MULTIPLIER, WEEK_SECS);

  /* Check that incremented TOW1 matches with next TOW2. */
  TOW_trunc1++;
  /* Handle end of week roll over. */
  if (TOW_trunc1 * GPS_TOW_MULTIPLIER == WEEK_SECS) {
    TOW_trunc1 = 0;
  }

  EXPECT_EQ(TOW_trunc1, TOW_trunc2);

  /* Repeat the test for inverse bit stream. */
  s32 TOW2 = TOW_INVALID;
  for (u16 i = FIRST_PREAMBLE_BIT_INDEX - 2;
       i <= SECOND_PREAMBLE_BIT_INDEX + 59;
       i++) {
    bit = get_bit(inverse_stream, i);
    TOW2 = nav_msg_update(&n2, (bool)bit);
    if (0 != n2.subframe_start_index) {
      break;
    }
  }
  EXPECT_LT(n2.subframe_start_index, 0);
  (void)TOW2;

  /* Let's extract both preambles. */
  first_preamble = extract_word(&n2, 0, 8, 0);
  second_preamble = extract_word(&n2, 300, 8, 0);

  /* This was inverted bit stream,
   * but extract word inverts the preamble to normal polarity. */
  EXPECT_TRUE(first_preamble == second_preamble &&
              GPS_L1CA_PREAMBLE_NORMAL == first_preamble);

  /* Let's extract both TOWs. */
  parity_bit1 = extract_word(&n2, 29, 1, 0);
  parity_bit2 = extract_word(&n2, 329, 1, 0);

  TOW_trunc1 = extract_word(&n2, 30, 17, parity_bit1);
  EXPECT_LT(TOW_trunc1 * GPS_TOW_MULTIPLIER, WEEK_SECS);

  TOW_trunc2 = extract_word(&n2, 330, 17, parity_bit2);
  EXPECT_LT(TOW_trunc2 * GPS_TOW_MULTIPLIER, WEEK_SECS);

  /* Check that incremented TOW1 matches with next TOW2. */
  TOW_trunc1++;
  /* Handle end of week roll over. */
  if (TOW_trunc1 * GPS_TOW_MULTIPLIER == WEEK_SECS) {
    TOW_trunc1 = 0;
  }

  EXPECT_EQ(TOW_trunc1, TOW_trunc2);
}

TEST(nav_msg_tests, nav_parity) {
  nav_msg_t n1;
  nav_msg_t n2;
  nav_msg_init(&n1);
  nav_msg_init(&n2);

  /* Select input stream to read. */
  const u32 *normal_stream = &normal_bit_stream[0];
  const u32 *inverse_stream = &inverse_bit_stream[0];

  /* Check that parity algorithm works.
   * First preamble starts at index 55, and the second preamble at index 355.
   * Subframe should be found when bit 414 (355 + 59) is processed.
   * We also need the 2 last parity bits from first Word 10,
   * i.e. bits from indexes 53 & 54. */

  /* Let's start processing bits from bit_index 53 to 414.
   * We should have all necessary data in buffer then. */
  u8 bit = -1;
  s32 TOW1 = TOW_INVALID;
  for (u16 i = FIRST_PREAMBLE_BIT_INDEX - 2;
       i <= SECOND_PREAMBLE_BIT_INDEX + 59;
       i++) {
    bit = get_bit(normal_stream, i);
    TOW1 = nav_msg_update(&n1, (bool)bit);
    if (0 != n1.subframe_start_index) {
      break;
    }
  }
  EXPECT_GT(n1.subframe_start_index, 0);
  (void)TOW1;

  /* Let's 2 parity bits + first HOW word. */
  u32 how = extract_word(&n1, 28, 32, 0);
  u8 parity = nav_parity(&how);
  EXPECT_EQ(0, parity);
  /* Let's test bit error case. */
  /* Flip previous parity bits. */
  u32 bit_error_how = how ^ (0x3 << 30);
  parity = nav_parity(&bit_error_how);
  EXPECT_NE(0, parity);

  /* Repeat the test for inverse bit stream. */
  s32 TOW2 = TOW_INVALID;
  for (u16 i = FIRST_PREAMBLE_BIT_INDEX - 2;
       i <= SECOND_PREAMBLE_BIT_INDEX + 59;
       i++) {
    bit = get_bit(inverse_stream, i);
    TOW2 = nav_msg_update(&n2, (bool)bit);
    if (0 != n2.subframe_start_index) {
      break;
    }
  }
  EXPECT_LT(n2.subframe_start_index, 0);
  (void)TOW2;

  /* Let's 2 parity bits + first HOW word. */
  how = extract_word(&n2, 28, 32, 0);
  parity = nav_parity(&how);
  EXPECT_EQ(0, parity);

  /* Let's test bit error case. */
  /* Flip previous parity bits. */
  bit_error_how = how ^ (0x3 << 30);
  parity = nav_parity(&bit_error_how);
  EXPECT_NE(0, parity);
}
