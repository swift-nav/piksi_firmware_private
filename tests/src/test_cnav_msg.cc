#include "gtest/gtest.h"
#include "nav_msg/cnav_msg.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define G1 0x79 /* convolution encoder coefficient A 0171 */
#define G2 0x5B /* convolution encoder coefficient B 0133 */
#define SUFFIX_SIZE                                       \
  16 /* Data to feed after the last message to ensure the \
      * decoder completes message. */

static inline size_t encode_byte(u32 *pacc, u8 v, u8 *dst) {
  u32 acc = *pacc;
  for (int n_bit = CHAR_BIT - 1; n_bit >= 0; --n_bit) {
    u32 t = ((v >> n_bit) & 1) << 6;
    acc = (acc >> 1) | t;
    *dst++ = parity(acc & G1) ? 0xFF : 00;
    *dst++ = parity(acc & G2) ? 0xFF : 00;
  }
  *pacc = acc;
  return CHAR_BIT * 2;
}

TEST(cnav_tests, test_crc) {
  cnav_v27_part_t part;
  memset(&part, 0, sizeof(part));
  part.n_decoded = GPS_CNAV_MSG_LENGTH;
  u32 crc = _cnav_compute_crc(&part);
  EXPECT_EQ(crc, 0);

  part.decoded[0] = 1;
  crc = _cnav_compute_crc(&part);
  EXPECT_EQ(crc, 0x733518);
}

static inline size_t encode(u32 *pacc, const u8 *src, size_t bytes, u8 *dst) {
  size_t cnt = 0;

  for (size_t i = 0; i < bytes; ++i) {
    cnt += encode_byte(pacc, src[i], dst + cnt);
  }

  return cnt;
}
static inline size_t encode_bits(u32 *pacc,
                                 const u8 *src,
                                 size_t bits,
                                 u8 *dst) {
  size_t cnt = 0;
  size_t n_bytes = bits / CHAR_BIT;

  for (size_t i = 0; i < n_bytes; ++i) {
    cnt += encode_byte(pacc, src[i], dst + cnt);
  }
  if (bits - n_bytes * CHAR_BIT != 0) {
    size_t x_bits = bits % CHAR_BIT;
    u8 v = src[n_bytes];

    u32 acc = *pacc;
    for (size_t i = 0; i < x_bits; ++i) {
      u32 t = ((v >> (CHAR_BIT - 1 - i)) & 1) << 6;
      acc = (acc >> 1) | t;
      dst[cnt++] = parity(acc & G1) ? 0xFF : 00;
      dst[cnt++] = parity(acc & G2) ? 0xFF : 00;
    }
    *pacc = acc;
  }

  return cnt;
}

TEST(cnav_tests, cnav_extract_crc) {
  cnav_v27_part_t part;
  memset(&part, 0, sizeof(part));
  part.n_decoded = GPS_CNAV_MSG_LENGTH;

  u32 crc = _cnav_extract_crc(&part);
  EXPECT_EQ(0, crc);

  part.decoded[34] = 0xDE;
  part.decoded[35] = 0xAD;
  part.decoded[36] = 0xBE;
  part.decoded[37] = 0xEF;
  crc = _cnav_extract_crc(&part);
  EXPECT_EQ(0xEADBEE, crc);
}

TEST(cnav_tests, cnav_rescan_preamble) {
  cnav_v27_part_t part;
  memset(&part, 0, sizeof(part));
  part.n_decoded = GPS_CNAV_MSG_LENGTH;

  _cnav_rescan_preamble(&part);
  EXPECT_EQ(7, part.n_decoded);
  EXPECT_EQ(0, part.decoded[0]);

  part.n_decoded = GPS_CNAV_MSG_LENGTH;
  part.decoded[36] = 0x07;
  part.decoded[37] = 0xF0;
  _cnav_rescan_preamble(&part);
  EXPECT_EQ(7, part.n_decoded);
  EXPECT_EQ(0xFE, part.decoded[0]);

  part.n_decoded = GPS_CNAV_MSG_LENGTH;
  part.decoded[10] = GPS_CNAV_PREAMBLE1;
  part.decoded[36] = 0x07;
  part.decoded[37] = 0xF0;
  _cnav_rescan_preamble(&part);
  EXPECT_EQ(220, part.n_decoded);
  EXPECT_EQ(GPS_CNAV_PREAMBLE1, part.decoded[0]);
  EXPECT_FALSE(part.invert);

  part.n_decoded = GPS_CNAV_MSG_LENGTH;
  part.decoded[10] = GPS_CNAV_PREAMBLE2;
  part.decoded[36] = 0x07;
  part.decoded[37] = 0xF0;
  _cnav_rescan_preamble(&part);
  EXPECT_EQ(220, part.n_decoded);
  EXPECT_EQ(GPS_CNAV_PREAMBLE2, part.decoded[0]);
  EXPECT_TRUE(part.invert);
}

TEST(cnav_tests, cnav_decoder_init) {
  cnav_msg_decoder_t dec;
  cnav_msg_decoder_init(&dec);

  EXPECT_FALSE(dec.part1.invert);
  EXPECT_FALSE(dec.part1.message_lock);
  EXPECT_FALSE(dec.part1.preamble_seen);
  EXPECT_EQ(0, dec.part1.n_symbols);
  EXPECT_EQ(0, dec.part1.n_decoded);

  EXPECT_FALSE(dec.part2.invert);
  EXPECT_FALSE(dec.part2.message_lock);
  EXPECT_FALSE(dec.part2.preamble_seen);
  EXPECT_EQ(1, dec.part2.n_symbols);
  EXPECT_EQ(0, dec.part2.n_decoded);
}

static cnav_msg_t basic = {22, 0, 1000, 0, 0, .data = {{0, 0, 0, 0, 0, 0}}};

static size_t add_encoded_message(u32 *pacc, u8 *buf, cnav_msg_t *msg) {
  u8 src_msg[38] = {0};
  memset(src_msg, 0x55, sizeof(src_msg));
  setbitu(src_msg, 0, 8, GPS_CNAV_PREAMBLE1); /* preamble */
  setbitu(src_msg, 8, 6, msg->prn);           /* prn */
  setbitu(src_msg, 14, 6, msg->msg_id);       /* msg */
  setbitu(src_msg, 20, 17, msg->tow);         /* tow */
  setbitu(src_msg, 37, 1, msg->alert);        /* flag */

  switch (msg->msg_id) {
    case CNAV_MSG_TYPE_30:
      setbitu(src_msg, 127, 13, msg->data.type_30.tgd);      /* tgd */
      setbitu(src_msg, 140, 13, msg->data.type_30.isc_l1ca); /* isc_l1ca */
      setbitu(src_msg, 153, 13, msg->data.type_30.isc_l2c);  /* isc_l2c */
      break;

    default:
      break;
  }

  u32 crc = crc24q_bits(0, src_msg, GPS_CNAV_MSG_DATA_LENGTH, false);
  setbitu(src_msg, GPS_CNAV_MSG_DATA_LENGTH, GPS_CNAV_MSG_CRC_LENGTH, crc);

  return encode_bits(pacc, src_msg, GPS_CNAV_MSG_LENGTH, buf);
}

static size_t encode_buffer_p(u8 *buf,
                              const u8 *prefix,
                              size_t prefix_size,
                              cnav_msg_t *msg) {
  u8 suffix[SUFFIX_SIZE] = {0};
  memset(suffix, 0x55, sizeof(suffix));

  size_t dst = 0;
  u32 acc = 0;
  dst += encode(&acc, prefix, prefix_size, buf + dst);
  dst += add_encoded_message(&acc, buf + dst, msg);
  dst += encode(&acc, suffix, sizeof(suffix), buf + dst);

  return dst;
}

static size_t encode_buffer(u8 *buf, cnav_msg_t *msg) {
  u8 prefix[4] = {0};
  memset(prefix, 0x55, sizeof(prefix));

  return encode_buffer_p(buf, prefix, sizeof(prefix), msg);
}

TEST(cnav_tests, cnav_decode_isc) {
  cnav_msg_decoder_t dec;
  cnav_msg_decoder_init(&dec);
  cnav_msg_t to_encode;
  to_encode.prn = 7;
  to_encode.msg_id = CNAV_MSG_TYPE_30;
  to_encode.tow = 123456;
  to_encode.alert = 0;
  to_encode.data.type_30.tgd_valid = true;
  to_encode.data.type_30.tgd = 123;
  to_encode.data.type_30.isc_l1ca_valid = true;
  to_encode.data.type_30.isc_l1ca = -45;
  to_encode.data.type_30.isc_l2c_valid = false;
  to_encode.data.type_30.isc_l2c = INVALID_GROUP_DELAY_VALUE;
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 enc[((4 + SUFFIX_SIZE) * CHAR_BIT + GPS_CNAV_MSG_LENGTH) * 2];
  size_t dst = encode_buffer(enc, &to_encode);

  EXPECT_EQ((4 + SUFFIX_SIZE) * CHAR_BIT * 2 + GPS_CNAV_MSG_LENGTH * 2, dst);

  bool ret = false;
  for (i = 0; i < dst; ++i) {
    u32 delay = 0;

    ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      EXPECT_EQ(msg.prn, to_encode.prn);
      EXPECT_EQ(msg.msg_id, to_encode.msg_id);
      EXPECT_EQ(msg.tow, to_encode.tow);
      EXPECT_EQ(msg.alert, to_encode.alert);

      EXPECT_EQ(msg.data.type_30.tgd, to_encode.data.type_30.tgd);
      EXPECT_EQ(msg.data.type_30.isc_l1ca, to_encode.data.type_30.isc_l1ca);
      EXPECT_EQ(msg.data.type_30.isc_l2c, to_encode.data.type_30.isc_l2c);

      break;
    }
  }

  EXPECT_FALSE(!ret);
}

TEST(cnav_tests, cnav_decode1) {
  cnav_msg_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  cnav_msg_decoder_init(&dec);
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 enc[((4 + SUFFIX_SIZE) * CHAR_BIT + GPS_CNAV_MSG_LENGTH) * 2];
  size_t dst = encode_buffer(enc, &basic);

  EXPECT_EQ((4 + SUFFIX_SIZE) * CHAR_BIT * 2 + GPS_CNAV_MSG_LENGTH * 2, dst);

  bool ret = false;
  for (i = 0; i < dst; ++i) {
    u32 delay = 0;

    ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      EXPECT_EQ(msg.prn, 22);
      EXPECT_EQ(msg.msg_id, 0);
      EXPECT_EQ(msg.tow, 1000);
      EXPECT_EQ(msg.alert, false);

      break;
    }
  }

  EXPECT_FALSE(!ret);
}

TEST(cnav_tests, cnav_decode2) {
  cnav_msg_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  cnav_msg_decoder_init(&dec);
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 enc[((4 + SUFFIX_SIZE) * CHAR_BIT + GPS_CNAV_MSG_LENGTH) * 2 + 1];
  size_t dst = encode_buffer(enc + 1, &basic);

  EXPECT_EQ((4 + SUFFIX_SIZE) * CHAR_BIT * 2 + GPS_CNAV_MSG_LENGTH * 2, dst);

  bool ret = false;
  for (i = 0; i < dst; ++i) {
    u32 delay = 0;

    ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      u32 expected_delay;
      EXPECT_EQ(msg.prn, 22);
      EXPECT_EQ(msg.msg_id, 0);
      EXPECT_EQ(msg.tow, 1000);
      EXPECT_EQ(msg.alert, false);
      expected_delay = i + 1; /* symbols fed into the decoder */
      expected_delay -= 600;  /* symbols in L2C message */
      expected_delay -= 64;   /* size of dummy header outside
                                 of data message [symbols] */
      EXPECT_NE(expected_delay, delay);

      break;
    }
  }

  EXPECT_FALSE(!ret);
}

TEST(cnav_tests, cnav_decode_neg) {
  cnav_msg_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  cnav_msg_decoder_init(&dec);
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 enc[((4 + SUFFIX_SIZE) * CHAR_BIT + GPS_CNAV_MSG_LENGTH) * 2];
  size_t dst = encode_buffer(enc, &basic);

  EXPECT_EQ((4 + SUFFIX_SIZE) * CHAR_BIT * 2 + GPS_CNAV_MSG_LENGTH * 2, dst);

  for (i = 0; i < sizeof(enc); ++i) {
    enc[i] ^= 0xFFu;
  }

  bool ret = false;

  for (i = 0; i < dst; ++i) {
    u32 delay = 0;
    ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      EXPECT_EQ(msg.prn, 22);
      EXPECT_EQ(msg.msg_id, 0);
      EXPECT_EQ(msg.tow, 1000);
      EXPECT_EQ(msg.alert, false);

      break;
    }
  }

  EXPECT_FALSE(!ret);
}

TEST(cnav_tests, cnav_decode_false) {
  cnav_msg_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  cnav_msg_decoder_init(&dec);
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 prefix[] = {0x55, 0x55, 0x55, 0x55, 0x8B, 0x8B};
  u8 suffix[SUFFIX_SIZE] = {0};
  u8 enc[((sizeof(prefix) + sizeof(suffix)) * CHAR_BIT + GPS_CNAV_MSG_LENGTH) *
             2 +
         1];
  size_t dst = encode_buffer_p(enc, prefix, sizeof(prefix), &basic);

  EXPECT_EQ(
      ((sizeof(prefix) + sizeof(suffix)) * CHAR_BIT + GPS_CNAV_MSG_LENGTH) * 2,
      dst);

  u32 counter = 0;
  for (i = 0; i < dst; ++i) {
    u32 delay = 0;

    bool ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      counter++;

      EXPECT_EQ(msg.prn, 22);
      EXPECT_EQ(msg.msg_id, 0);
      EXPECT_EQ(msg.tow, 1000);
      EXPECT_EQ(msg.alert, false);
    }
  }

  EXPECT_NE(counter, 0);
}

TEST(cnav_tests, cnav_decode_unlock) {
  cnav_msg_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  cnav_msg_decoder_init(&dec);
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 prefix[4];
  u8 suffix[SUFFIX_SIZE];

  memset(prefix, 0x55, sizeof(prefix));
  memset(suffix, 0x55, sizeof(suffix));

  u8 enc[((sizeof(prefix) + sizeof(suffix)) * CHAR_BIT + /* Prefix + suffix */
          GPS_CNAV_MSG_LENGTH +                          /* Message CRC_OK */
          GPS_CNAV_MSG_LENGTH *
              (1 + GPS_CNAV_LOCK_MAX_CRC_FAILS) + /* Messages CRC bad */
          GPS_CNAV_MSG_LENGTH                     /* Message CRC_OK */
          ) * 2 +
         1];
  u8 bad_message[38] = {0};
  memset(bad_message, 0x55, sizeof(bad_message));

  size_t dst = 0;
  u32 acc = 0;

  dst += encode(&acc, prefix, sizeof(prefix), &enc[0] + dst);
  dst += add_encoded_message(&acc, &0 [enc] + dst, &basic);

  for (i = 0; i <= GPS_CNAV_LOCK_MAX_CRC_FAILS; ++i) {
    dst += encode_bits(&acc, bad_message, GPS_CNAV_MSG_LENGTH, &enc[0] + dst);
  }

  dst += add_encoded_message(&acc, &0 [enc] + dst, &basic);
  dst += encode(&acc, suffix, sizeof(suffix), &enc[0] + dst);

  EXPECT_EQ(((sizeof(prefix) + sizeof(suffix)) * CHAR_BIT +
             GPS_CNAV_MSG_LENGTH * (GPS_CNAV_LOCK_MAX_CRC_FAILS + 3)) *
                2,
            dst);

  u32 decoded = 0;
  for (i = 0; i < dst; ++i) {
    u32 delay = 0;

    bool ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      decoded++;

      EXPECT_EQ(msg.prn, 22);
      EXPECT_EQ(msg.msg_id, 0);
      EXPECT_EQ(msg.tow, 1000);
      EXPECT_EQ(msg.alert, false);
    }
  }

  EXPECT_EQ(decoded, 2);
}

TEST(cnav_tests, cnav_decode_ok_nok_ok) {
  cnav_msg_decoder_t dec;
  memset(&dec, 0, sizeof(dec));
  cnav_msg_decoder_init(&dec);
  cnav_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  size_t i;

  u8 prefix[4];
  u8 suffix[SUFFIX_SIZE];

  memset(prefix, 0x55, sizeof(prefix));
  memset(suffix, 0x55, sizeof(suffix));

  u8 enc[((sizeof(prefix) + sizeof(suffix)) * CHAR_BIT + /* Prefix + suffix */
          GPS_CNAV_MSG_LENGTH +                          /* Message CRC_OK */
          GPS_CNAV_MSG_LENGTH +                          /* Messages CRC bad
                                                          */
          GPS_CNAV_MSG_LENGTH                            /* Message CRC_OK */
          ) * 2 +
         1];
  u8 bad_message[38] = {0};
  memset(bad_message, 0x55, sizeof(bad_message));

  size_t dst = 0;
  u32 acc = 0;

  dst += encode(&acc, prefix, sizeof(prefix), &enc[0] + dst);
  dst += add_encoded_message(&acc, &0 [enc] + dst, &basic);

  dst += encode_bits(&acc, bad_message, GPS_CNAV_MSG_LENGTH, &enc[0] + dst);

  dst += add_encoded_message(&acc, &0 [enc] + dst, &basic);
  dst += encode(&acc, suffix, sizeof(suffix), &enc[0] + dst);

  EXPECT_EQ(
      ((sizeof(prefix) + sizeof(suffix)) * CHAR_BIT + GPS_CNAV_MSG_LENGTH * 3) *
          2,
      dst);

  u32 decoded = 0;
  for (i = 0; i < dst; ++i) {
    u32 delay = 0;

    bool ret = cnav_msg_decoder_add_symbol(&dec, enc[i], &msg, &delay);

    if (ret) {
      decoded++;

      EXPECT_EQ(msg.prn, 22);
      EXPECT_EQ(msg.msg_id, 0);
      EXPECT_EQ(msg.tow, 1000);
      EXPECT_EQ(msg.alert, false);
    }
  }

  EXPECT_EQ(decoded, 2);
}

TEST(cnav_tests, cnav_crc) {
  u32 crc0, crc1;
  u8 data0[35] = {0x0D, 0xEA, 0xDB, 0xEE, 0xF0};

  crc0 = crc24q(data0, 35, 0);
  bitshl(data0, 35, 4);
  crc1 = crc24q_bits(0, data0, 35 * 8 - 4, false);

  EXPECT_EQ(crc0, crc1);
}
