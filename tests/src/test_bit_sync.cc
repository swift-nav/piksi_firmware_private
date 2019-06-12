#include <inttypes.h>
#include <stdio.h>

#include "bit_sync/bit_sync.h"
#include "gtest/gtest.h"

TEST(bit_sync_test, test_bit_sync_single_update_20ms) {
  bit_sync_t bit_sync_gps;
  me_gnss_signal_t mesid_gps = {
      .sat = 1,
      .code = CODE_GPS_L1CA,
  };

  bit_sync_init(&bit_sync_gps, mesid_gps);
  EXPECT_EQ(20, bit_sync_gps.bit_length);

  s32 corr_prompt_real = 100;
  s32 corr_prompt_imag = 0;
  u32 ms = 20;
  s32 bit_integrate = 0;

  bit_sync_update(
      &bit_sync_gps, corr_prompt_real, corr_prompt_imag, ms, &bit_integrate);
  EXPECT_EQ(0, bit_integrate);

  s8 bit_phase_ref = 7;
  bit_sync_set(&bit_sync_gps, bit_phase_ref);
  EXPECT_EQ(7, bit_sync_gps.bit_phase_ref);
}

TEST(bit_sync_test, test_bit_sync_single_update_10ms) {
  bit_sync_t bit_sync_gps;
  me_gnss_signal_t mesid_gps = {
      .sat = 32,
      .code = CODE_GPS_L1CA,
  };

  bit_sync_init(&bit_sync_gps, mesid_gps);
  EXPECT_EQ(20, bit_sync_gps.bit_length);

  s32 corr_prompt_real = 100;
  s32 corr_prompt_imag = 0;
  u32 ms = 10;
  s32 bit_integrate = 0;

  bit_sync_update(
      &bit_sync_gps, corr_prompt_real, corr_prompt_imag, ms, &bit_integrate);
  EXPECT_EQ(0, bit_integrate);

  s8 bit_phase_ref = 7;
  bit_sync_set(&bit_sync_gps, bit_phase_ref);
  EXPECT_EQ(7, bit_sync_gps.bit_phase_ref);

  bit_sync_t bit_sync_glo;
  me_gnss_signal_t mesid_glo = {
      .sat = 14,
      .code = CODE_GLO_L1OF,
  };

  bit_sync_init(&bit_sync_glo, mesid_glo);
  EXPECT_EQ(10, bit_sync_glo.bit_length);

  corr_prompt_real = 100;
  corr_prompt_imag = 0;
  ms = 10;
  bit_integrate = 0;

  bit_sync_update(
      &bit_sync_glo, corr_prompt_real, corr_prompt_imag, ms, &bit_integrate);
  EXPECT_EQ(0, bit_integrate);

  bit_phase_ref = 8;
  bit_sync_set(&bit_sync_glo, bit_phase_ref);
  EXPECT_EQ(8, bit_sync_glo.bit_phase_ref);
}

TEST(bit_sync_test, test_bit_sync_multi_update_10ms) {
  bit_sync_t bit_sync_gps;
  me_gnss_signal_t mesid_gps = {
      .sat = 32,
      .code = CODE_GPS_L1CA,
  };

  bit_sync_init(&bit_sync_gps, mesid_gps);
  EXPECT_EQ(20, bit_sync_gps.bit_length);

  s32 corr_prompt_real = 100;
  s32 corr_prompt_imag = 100;
  u32 ms = 1;
  s32 bit_integrate = 0;
  s8 sign = 1;
  s8 true_bit_phase = 10;

  for (int i = 0; i < 1000; i++) {
    if (i % 20 == true_bit_phase) {
      sign *= -1;
    }
    bit_sync_update(&bit_sync_gps,
                    sign * corr_prompt_real,
                    sign * corr_prompt_imag,
                    ms,
                    &bit_integrate);
  }

  EXPECT_EQ(true_bit_phase, bit_sync_gps.bit_phase_ref);

  s8 bit_phase_ref = 7;
  bit_sync_set(&bit_sync_gps, bit_phase_ref);
  EXPECT_EQ(7, bit_sync_gps.bit_phase_ref);

  bit_sync_t bit_sync_glo;
  me_gnss_signal_t mesid_glo = {
      .sat = 14,
      .code = CODE_GLO_L1OF,
  };

  bit_sync_init(&bit_sync_glo, mesid_glo);
  EXPECT_EQ(10, bit_sync_glo.bit_length);

  corr_prompt_real = 100;
  corr_prompt_imag = 100;
  ms = 1;
  bit_integrate = 0;
  sign = 1;
  true_bit_phase = 5;

  for (int i = 0; i < 1000; i++) {
    if (i % 10 == true_bit_phase) {
      sign *= -1;
    }
    bit_sync_update(&bit_sync_glo,
                    sign * corr_prompt_real,
                    sign * corr_prompt_imag,
                    ms,
                    &bit_integrate);
  }
  EXPECT_EQ(true_bit_phase, bit_sync_glo.bit_phase_ref);

  bit_phase_ref = 8;
  bit_sync_set(&bit_sync_glo, bit_phase_ref);
  EXPECT_EQ(8, bit_sync_glo.bit_phase_ref);
}
