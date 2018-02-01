#include "gtest/gtest.h"
#include "lib/fixed_fft_r2.h"

#define FFT_SIZE 16

TEST(fft_tests, init) {
  FFT_DECL(FFT_SIZE, fft_conf);
  InitIntFFTr2(&fft_conf, FFT_SIZE);
  EXPECT_EQ(fft_conf.N, FFT_SIZE);
}
