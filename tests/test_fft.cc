#include "gtest/gtest.h"
#include "lib/fixed_fft_r2.h"

TEST(fft_tests, init) {
  intFFTr2_t fft_conf;
  InitIntFFTr2(&fft_conf, 10);
  EXPECT_EQ(fft_conf.N, 10);
}
