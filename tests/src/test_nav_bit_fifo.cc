#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <iostream>

#include "gtest/gtest.h"

#include "nav_bit_fifo/nav_bit_fifo.h"

TEST(nav_bit_fifo_tests, delay) {
  nav_bit_t bit[] = {0, 1, 2};
  size_t bwi = 0;
  size_t bri = 0;
  nav_bit_fifo_t fifo;
  nav_bit_fifo_init(&fifo, CODE_SBAS_L1CA);

  for (size_t delay_symbols = 0; delay_symbols < 65; delay_symbols++) {
    for (size_t j = 0; j < delay_symbols; j++) {
      if (nav_bit_fifo_write(&fifo, &bit[bwi])) {
        bwi = (bwi + 1) % ARRAY_SIZE(bit);
      } else {
        std::cerr << " init failed at " << j << " out of " << delay_symbols
                  << std::endl;
      }
    }

    nav_bit_t el;
    for (size_t b = 0; b < 257; b++) {
      if (nav_bit_fifo_write(&fifo, &bit[bwi])) {
        bwi = (bwi + 1) % ARRAY_SIZE(bit);
      } else {
        std::cerr << " write failed at " << b << " out of " << 257 << std::endl;
      }
      if (nav_bit_fifo_read(&fifo, &el)) {
        EXPECT_EQ(el, bit[bri]) << " for delay_symbols " << delay_symbols;
        bri = (bri + 1) % ARRAY_SIZE(bit);
      } else {
        std::cerr << " read failed at " << b << " out of " << 257 << std::endl;
      }
    }

    for (size_t j = 0; j < delay_symbols; j++) {
      if (nav_bit_fifo_read(&fifo, &el)) {
        EXPECT_EQ(el, bit[bri]) << " for delay_symbols " << delay_symbols;
        bri = (bri + 1) % ARRAY_SIZE(bit);
      } else {
        std::cerr << " read failed at " << j << " out of " << delay_symbols
                  << std::endl;
      }
    }
  }
}
