#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gtest/gtest.h"

#include "nav_bit_fifo/nav_bit_fifo.h"

#define DELAY_SYMBOLS 16
#define SYMBOL_MS 20

TEST(nav_bit_fifo_tests, delay) {
  nav_bit_t bit[] = {0, 1, 2};
  size_t bwi = 0;
  size_t bri = 0;
  u8 sz = (u8)(MAX_NAV_BIT_LATENCY_MS / SYMBOL_MS);
  nav_bit_fifo_t fifo;
  nav_bit_fifo_init(&fifo, sz);

  for (size_t j = 0; j < DELAY_SYMBOLS; j++) {
    nav_bit_fifo_write(&fifo, &bit[bwi]);
    bwi = (bwi + 1) % ARRAY_SIZE(bit);
  }

  nav_bit_t el;
  for (size_t b = 0; b < 257; b++) {
    nav_bit_fifo_write(&fifo, &bit[bwi]);
    bwi = (bwi + 1) % ARRAY_SIZE(bit);
    nav_bit_fifo_read(&fifo, &el);
    EXPECT_EQ(el, bit[bri]);
    bri = (bri + 1) % ARRAY_SIZE(bit);
  }

  for (size_t j = 0; j < DELAY_SYMBOLS; j++) {
    nav_bit_fifo_read(&fifo, &el);
    EXPECT_EQ(el, bit[bri]);
    bri = (bri + 1) % ARRAY_SIZE(bit);
  }
}
