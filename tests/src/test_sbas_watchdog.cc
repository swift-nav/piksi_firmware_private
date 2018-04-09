#include "gtest/gtest.h"
#include "sbas_watchdog/sbas_watchdog.h"

#define SBAS_SYMBOLS_PER_SEC 500

static void sbas_feed_symbols_for(sbas_watchdog_t *wdog, u8 msgnum) {
  for (u32 i = 0; i < SBAS_SYMBOLS_PER_SEC * msgnum; i++) {
    sbas_watchdog_hnd_symbol(wdog);
  }
}

TEST(sbas_watchdog_tests, 9_out_of_16) {
  sbas_watchdog_t wdog;

  sbas_watchdog_init(&wdog);

  EXPECT_FALSE(sbas_watchdog_is_triggered(&wdog));

  u8 losscnt = 0;
  for (u8 msgcnt = 0; msgcnt < 16; msgcnt++) {
    sbas_feed_symbols_for(&wdog, /*msgnum=*/1);
    if (3 == losscnt) {
      sbas_watchdog_hnd_message(&wdog);
      losscnt = 0;
    }
  }

  sbas_feed_symbols_for(&wdog, /*msgnum=*/1);

  EXPECT_TRUE(sbas_watchdog_is_triggered(&wdog));
}

TEST(sbas_watchdog_tests, 8_in_a_row) {
  sbas_watchdog_t wdog;

  sbas_watchdog_init(&wdog);

  EXPECT_FALSE(sbas_watchdog_is_triggered(&wdog));

  sbas_feed_symbols_for(&wdog, /*msgnum=*/8);

  EXPECT_FALSE(sbas_watchdog_is_triggered(&wdog));

  sbas_feed_symbols_for(&wdog, /*msgnum=*/1);

  EXPECT_TRUE(sbas_watchdog_is_triggered(&wdog));
}
