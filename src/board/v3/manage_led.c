/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "manage_led.h"

#include <ch.h>
#include <hal.h>
#include <libswiftnav/logging.h>

#include "peripherals/led_adp8866.h"
#include "peripherals/antenna.h"
#include "base_obs.h"
#include "solution.h"

#define LED_POS_R   0
#define LED_POS_G   1
#define LED_POS_B   2
#define LED_LINK_R  3
#define LED_LINK_G  4
#define LED_LINK_B  5
#define LED_MODE_R  6
#define LED_MODE_G  7
#define LED_MODE_B  8

#define LED_MAX LED_ADP8866_BRIGHTNESS_MAX

/* LED driver uses a square law output encoding. Calculate RGB components
   such that total current (and thus brightness) is constant. */
#define RGB_TO_RGB_LED_COMPONENT(_r,_g,_b,_c) \
    (LED_MAX * sqrtf((float)(_c) / ((_r) + (_g) + (_b))))

#define RGB_TO_RGB_LED(_r,_g,_b) \
    (((_r) == 0) && ((_g) == 0) && ((_b) == 0)) ? \
    (rgb_led_state_t) { .r = 0, .g = 0, .b = 0 } : \
    (rgb_led_state_t) { \
      .r = RGB_TO_RGB_LED_COMPONENT(_r,_g,_b,_r), \
      .g = RGB_TO_RGB_LED_COMPONENT(_r,_g,_b,_g), \
      .b = RGB_TO_RGB_LED_COMPONENT(_r,_g,_b,_b)  \
    }

#define COUNTER_INTERVAL_ms   MANAGE_LED_THREAD_PERIOD
#define MS2COUNTS(ms)         (((ms) + COUNTER_INTERVAL_ms - 1) / \
                               COUNTER_INTERVAL_ms)

#define INTERVAL_COUNTS           MS2COUNTS(1000)
#define LED_LINK_BLIP_COUNTS      MS2COUNTS(20)
#define SLOW_BLINK_PERIOD_COUNTS  (INTERVAL_COUNTS / 1)
#define FAST_BLINK_PERIOD_COUNTS  (INTERVAL_COUNTS / 2)

#define LED_MODE_TIMEOUT          MS2ST(1500)

#define MANAGE_LED_THREAD_STACK   2000
#define MANAGE_LED_THREAD_PERIOD  MS2ST(10)
#define MANAGE_LED_THREAD_PRIO    (NORMALPRIO + 10)

typedef struct {
  u8 r;
  u8 g;
  u8 b;
} rgb_led_state_t;

/* Must be wide enough to store 2*INTERVAL_COUNTS */
typedef u8 counter_t;

typedef enum {
  BLINK_OFF,
  BLINK_SLOW,
  BLINK_FAST,
  BLINK_ON
} blink_mode_t;

typedef struct {
  blink_mode_t mode;
  counter_t period;
  counter_t counter;
} blinker_state_t;

static const counter_t blink_mode_periods[] = {
  [BLINK_OFF] =   2*INTERVAL_COUNTS, /* Half period will never occur */
  [BLINK_SLOW] =  SLOW_BLINK_PERIOD_COUNTS,
  [BLINK_FAST] =  FAST_BLINK_PERIOD_COUNTS,
  [BLINK_ON] =    0, /* Half period will occur immediately */
};

static THD_WORKING_AREA(wa_manage_led_thread, MANAGE_LED_THREAD_STACK);

static void blinker_reset(blinker_state_t *b, blink_mode_t mode)
{
  b->mode = mode;
  /* Initialize counter to -1 so that it rolls over
   * to zero on the next update */
  b->counter = -1;
  b->period = blink_mode_periods[mode];
}

static bool blinker_update(blinker_state_t *b)
{
  /* Increment counter */
  if (++b->counter >= b->period) {
    b->counter = 0;
  }

  /* Turn on for the second half of the period */
  return (b->counter >= (b->period / 2));
}

static blink_mode_t pos_blink_mode_get(void)
{
  /* Off if no antenna present */
  if (!antenna_present()) {
    return BLINK_OFF;
  }

  /* On if PVT available */
  systime_t last_pvt_systime = solution_last_pvt_stats_get().systime;
  if ((last_pvt_systime != TIME_INFINITE) &&
      (chVTTimeElapsedSinceX(last_pvt_systime) < LED_MODE_TIMEOUT)) {
    return BLINK_ON;
  }

  /* Blink according to signals tracked */
  u8 signals_tracked = solution_last_stats_get().signals_tracked;
  return (signals_tracked > 0) ? BLINK_FAST : BLINK_SLOW;
}

static void handle_pos(counter_t c, rgb_led_state_t *s)
{
  static blinker_state_t blinker_state;

  /* Reset when global counter rolls over */
  if (c == 0) {
    blinker_reset(&blinker_state, pos_blink_mode_get());
  }

  *s = blinker_update(&blinker_state) ? RGB_TO_RGB_LED(255, 131, 0) :
                                        RGB_TO_RGB_LED(0, 0, 0);
}

static void handle_link(counter_t c, rgb_led_state_t *s)
{
  (void)c;

  static u8 last_base_obs_msg_counter = 0;
  static u8 on_counter = 0;

  /* Reset on time if the base obs counter changed */
  u8 base_obs_msg_counter = base_obs_msg_counter_get();
  if (base_obs_msg_counter != last_base_obs_msg_counter) {
    last_base_obs_msg_counter = base_obs_msg_counter;
    on_counter = LED_LINK_BLIP_COUNTS;
  }

  /* Count down to turn off */
  if (on_counter > 0) {
    on_counter--;
  }

  *s = (on_counter > 0) ? RGB_TO_RGB_LED(255, 0, 0) :
                          RGB_TO_RGB_LED(0, 0, 0);
}

static blink_mode_t mode_blink_mode_get(void)
{
  soln_dgnss_stats_t last_dgnss_stats = solution_last_dgnss_stats_get();

  /* Off if no DGNSS */
  if ((last_dgnss_stats.systime == TIME_INFINITE) ||
      (chVTTimeElapsedSinceX(last_dgnss_stats.systime) >= LED_MODE_TIMEOUT)) {
    return BLINK_OFF;
  }

  /* On if fixed, blink if float */
  return (last_dgnss_stats.mode == FILTER_FIXED) ? BLINK_ON : BLINK_SLOW;
}

static void handle_mode(counter_t c, rgb_led_state_t *s)
{
  static blinker_state_t blinker_state;

  /* Reset when global counter rolls over */
  if (c == 0) {
    blinker_reset(&blinker_state, mode_blink_mode_get());
  }

  *s = blinker_update(&blinker_state) ? RGB_TO_RGB_LED(0, 0, 255) :
                                        RGB_TO_RGB_LED(0, 0, 0);
}

static void manage_led_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage LED");

  led_adp8866_init();

  while (TRUE) {
    static counter_t counter = 0;

    rgb_led_state_t pos_state;
    handle_pos(counter, &pos_state);

    rgb_led_state_t link_state;
    handle_link(counter, &link_state);

    rgb_led_state_t mode_state;
    handle_mode(counter, &mode_state);

    led_adp8866_led_state_t led_states[] = {
      { .led = LED_POS_R,   .brightness = pos_state.r },
      { .led = LED_POS_G,   .brightness = pos_state.g },
      { .led = LED_POS_B,   .brightness = pos_state.b },
      { .led = LED_LINK_R,  .brightness = link_state.r },
      { .led = LED_LINK_G,  .brightness = link_state.g },
      { .led = LED_LINK_B,  .brightness = link_state.b },
      { .led = LED_MODE_R,  .brightness = mode_state.r },
      { .led = LED_MODE_G,  .brightness = mode_state.g },
      { .led = LED_MODE_B,  .brightness = mode_state.b }
    };
    led_adp8866_leds_set(led_states, sizeof(led_states)/sizeof(led_states[0]));

    if (++counter >= INTERVAL_COUNTS) {
      counter = 0;
    }

    chThdSleep(MANAGE_LED_THREAD_PERIOD);
  }
}

void manage_led_setup(void)
{
  chThdCreateStatic(wa_manage_led_thread, sizeof(wa_manage_led_thread),
                    MANAGE_LED_THREAD_PRIO, manage_led_thread, NULL);
}
