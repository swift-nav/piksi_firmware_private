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

#include "manage_rtc.h"

#include <ch.h>
#include <hal.h>
#include <libswiftnav/logging.h>

#include "peripherals/rtc_m41t62.h"
#include "nap/nap_rtc_hw.h"

#define MANAGE_RTC_THREAD_STACK   2000
#define MANAGE_RTC_THREAD_PERIOD  S2ST(10)
#define MANAGE_RTC_THREAD_PRIO    (LOWPRIO + 5)

/* Allow some margin to ensure a second interrupt does not occur. */
#define NAP_32k_LIMIT 16384 /* 0.5s */

static THD_WORKING_AREA(wa_manage_rtc_thread, MANAGE_RTC_THREAD_STACK);

static bool time_read_precise(rtc_m41t62_time_t *rtc_time, u64 *nap_tc)
{
  bool success = false;

  /* Enable square wave and IRQ output and wait for 1s boundary */
  if (rtc_m41t62_second_wait()) {

    /* Read RTC time */
    bool time_valid;
    bool time_get_ok = rtc_m41t62_time_get(rtc_time, &time_valid);

    /* Set centiseconds to zero (aligned to 1s boundary) */
    rtc_time->centisecond = 0;

    /* Read latched NAP timing count aligned to 1s boundary (time of IRQ) */
    *nap_tc = NAP_RTC->SAMPLE_COUNT;

    /* Make sure everything was completed before the second rolls over */
    bool nap_32k_ok = (NAP_RTC->RTC_COUNT < NAP_32k_LIMIT);

    success = time_get_ok && time_valid && nap_32k_ok;
  }

  /* Disable square wave output */
  rtc_m41t62_second_wait_cleanup();

  return success;
}

static void manage_rtc_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("manage RTC");

  rtc_m41t62_init();

  while (TRUE) {

    /* TODO: Check for precise GNSS time and initialize RTC if necessary */

    /* Do read sequence */
    rtc_m41t62_time_t rtc_time;
    u64 nap_tc;
    if (time_read_precise(&rtc_time, &nap_tc)) {
      /* TODO: update calibration */
    }

    chThdSleep(MANAGE_RTC_THREAD_PERIOD);
  }
}

void manage_rtc_setup(void)
{
  chThdCreateStatic(wa_manage_rtc_thread, sizeof(wa_manage_rtc_thread),
                    MANAGE_RTC_THREAD_PRIO, manage_rtc_thread, NULL);
}
