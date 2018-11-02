/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libsbp/ext_events.h>
#include <libsbp/navigation.h>
#include <swiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "ext_events.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "timing/timing.h"

/** \defgroup ext_events External Events
 * Capture accurate timestamps of external pin events
 * \{ */

typedef struct event_config_s {
  u8 pin;
  ext_event_trigger_t trigger;
  u32 timeout_microseconds;
} event_config_t;

static event_config_t event_config[3] = {
  [0] = { .pin = 0, .trigger = 0, .timeout_microseconds = 0 },
  [1] = { .pin = 1, .trigger = 0, .timeout_microseconds = 0 },
  [2] = { .pin = 2, .trigger = 0, .timeout_microseconds = 0 },
};

/** Settings callback to inform NAP which trigger mode and timeout is desired */
static int event0_changed(void *ctx) {
  event_config_t *cfg = (event_config_t *)ctx;

  nap_set_ext_event(cfg->pin, cfg->trigger, cfg->timeout_microseconds);

  return SBP_SETTINGS_WRITE_STATUS_OK;
}

/** Set up the external event detection system
 *
 * Informs the NAP of the desired trigger mode, and registers a settings
 * callback
 * to update the NAP if the trigger mode is changed.
 *
 */
void ext_event_setup(void) {
  static const char *const trigger_enum[] = {
      "None", "Rising", "Falling", "Both", NULL};
  settings_type_t trigger_setting;
  settings_type_register_enum(trigger_enum, &trigger_setting);

  SETTING_NOTIFY_CTX("ext_event_a",
                     "edge_trigger",
                     event_config[0].trigger,
                     trigger_setting,
                     event0_changed,
                     &event_config[0]);
  SETTING_NOTIFY_CTX("ext_event_a",
                     "sensitivity",
                     event_config[0].timeout_microseconds,
                     SETTINGS_TYPE_INT,
                     event0_changed,
                     &event_config[0]);

  SETTING_NOTIFY_CTX("ext_event_b",
                     "edge_trigger",
                     event_config[1].trigger,
                     trigger_setting,
                     event0_changed,
                     &event_config[1]);
  SETTING_NOTIFY_CTX("ext_event_b",
                     "sensitivity",
                     event_config[1].timeout_microseconds,
                     SETTINGS_TYPE_INT,
                     event0_changed,
                     &event_config[1]);

  SETTING_NOTIFY_CTX("ext_event_c",
                     "edge_trigger",
                     event_config[2].trigger,
                     trigger_setting,
                     event0_changed,
                     &event_config[2]);
  SETTING_NOTIFY_CTX("ext_event_c",
                     "sensitivity",
                     event_config[2].timeout_microseconds,
                     SETTINGS_TYPE_INT,
                     event0_changed,
                     &event_config[2]);
}

/** Service an external event interrupt
 *
 * When an event occurs (i.e. pin edge) that matches the NAP's trigger
 * condition, the NAP will latch the time, pin number and trigger direction.
 * It will also set an IRQ bit which will lead to an EXTI.  The firmware
 * EXTI handling routine handle_nap_exti() will call this function, which
 * reads out the details and spits them out as an SBP message to our host.
 *
 */
void ext_event_service(u32 events) {
  while (events) {
    u8 event_pin;
    ext_event_trigger_t event_trig;

    if (events & NAP_IRQS_EXT_EVENT0_Msk) {
      event_pin = 0;
      events &= ~NAP_IRQS_EXT_EVENT0_Msk;
    } else if (events & NAP_IRQS_EXT_EVENT1_Msk) {
      event_pin = 1;
      events &= ~NAP_IRQS_EXT_EVENT1_Msk;
    } else if (events & NAP_IRQS_EXT_EVENT2_Msk) {
      event_pin = 2;
      events &= ~NAP_IRQS_EXT_EVENT2_Msk;
    } else {
      return;
    }

    u32 event_nap_time = nap_get_ext_event(event_pin, &event_trig);

    union {
      u32 half[2];
      u64 full;
    } tc;
    tc.full = nap_timing_count();
    u8 time_qual = get_time_quality();
    if (tc.half[0] < event_nap_time) { /* Rollover occurred since event */
      tc.half[1]--;
    }
    tc.half[0] = event_nap_time;

    msg_ext_event_t msg;
    msg.flags = (event_trig == TRIG_RISING) ? (1 << 0) : (0 << 0);
    /* Is gps time good, i.e. within 1 microsecond */
    if (time_qual >= TIME_PROPAGATED) {
      msg.flags |= (1 << 1);
    }
    msg.pin = event_pin;

    /* Convert to the SBP convention of rounded ms + signed ns residual */
    gps_time_t gpst = napcount2gpstime(tc.full);
    msg_gps_time_t mgt;
    sbp_make_gps_time(&mgt, &gpst, time_qual);
    msg.wn = mgt.wn;
    msg.tow = mgt.tow;
    msg.ns_residual = mgt.ns_residual;

    sbp_send_msg(SBP_MSG_EXT_EVENT, sizeof(msg), (u8 *)&msg);
  }
}

/** \} */
