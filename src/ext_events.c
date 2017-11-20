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
#include <libswiftnav/logging.h>

#include "./ext_events.h"
#include "./sbp.h"
#include "./sbp_utils.h"
#include "./settings.h"
#include "./timing.h"
#include "board/nap/nap_common.h"

/** \defgroup ext_events External Events
 * Capture accurate timestamps of external pin events
 * \{ */

static struct {
  ext_event_trigger_t trigger;
  u32 timeout_microseconds;
} event_config[3];

/** Settings callback to inform NAP which trigger mode and timeout is desired */
static bool event0_changed(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    u8 pin = 0;
    nap_set_ext_event(
        pin, event_config[pin].trigger, event_config[pin].timeout_microseconds);
    return true;
  }
  return false;
}

/** Settings callback to inform NAP which trigger mode and timeout is desired */
static bool event1_changed(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    u8 pin = 1;
    nap_set_ext_event(
        pin, event_config[pin].trigger, event_config[pin].timeout_microseconds);
    return true;
  }
  return false;
}

/** Settings callback to inform NAP which trigger mode and timeout is desired */
static bool event2_changed(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    u8 pin = 2;
    nap_set_ext_event(
        pin, event_config[pin].trigger, event_config[pin].timeout_microseconds);
    return true;
  }
  return false;
}

/** Set up the external event detection system
 *
 * Informs the NAP of the desired trigger mode, and registers a settings
 * callback
 * to update the NAP if the trigger mode is changed.
 *
 */
void ext_event_setup(void) {
  static const char * const trigger_enum[] = {
      "None", "Rising", "Falling", "Both", NULL};
  static struct setting_type trigger_setting;
  int TYPE_TRIGGER =
      settings_type_register_enum(trigger_enum, &trigger_setting);

  SETTING_NOTIFY("ext_events_0",
                 "edge_trigger",
                 event_config[0].trigger,
                 TYPE_TRIGGER,
                 event0_changed);
  SETTING_NOTIFY("ext_events_0",
                 "sensitivity",
                 event_config[0].timeout_microseconds,
                 TYPE_INT,
                 event0_changed);

  SETTING_NOTIFY("ext_events_1",
                 "edge_trigger",
                 event_config[1].trigger,
                 TYPE_TRIGGER,
                 event1_changed);
  SETTING_NOTIFY("ext_events_1",
                 "sensitivity",
                 event_config[1].timeout_microseconds,
                 TYPE_INT,
                 event1_changed);

  SETTING_NOTIFY("ext_events_2",
                 "edge_trigger",
                 event_config[2].trigger,
                 TYPE_TRIGGER,
                 event2_changed);
  SETTING_NOTIFY("ext_events_2",
                 "sensitivity",
                 event_config[2].timeout_microseconds,
                 TYPE_INT,
                 event2_changed);
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
    if (tc.half[0] < event_nap_time) { /* Rollover occurred since event */
      tc.half[1]--;
    }
    tc.half[0] = event_nap_time;

    msg_ext_event_t msg;
    msg.flags = (event_trig == TRIG_RISING) ? (1 << 0) : (0 << 0);
    /* Is gps time good, i.e. within 1 microsecond */
    if (get_time_quality() >= TIME_PROPAGATED) {
      msg.flags |= (1 << 1);
    }
    msg.pin = event_pin;

    /* Convert to the SBP convention of rounded ms + signed ns residual */
    gps_time_t gpst = napcount2gpstime(tc.full);
    msg_gps_time_t mgt;
    sbp_make_gps_time(&mgt, &gpst, 0);
    msg.wn = mgt.wn;
    msg.tow = mgt.tow;
    msg.ns_residual = mgt.ns_residual;

    sbp_send_msg(SBP_MSG_EXT_EVENT, sizeof(msg), (u8 *)&msg);
  }
}

/** \} */
