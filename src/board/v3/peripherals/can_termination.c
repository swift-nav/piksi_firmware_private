/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "can_termination.h"

#include <board/nap/nap_common.h>

#include "settings/settings.h"

/* Can termination will only work on a Duro where the can term gpio
 * is wired to a circuit to toggle the 120 ohm resistor.*/

static bool can1_termination = false;

static void can1_termination_configure(bool can_term) {
  if (can_term) {
    nap_set_can_termination();
  } else {
    nap_unset_can_termination();
  }
}

static bool can1_term_notify(struct setting *s, const char *val) {
  bool res = s->type->from_string(s->type->priv, s->addr, s->len, val);
  if (!res) {
    return false;
  }
  can1_termination_configure(can1_termination);
  return true;
}

void can1_termination_init(void) {
  SETTING_NOTIFY(
      "can1", "termination", can1_termination, TYPE_BOOL, can1_term_notify);
}
