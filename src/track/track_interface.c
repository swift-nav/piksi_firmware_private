/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_interface.h"

static tracker_interface_t const *tracker_interface[CODE_COUNT];

/** Register a tracker interface to enable tracking for a code type.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void tracker_interface_register(const tracker_interface_t *element) {
  tracker_interface[element->code] = element;
}

/** Look up the tracker interface for the specified mesid.
 *
 * \param code
 *
 * \return Associated tracker interface. May be the default interface.
 */
const tracker_interface_t *tracker_interface_lookup(const code_t code) {
  if (NULL == tracker_interface[code]) {
    log_error("tracker interface missing for code %d", (s8)code);
    assert(0);
  }
  return tracker_interface[code];
}
