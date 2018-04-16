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

static const tracker_interface_t tracker_interface_default = {
    .code = CODE_INVALID, .init = 0, .disable = 0, .update = 0,
};

static tracker_interface_list_element_t *tracker_interface_list = NULL;

/** Return a pointer to the tracker interface list. */
tracker_interface_list_element_t **tracker_interface_list_ptr_get(void) {
  return &tracker_interface_list;
}

/** Register a tracker interface to enable tracking for a code type.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void tracker_interface_register(tracker_interface_list_element_t *element) {
  /* p_next = address of next pointer which must be updated */
  tracker_interface_list_element_t **p_next = tracker_interface_list_ptr_get();

  while (NULL != *p_next) {
    p_next = &(*p_next)->next;
  }

  element->next = NULL;
  *p_next = element;
}

/** Look up the tracker interface for the specified mesid.
 *
 * \param mesid ME signal to be tracked.
 *
 * \return Associated tracker interface. May be the default interface.
 */
const tracker_interface_t *tracker_interface_lookup(
    const me_gnss_signal_t mesid) {
  const tracker_interface_list_element_t *e = *tracker_interface_list_ptr_get();
  while (NULL != e) {
    const tracker_interface_t *interface = e->interface;
    if (interface->code == mesid.code) {
      return interface;
    }
    e = e->next;
  }

  return &tracker_interface_default;
}
