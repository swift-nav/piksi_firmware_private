/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_SBP_SETTINGS_H_
#define STARLING_SBP_SETTINGS_H_

#include "sbp_duplex_link.h"

/* Given an SBP link, this function will try to connect a bunch of
 * settings relevant for the Starling engine. */
void starling_register_sbp_settings(const SbpDuplexLink *sbp_link);

/* These settings are still needing to be accessed by the firmware. */
extern bool send_heading;
extern bool disable_raim;
extern double heading_offset;

#endif  // STARLING_SETTINGS_H_
