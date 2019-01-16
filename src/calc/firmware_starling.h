/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef CALC_FIRMWARE_STARLING_H_
#define CALC_FIRMWARE_STARLING_H_

/**
 * This initializes the Starling platform and API, and must take place
 * prior to any calls into the Starling engine. This is a lightweight
 * function and should be placed very early during startup.
 */
void firmware_starling_preinit(void);

/**
 * Allow the Starling engine to start (if configured to run on firmware).
 * This is a heavyweight call which initializes some rather large resources.
 * As long as the preinit call has been made at some point, this can be
 * delayed arbitrarily with no adverse effects.
 */
void firmware_starling_setup(void);

#endif
