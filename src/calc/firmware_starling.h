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

#include <stdbool.h>

/**
 * Top level wrapper for the Starling engine. The
 * entire engine may be disabled from running via
 * runtime setting.
 */

void firmware_starling_setup(void);

/**
 * Return true if the firmware Starling engine is currently
 * enabled. Note that due to the asynchronous nature of this setting,
 * you may get incosistent results from calling this function.
 */
bool firmware_starling_is_enabled(void);

#endif
