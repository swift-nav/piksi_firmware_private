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
 * Top level wrapper for the Starling engine. The
 * entire engine may be disabled from running via
 * runtime setting.
 */

/**
 * Registers settings for running (or not running)
 * Starling. This should be called as early as possible
 * after the settings system has been initialized.
 */
void firmware_starling_init(void);


/**
 * Run the Starling engine (or not), depending
 * on the user configurable settings.
 */
void firmware_starling_run(void);

#endif


