/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_ERROR_H
#define SWIFTNAV_ERROR_H

#define screaming_death(...) \
  _screaming_death(__VA_ARGS__)
__attribute__((noreturn)) void _screaming_death(const char *fmt, ...);

void fault_handling_setup(void);

#endif /* SWIFTNAV_ERROR_H */
