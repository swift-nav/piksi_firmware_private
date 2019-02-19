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

#ifndef STARLING_SBP_LINK_H_
#define STARLING_SBP_LINK_H_

#include "sbp_duplex_link.h"

/* Singleton. Becomes available after a call to setup. */
extern SbpDuplexLink *sbp_link;

/* Call this and then you may use the global object. Redundant calls will
 * be ignored. */
void starling_sbp_link_setup(void);

#endif
