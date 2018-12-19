/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef MESTA_NAP_HW_H_INCLUDED
#define MESTA_NAP_HW_H_INCLUDED

#include "src/board/v3/nap/nap_hw.h"

extern swiftnap_t mesta_nap;

#undef NAP
#define NAP (&mesta_nap)

#endif /* #include MESTA_NAP_HW_H_INCLUDED */
