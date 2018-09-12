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

#ifndef GRABBER_H
#define GRABBER_H

#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#define GRABBER_LEN_LOG2_MAX (17)
#define FIXED_GRABBER_LENGTH_DW (1 << GRABBER_LEN_LOG2_MAX)
#define FIXED_GRABBER_LENGTH (4 * FIXED_GRABBER_LENGTH_DW)

#define GRABBER_BUFFER_ALIGN 32
#define GRABBER_LENGTH_ALIGN 32
#define GRABBER_CEIL_DIV(a, b) (((a) + (b)-1) / (b))

#define GRABBER_BUFFER_LENGTH_BYTES(type, count) \
  (GRABBER_LENGTH_ALIGN *                        \
   GRABBER_CEIL_DIV(count * sizeof(type), GRABBER_LENGTH_ALIGN))
#define GRABBER_BUFFER_LENGTH_ELEMENTS(type, count) \
  (GRABBER_CEIL_DIV(GRABBER_BUFFER_LENGTH_BYTES(type, count), sizeof(type)))

#define GRABBER_BUFFER(name, type, count)                \
  type name[GRABBER_BUFFER_LENGTH_ELEMENTS(type, count)] \
      __attribute__((aligned(GRABBER_BUFFER_ALIGN)))

extern u8 pRawGrabberBuffer[FIXED_GRABBER_LENGTH] __attribute__((aligned(32)));

#ifdef __cplusplus
extern "C" {
#endif

u8 *grab_samples(u32 *length, u64 *p_count);

u8 *GrabberGetBufferPt(u32 *length);

#ifdef __cplusplus
}
#endif

#endif /* GRABBER_H */
