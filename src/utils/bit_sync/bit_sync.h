/*
 * Copyright (c) 2015,2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_BIT_SYNC_H
#define SWIFTNAV_BIT_SYNC_H

#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include "gnss_capabilities/gnss_capabilities.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** \addtogroup bit_sync
 * \{ */

#define BITSYNC_UNSYNCED (-1)

#define BIT_LENGTH_MAX 100

/** Structure containing bit sync state for a signal */
typedef struct {
  u8 bit_length; /** length of a single bit */

  u8 bit_phase;
  s8 bit_phase_ref; /**< -1 = not synced.*/
  s32 bit_integrate;

  s8 histogram[BIT_LENGTH_MAX];
  s32 prev_real;
  s32 prev_imag;

  me_gnss_signal_t mesid;

} bit_sync_t;

/** \} */

void bit_sync_init(bit_sync_t *b, me_gnss_signal_t mesid);
void bit_sync_set(bit_sync_t *b, s8 bit_phase_ref);
bool bit_sync_update(bit_sync_t *b,
                     s32 corr_prompt_real,
                     s32 corr_prompt_imag,
                     u32 ms,
                     s32 *bit_integrate);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_BIT_SYNC_H */
