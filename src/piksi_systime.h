/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_PIKSI_SYSTIME_H
#define SWIFTNAV_PIKSI_SYSTIME_H

#define memory_pool_t MemoryPool
#include <ch.h>
#undef memory_pool_t

#include <libswiftnav/common.h>

typedef struct piksi_systime {
  systime_t systime;
  u32 rollover_cnt;
} piksi_systime_t;

#define PIKSI_SYSTIME_RO_CNT_INIT 0
#define PIKSI_SYSTIME_SYSTIME_INIT 0
#define PIKSI_SYSTIME_INIT ((piksi_systime_t){PIKSI_SYSTIME_SYSTIME_INIT, \
                                              PIKSI_SYSTIME_RO_CNT_INIT})

bool piksi_systime_get(piksi_systime_t *t);
bool piksi_systime_get_x(piksi_systime_t *t);

u64 piksi_systime_to_ticks(const piksi_systime_t *t);

systime_t piksi_systime_elapsed_since_x(const piksi_systime_t *t);

bool piksi_systime_add(piksi_systime_t *t, systime_t a);
s8 piksi_systime_cmp(const piksi_systime_t *a, const piksi_systime_t *b);
systime_t piksi_systime_sub(const piksi_systime_t *a, const piksi_systime_t *b);

systime_t piksi_systime_sleep_until_windowed(const piksi_systime_t *t,
                                             systime_t window_len);

#endif /* SWIFTNAV_PIKSI_SYSTIME_H */

