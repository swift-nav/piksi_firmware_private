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

#include <ch.h>

#include <libswiftnav/common.h>

#include "chconf.h"

typedef struct piksi_systime {
  systime_t systime;
  u32 rollover_cnt;
} piksi_systime_t;

#define PIKSI_SYSTIME_RO_CNT_INIT 0
#define PIKSI_SYSTIME_SYSTIME_INIT 0
#define PIKSI_SYSTIME_INIT \
  ((piksi_systime_t){PIKSI_SYSTIME_SYSTIME_INIT, PIKSI_SYSTIME_RO_CNT_INIT})

#ifdef __cplusplus
extern "C" {
#endif

bool piksi_systime_get(piksi_systime_t *t);
bool piksi_systime_get_x(piksi_systime_t *t);

u64 piksi_systime_elapsed_since_us_x(const piksi_systime_t *t);
u64 piksi_systime_elapsed_since_ms_x(const piksi_systime_t *t);
u64 piksi_systime_elapsed_since_s_x(const piksi_systime_t *t);

u64 piksi_systime_to_us(const piksi_systime_t *t);
u64 piksi_systime_to_ms(const piksi_systime_t *t);
u64 piksi_systime_to_s(const piksi_systime_t *t);

void piksi_systime_inc_us(piksi_systime_t *t, u64 inc);
void piksi_systime_inc_ms(piksi_systime_t *t, u64 inc);
void piksi_systime_inc_s(piksi_systime_t *t, u64 inc);

void piksi_systime_dec_us(piksi_systime_t *t, u64 dec);
void piksi_systime_dec_ms(piksi_systime_t *t, u64 dec);
void piksi_systime_dec_s(piksi_systime_t *t, u64 dec);

s8 piksi_systime_cmp(const piksi_systime_t *a, const piksi_systime_t *b);

s64 piksi_systime_sub_us(const piksi_systime_t *a, const piksi_systime_t *b);
s64 piksi_systime_sub_ms(const piksi_systime_t *a, const piksi_systime_t *b);
s64 piksi_systime_sub_s(const piksi_systime_t *a, const piksi_systime_t *b);

void piksi_systime_sleep_us_s(u32 len_us);

void piksi_systime_sleep_ms(u32 len_us);

u32 piksi_systime_sleep_until_us(const piksi_systime_t *t);

u32 piksi_systime_sleep_until_windowed_us(const piksi_systime_t *t,
                                          u32 window_len_us);
u32 piksi_systime_sleep_until_windowed_ms(const piksi_systime_t *t,
                                          u32 window_len_ms);

#ifdef __cplusplus
}
#endif
#endif /* SWIFTNAV_PIKSI_SYSTIME_H */
