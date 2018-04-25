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

#ifndef _CH_H_
#define _CH_H_

#define CH_KERNEL_MAJOR 3
#define CH_KERNEL_MINOR 1
#define CH_KERNEL_PATCH 3

typedef unsigned int systime_t;
typedef unsigned int binary_semaphore_t;
typedef unsigned int mutex_t;

void chSysLock(void);
void chSysUnlock(void);
systime_t chVTGetSystemTimeX(void);
systime_t chThdSleep(systime_t time);
systime_t chThdSleepS(systime_t time);
systime_t chThdSleepMilliseconds(systime_t time);

#define TIME_INFINITE (systime_t)(-1)

#endif /* _CH_H_ */

/** @} */
