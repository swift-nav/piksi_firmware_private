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

#ifndef MESTA_CH_H_INCLUDED
#define MESTA_CH_H_INCLUDED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define asm(x)

#define CH_KERNEL_MAJOR 3
#define CH_KERNEL_MINOR 1
#define CH_KERNEL_PATCH 3

#define HIGHPRIO 0
#define LOWPRIO 0
#define NORMALPRIO 0

#define CH_CFG_ST_FREQUENCY 4000

#define MSG_OK 1

#define MUTEX_DECL(x) int x
#define BSEMAPHORE_DECL(sem, state) int sem = state

#define THD_WORKING_AREA(s, n) uint64_t s[n / sizeof(uint64_t)]

#define TIME_INFINITE (systime_t)(-1)
#define TIME_IMMEDIATE ((systime_t)-1)

#define MS2ST(x) x

#define THD_FUNCTION(tname, arg) __attribute__((noreturn)) void tname(void *arg)

typedef int binary_semaphore_t;
typedef int mutex_t;
typedef int systime_t;
typedef int thread_t;
typedef int tprio_t;
typedef void (*tfunc_t)(void *p);
typedef int msg_t;

void chMtxObjectInit(mutex_t *mp);

void chMtxLock(mutex_t *mp);
void chMtxUnlock(mutex_t *mp);

systime_t chThdSleepS(systime_t time);

thread_t *chThdCreateStatic(
    void *wsp, size_t size, tprio_t prio, tfunc_t pf, void *arg);
systime_t chVTGetSystemTimeX(void);

void chRegSetThreadNameX(thread_t *tp, const char *name);

void chRegSetThreadName(const char *name);

systime_t chThdSleepMilliseconds(systime_t time);

void chSysLock(void);
void chSysUnlock(void);

void chSysLockFromISR(void);
void chSysUnlockFromISR(void);

systime_t chThdSleep(systime_t time);
void chBSemSignalI(binary_semaphore_t *bsp);
void chBSemSignal(binary_semaphore_t *bsp);

void chBSemObjectInit(binary_semaphore_t *bsp, bool taken);

msg_t chBSemWaitTimeoutS(binary_semaphore_t *bsp, systime_t time);
msg_t chBSemWaitTimeout(binary_semaphore_t *bsp, systime_t time);

void chBSemResetI(binary_semaphore_t *bsp, bool taken);
void chBSemReset(binary_semaphore_t *bsp, bool taken);

bool chMtxTryLockS(mutex_t *mp);
bool chMtxTryLock(mutex_t *mp);

#endif /* #ifndef MESTA_CH_H_INCLUDED */
