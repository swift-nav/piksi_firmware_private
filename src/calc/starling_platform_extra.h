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
#ifndef STARLING_PLATFORM_EXTRA_H_
#define STARLING_PLATFORM_EXTRA_H_

/**
 * Bare-bones semaphore abstraction used for implementating
 * the Starling integration layer.
 *
 * If you want everything to work out of the box, you should provide
 * a platform-specific implementation of this API.
 */

#define PLATFORM_SEM_OK      0
#define PLATFORM_SEM_TIMEOUT 1
#define PLATFORM_SEM_ERROR   2

typedef struct platform_sem_t platform_sem_t;

platform_sem_t *platform_sem_create(void);

platform_sem_t *platform_sem_create_count(int count);

void platform_sem_destroy(platform_sem_t **sem_loc);

void platform_sem_signal(platform_sem_t *sem);


/**
 * "Wait" functions return 0 on valid decrement operation,
 * non-zero on timeout or otherwise spurious return.
 */

int platform_sem_wait(platform_sem_t *sem);

int platform_sem_wait_timeout(platform_sem_t *sem, unsigned long millis);

#endif


