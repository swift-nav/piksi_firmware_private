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

#include <starling/platform/starling_platform_semaphore.h>

#include <ch.h>

#define MAX_N_SEMAPHORES 8

static int convert_chibios_ret(msg_t ret) {
  switch (ret) {
    case MSG_OK:
      return PLATFORM_SEM_OK;
    case MSG_TIMEOUT:
      return PLATFORM_SEM_TIMEOUT;
    default:
      return PLATFORM_SEM_ERROR;
  }
}

platform_sem_t *platform_sem_create(void) {
  return platform_sem_create_count(0);
}

/**
 * We make no effort here to reuse destroyed semaphores,
 * there is an upper bound on the number of semaphores which
 * may be created during a single execution, and that is that.
 */
platform_sem_t *platform_sem_create_count(int count) {
  static int n_semaphores = 0;
  static semaphore_t semaphores[MAX_N_SEMAPHORES];

  if (n_semaphores >= MAX_N_SEMAPHORES) {
    return NULL;
  }

  semaphore_t *sem = &semaphores[n_semaphores++];

  chSemObjectInit(sem, count);
  return (platform_sem_t *)sem;
}

void platform_sem_destroy(platform_sem_t **sem_loc) {
  if (sem_loc) {
    *sem_loc = NULL;
  }
}

void platform_sem_signal(platform_sem_t *sem) {
  chSemSignal((semaphore_t *)sem);
}

int platform_sem_wait(platform_sem_t *sem) {
  int ret = chSemWait((semaphore_t *)sem);
  return convert_chibios_ret(ret);
}

int platform_sem_wait_timeout(platform_sem_t *sem, unsigned long millis) {
  const systime_t timeout = MS2ST(millis);
  int ret = chSemWaitTimeout((semaphore_t *)sem, timeout);
  return convert_chibios_ret(ret);
}
