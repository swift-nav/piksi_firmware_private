/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <openamp/env.h>
#include <openamp/hil.h>

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <ch.h>
#include <hal.h>

#include "remoteproc_env.h"

#include "gic.h"

static bool env_initialized = false;

#define MUTEX_POOL_COUNT 2
static unsigned int mutex_pool_bitfield = 0;
static mutex_t mutex_pool[MUTEX_POOL_COUNT];

#define IRQ_STATUS_COUNT 2
typedef struct {
  bool allocated;
  volatile bool pending;
  struct proc_vring *vring_hw;
} irq_status_t;
static irq_status_t irq_status[IRQ_STATUS_COUNT];

static remoteproc_env_irq_callback_t irq_callback = NULL;

static void rproc_virtio_irq_handler(void *context);

static int _enable_interrupt(struct proc_vring *vring_hw);
static void _notify(int cpu_id, struct proc_intr *intr_info);
static int _boot_cpu(int cpu_id, unsigned int load_addr);
static void _shutdown_cpu(int cpu_id);

/* Global proc_ops struct */
struct hil_platform_ops proc_ops = {
  .enable_interrupt = _enable_interrupt,
  .notify           = _notify,
  .boot_cpu         = _boot_cpu,
  .shutdown_cpu     = _shutdown_cpu,
};

void remoteproc_env_irq_callback_set(remoteproc_env_irq_callback_t callback)
{
  irq_callback = callback;
}

void remoteproc_env_irq_process(void)
{
  for (int i=0; i<IRQ_STATUS_COUNT; i++) {
    irq_status_t *s = &irq_status[i];
    if (s->pending) {
      s->pending = false;

      if (s->allocated) {
        hil_isr(s->vring_hw);
      }
    }
  }
}

int env_init()
{
  if (env_initialized) {
    return 0;
  }
  env_initialized = true;

  for (int i=0; i<MUTEX_POOL_COUNT; i++) {
    chMtxObjectInit(&mutex_pool[i]);
    mutex_pool_bitfield |= (1 << i);
  }

  for (int i=0; i<IRQ_STATUS_COUNT; i++) {
    irq_status_t *s = &irq_status[i];
    s->allocated = false;
    s->pending = false;
    s->vring_hw = NULL;
  }

  return 0;
}

int env_deinit()
{
  env_initialized = false;
  return 0;
}

void *env_allocate_memory(unsigned int size)
{
  return (malloc(size));
}

void env_free_memory(void *ptr)
{
  if (ptr != NULL) {
    free(ptr);
  }
}

void env_memset(void *ptr, int value, unsigned long size)
{
  memset(ptr, value, size);
}

void env_memcpy(void *dst, void const *src, unsigned long len)
{
  memcpy(dst, src, len);
}

int env_strcmp(const char *dst, const char *src)
{
  return (strcmp(dst, src));
}

void env_strncpy(char *dest, const char *src, unsigned long len)
{
  strncpy(dest, src, len);
}

int env_strncmp(char *dest, const char *src, unsigned long len)
{
  return (strncmp(dest, src, len));
}

unsigned long env_map_vatopa(void *address)
{
  return (unsigned long)address;
}

void *env_map_patova(unsigned long address)
{
  return (void *)address;
}

void env_mb()
{
  asm volatile("dsb" : : : "memory");
}

void env_rmb()
{
  asm volatile("dsb" : : : "memory");
}

void env_wmb()
{
  asm volatile("dsb" : : : "memory");
}

int env_create_mutex(void **lock, int count)
{
  (void)count;
  assert(count == 1);

  for (int i=0; i<MUTEX_POOL_COUNT; i++) {
    if (mutex_pool_bitfield & (1 << i)) {
      *lock = &mutex_pool[i];
      mutex_pool_bitfield &= ~(1 << i);
      return 0;
    }
  }

  return 1;
}

void env_delete_mutex(void *lock)
{
  for (int i=0; i<MUTEX_POOL_COUNT; i++) {
    if (lock == &mutex_pool[i]) {
      mutex_pool_bitfield |= (1 << i);
      return;
    }
  }

  assert(!"invalid mutex");
}

void env_lock_mutex(void *lock)
{
  chMtxLock(lock);
}

void env_unlock_mutex(void *lock)
{
  chMtxUnlock(lock);
}

int env_create_sync_lock(void **lock, int state)
{
  (void)lock;
  (void)state;
  assert(!"unsupported");
  return 0;
}

void env_delete_sync_lock(void *lock)
{
  (void)lock;
  assert(!"unsupported");
}

void env_acquire_sync_lock(void *lock)
{
  (void)lock;
  assert(!"unsupported");
}

void env_release_sync_lock(void *lock)
{
  (void)lock;
  assert(!"unsupported");
}

void env_sleep_msec(int num_msec)
{
  chThdSleepMilliseconds(num_msec);
}

void env_disable_interrupts()
{
  assert(!"unsupported");
}

void env_restore_interrupts()
{
  assert(!"unsupported");
}

void env_register_isr_shared(int vector, void *data,
          void (*isr) (int vector, void *data),
          char *name,
          int shared)
{
  (void)vector;
  (void)data;
  (void)isr;
  (void)name;
  (void)shared;
  assert(!"unsupported");
}

void env_register_isr(int vector, void *data,
          void (*isr) (int vector, void *data))
{
  (void)vector;
  (void)data;
  (void)isr;
  assert(!"unsupported");
}

void env_update_isr(int vector, void *data,
        void (*isr) (int vector, void *data),
        char *name,
        int shared)
{
  (void)vector;
  (void)data;
  (void)isr;
  (void)name;
  (void)shared;
  assert(!"unsupported");
}

void env_enable_interrupt(unsigned int vector, unsigned int priority,
        unsigned int polarity)
{
  (void)vector;
  (void)priority;
  (void)polarity;
  assert(!"unsupported");
}

void env_disable_interrupt(unsigned int vector)
{
  (void)vector;
  assert(!"unsupported");
}

void env_map_memory(unsigned int pa, unsigned int va, unsigned int size,
        unsigned int flags)
{
  (void)pa;
  (void)va;
  (void)size;
  (void)flags;

  /* This function is not implemented. It is assumed that memory will be
   * mapped statically.
   */
}

void env_disable_cache(void)
{

}

void env_flush_invalidate_all_caches(void)
{

}

unsigned long long env_get_timestamp(void)
{
  assert(!"unsupported");
  return 0;
}

int platform_get_processor_info(struct hil_proc *proc, int cpu_id)
{
  assert((unsigned int)cpu_id == HIL_RSVD_CPU_ID);

  extern const struct hil_proc hil_proc;
  env_memcpy(proc, &hil_proc, sizeof(*proc));
  return 0;
}

int platform_get_processor_for_fw(char *fw_name)
{
  (void)fw_name;
  return 1;
}

static int _enable_interrupt(struct proc_vring *vring_hw)
{
  void *context = NULL;

  /* Find an unallocated irq_status structure */
  for (int i=0; i<IRQ_STATUS_COUNT; i++) {
    irq_status_t *s = &irq_status[i];
    if (!s->allocated) {
      s->allocated = true;
      s->vring_hw = vring_hw;
      s->pending = false;
      context = s;
      break;
    }
  }

  if (context == NULL) {
    return 1;
  }

  /* Configure and enable ISR */
  irq_id_t irq_id = vring_hw->intr_info.vect_id;
  gic_handler_register(irq_id, rproc_virtio_irq_handler, context);
  gic_irq_sensitivity_set(irq_id, vring_hw->intr_info.trigger_type);
  gic_irq_priority_set(irq_id, vring_hw->intr_info.priority);
  gic_irq_enable(irq_id);

  return 0;
}

static void _notify(int cpu_id, struct proc_intr *intr_info)
{
  /* Trigger SGI to cpu_id */
  GIC_ICD->ICDSGIR = ((1 << cpu_id)         << 16) |
                     (intr_info->vect_id    << 0);
}

static int _boot_cpu(int cpu_id, unsigned int load_addr)
{
  (void)cpu_id;
  (void)load_addr;
  assert(!"unsupported");
  return 1;
}

static void _shutdown_cpu(int cpu_id)
{
  (void)cpu_id;
  assert(!"unsupported");
}

static void rproc_virtio_irq_handler(void *context)
{
  irq_status_t *s = (irq_status_t *)context;
  s->pending = true;

  if (irq_callback != NULL) {
    irq_callback();
  }
}
