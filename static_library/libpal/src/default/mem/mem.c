#include <stddef.h>

#include <libpal/impl/mem/mem.h>
#include <libpal/mem/mem.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default Memory Allocation Routine
 * @param size: unused
 * @param ptr: unused
 * @return NULL
 */
static enum pal_error default_mem_alloc(void **ptr, size_t size) {
  NOT_IMPLEMENTED();
  (void)ptr;
  (void)size;
  return PAL_INVALID;
}

/**
 * Default Memory Free Routine
 * @param ptr: unused
 */
static enum pal_error default_mem_free(void *ptr) {
  NOT_IMPLEMENTED();
  (void)ptr;
  return PAL_INVALID;
}

static struct pal_impl_mem mem_impl = {
    .alloc = default_mem_alloc,
    .free = default_mem_free,
};

void pal_reset_impl_mem(void) {
  mem_impl.alloc = default_mem_alloc;
  mem_impl.free = default_mem_free;
}

void pal_set_impl_mem(struct pal_impl_mem *impl) {
  assert(NULL != impl);
  assert(NULL != impl->alloc);
  assert(NULL != impl->free);

  mem_impl = *impl;
}

bool pal_has_impl_mem() { return mem_impl.alloc != default_mem_alloc; }

enum pal_error pal_mem_alloc(void **ptr, size_t size) {
  return mem_impl.alloc(ptr, size);
}
enum pal_error pal_mem_free(void **ptr) {
  if (!ptr) {
    return PAL_INVALID;
  }
  enum pal_error ret = mem_impl.free(*ptr);
  if (ret == PAL_SUCCESS) {
    *ptr = NULL;
  }
  return ret;
}
