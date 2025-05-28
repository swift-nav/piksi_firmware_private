#include <string.h>

#include <libpal/impl/mem/mem.h>
#include <libpal/mem/mem.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_mem mem_impl = {
    .alloc = NULL,
    .free = NULL,
};

void pal_reset_impl_mem(void) { memset(&mem_impl, 0, sizeof(mem_impl)); }

enum pal_error pal_set_impl_mem(struct pal_impl_mem *impl) {
  enum pal_error ret =
      pal_require(impl != NULL && impl->alloc != NULL && impl->free != NULL);
  if (ret == PAL_SUCCESS) {
    mem_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_mem() { return mem_impl.alloc != NULL; }

enum pal_error pal_mem_alloc(void **ptr, size_t size) {
  enum pal_error ret = pal_require(pal_has_impl_mem() && ptr != NULL);
  if (ret == PAL_SUCCESS) {
    ret = mem_impl.alloc(ptr, size);
  }
  return ret;
}

enum pal_error pal_mem_free(void **ptr) {
  enum pal_error ret = pal_require(ptr != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*ptr) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_mem());
  if (ret == PAL_SUCCESS) {
    ret = mem_impl.free(*ptr);
    *ptr = NULL;
  }
  return ret;
}
