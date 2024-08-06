#include <string.h>

#include <libpal/impl/thread/thread.h>
#include <libpal/require.h>
#include <libpal/thread/thread.h>
#include <pal_private.h>

static struct pal_impl_thread thread_impl = {
    .create = NULL,
    .join = NULL,
    .interrupt = NULL,
};

void pal_reset_impl_thread(void) {
  memset(&thread_impl, 0, sizeof(thread_impl));
}

enum pal_error pal_set_impl_thread(struct pal_impl_thread *impl) {
  enum pal_error ret =
      pal_require(NULL != impl && NULL != impl->create && NULL != impl->join);
  if (ret == PAL_SUCCESS) {
    thread_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_thread() { return thread_impl.create != NULL; }

bool pal_has_impl_thread_interrupt() {
  return pal_has_impl_thread() && thread_impl.interrupt != NULL;
}

enum pal_error pal_thread_create(pal_thread_t *thread, const char *name,
                                 pal_thread_entry_t entry, void *ctx,
                                 size_t stacksize) {
  enum pal_error ret =
      pal_require(pal_has_impl_thread() && thread != NULL && entry != NULL);
  if (ret == PAL_SUCCESS) {
    ret = thread_impl.create(thread, name, entry, ctx, stacksize);
  }
  return ret;
}

enum pal_error pal_thread_join(pal_thread_t thread) {
  enum pal_error ret = pal_require(pal_has_impl_thread() && thread != NULL);
  if (ret == PAL_SUCCESS) {
    ret = thread_impl.join(thread);
  }
  return ret;
}

enum pal_error pal_thread_interrupt(pal_thread_t thread) {
  enum pal_error ret =
      pal_require(pal_has_impl_thread_interrupt() && thread != NULL);
  if (ret == PAL_SUCCESS) {
    ret = thread_impl.interrupt(thread);
  }
  return ret;
}
