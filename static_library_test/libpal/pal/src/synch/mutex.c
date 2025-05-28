#include <string.h>

#include <libpal/error.h>
#include <libpal/impl/synch/mutex.h>
#include <libpal/require.h>
#include <libpal/synch/mutex.h>
#include <pal_private.h>

static struct pal_impl_mutex mutex_impl = {
    .alloc = NULL,
    .free = NULL,
    .lock = NULL,
    .unlock = NULL,
};

void pal_reset_impl_mutex(void) { memset(&mutex_impl, 0, sizeof(mutex_impl)); }

enum pal_error pal_set_impl_mutex(struct pal_impl_mutex *impl) {
  enum pal_error ret =
      pal_require(NULL != impl && NULL != impl->alloc && NULL != impl->free &&
                  NULL != impl->lock && NULL != impl->unlock);

  if (ret == PAL_SUCCESS) {
    mutex_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_mutex(void) { return mutex_impl.alloc != NULL; }

enum pal_error pal_mutex_alloc(pal_mutex_t *mutex) {
  enum pal_error ret = pal_require(pal_has_impl_mutex() && mutex != NULL);
  if (ret == PAL_SUCCESS) {
    ret = mutex_impl.alloc(mutex);
  }
  return ret;
}

enum pal_error pal_mutex_free(pal_mutex_t *mutex) {
  enum pal_error ret = pal_require(mutex != NULL);
  if (ret != PAL_SUCCESS) {
    return ret;
  }
  if (!*mutex) {
    return PAL_SUCCESS;
  }
  ret = pal_require(pal_has_impl_mutex());
  if (ret == PAL_SUCCESS) {
    ret = mutex_impl.free(*mutex);
    *mutex = NULL;
  }
  return ret;
}

enum pal_error pal_mutex_lock(pal_mutex_t mutex) {
  enum pal_error ret = pal_require(pal_has_impl_mutex() && mutex != NULL);
  if (ret == PAL_SUCCESS) {
    ret = mutex_impl.lock(mutex);
  }
  return ret;
}

enum pal_error pal_mutex_unlock(pal_mutex_t mutex) {
  enum pal_error ret = pal_require(pal_has_impl_mutex() && mutex != NULL);
  if (ret == PAL_SUCCESS) {
    ret = mutex_impl.unlock(mutex);
  }
  return ret;
}
