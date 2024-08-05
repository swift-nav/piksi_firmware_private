#include <stddef.h>

#include <libpal/error.h>
#include <libpal/impl/synch/mutex.h>
#include <libpal/synch/mutex.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default Mutex Allocation Routine
 * @param mutex Unused
 * @return PAL_INVALID
 */
static enum pal_error default_mutex_alloc(pal_mutex_t *mutex) {
  NOT_IMPLEMENTED();
  (void)mutex;
  return PAL_INVALID;
}

/**
 * Default Mutex Free Routine
 * @param mutex unused
 * @return PAL_INVALID
 */
static enum pal_error default_mutex_free(pal_mutex_t mutex) {
  NOT_IMPLEMENTED();
  (void)mutex;
  return PAL_INVALID;
}

/**
 * Default Mutex Lock Routine
 * @param mutex unused
 * @return PAL_INVALID
 */
static enum pal_error default_mutex_lock(pal_mutex_t mutex) {
  NOT_IMPLEMENTED();
  (void)mutex;
  return PAL_INVALID;
}

/**
 * Default Mutex Unlock Routine
 * @param mutex unused
 * @return PAL_INVALID
 */
static enum pal_error default_mutex_unlock(pal_mutex_t mutex) {
  NOT_IMPLEMENTED();
  (void)mutex;
  return PAL_INVALID;
}

static struct pal_impl_mutex mutex_impl = {
    .alloc = default_mutex_alloc,
    .free = default_mutex_free,
    .lock = default_mutex_lock,
    .unlock = default_mutex_unlock,
};

void pal_reset_impl_mutex(void) {
  mutex_impl.alloc = default_mutex_alloc;
  mutex_impl.free = default_mutex_free;
  mutex_impl.lock = default_mutex_lock;
  mutex_impl.unlock = default_mutex_unlock;
}

void pal_set_impl_mutex(struct pal_impl_mutex *impl) {
  assert(NULL != impl);
  assert(NULL != impl->alloc);
  assert(NULL != impl->free);
  assert(NULL != impl->lock);
  assert(NULL != impl->unlock);

  mutex_impl = *impl;
}

bool pal_has_impl_mutex() { return mutex_impl.alloc != default_mutex_alloc; }

enum pal_error pal_mutex_alloc(pal_mutex_t *mutex) {
  return mutex_impl.alloc(mutex);
}

enum pal_error pal_mutex_free(pal_mutex_t *mutex) {
  if (!mutex) {
    return PAL_INVALID;
  }
  enum pal_error ret = mutex_impl.free(*mutex);
  if (ret == PAL_SUCCESS) {
    *mutex = NULL;
  }
  return ret;
}

enum pal_error pal_mutex_lock(pal_mutex_t mutex) {
  return mutex_impl.lock(mutex);
}

enum pal_error pal_mutex_unlock(pal_mutex_t mutex) {
  return mutex_impl.unlock(mutex);
}
