#include <libpal/impl/watchdog/watchdog.h>
#include <libpal/watchdog/watchdog.h>
#include <pal_private.h>

#include "not_implemented.h"

static enum pal_error default_watchdog_notify(void) {
  NOT_IMPLEMENTED();
  return PAL_INVALID;
}

static struct pal_impl_watchdog watchdog_impl = {
    .notify = default_watchdog_notify,
};

void pal_reset_impl_watchdog(void) {
  watchdog_impl.notify = default_watchdog_notify;
}

void pal_set_impl_watchdog(struct pal_impl_watchdog *impl) {
  assert(NULL != impl);
  assert(NULL != impl->notify);

  watchdog_impl = *impl;
}

bool pal_has_impl_watchdog() {
  return watchdog_impl.notify != default_watchdog_notify;
}

enum pal_error pal_watchdog_notify() { return watchdog_impl.notify(); }
