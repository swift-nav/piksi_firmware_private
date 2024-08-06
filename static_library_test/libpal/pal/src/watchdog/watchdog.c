#include <string.h>

#include <libpal/impl/watchdog/watchdog.h>
#include <libpal/require.h>
#include <libpal/watchdog/watchdog.h>
#include <pal_private.h>

static struct pal_impl_watchdog watchdog_impl = {
    .notify = NULL,
};

void pal_reset_impl_watchdog(void) {
  memset(&watchdog_impl, 0, sizeof(watchdog_impl));
}

enum pal_error pal_set_impl_watchdog(struct pal_impl_watchdog *impl) {
  enum pal_error ret = pal_require(NULL != impl && NULL != impl->notify

  );
  if (ret == PAL_SUCCESS) {
    watchdog_impl = *impl;
  }
  return ret;
}

bool pal_has_impl_watchdog() { return watchdog_impl.notify != NULL; }

enum pal_error pal_watchdog_notify() {
  enum pal_error ret = pal_require(pal_has_impl_watchdog());
  if (ret == PAL_SUCCESS) {
    ret = watchdog_impl.notify();
  }
  return ret;
}
