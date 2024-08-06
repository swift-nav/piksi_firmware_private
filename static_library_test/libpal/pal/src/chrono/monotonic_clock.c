#include <string.h>

#include <libpal/chrono/monotonic_clock.h>
#include <libpal/error.h>
#include <libpal/impl/chrono/monotonic_clock.h>
#include <libpal/require.h>
#include <pal_private.h>

static struct pal_impl_monotonic_clock monotonic_clock_impl = {
    NULL,
};

enum pal_error pal_set_impl_monotonic_clock(
    struct pal_impl_monotonic_clock *impl) {
  enum pal_error ret =
      pal_require(impl != NULL && impl->monotonic_clock_now != NULL

      );
  if (ret == PAL_SUCCESS) {
    monotonic_clock_impl = *impl;
  }
  return ret;
}

void pal_reset_impl_monotonic_clock() {
  memset(&monotonic_clock_impl, 0, sizeof(monotonic_clock_impl));
}

bool pal_has_impl_monotonic_clock(void) {
  return monotonic_clock_impl.monotonic_clock_now != NULL;
}

enum pal_error pal_monotonic_clock_now(uint64_t *now) {
  enum pal_error ret =
      pal_require(pal_has_impl_monotonic_clock() && now != NULL);
  if (ret == PAL_SUCCESS) {
    ret = monotonic_clock_impl.monotonic_clock_now(now);
  }
  return ret;
}
