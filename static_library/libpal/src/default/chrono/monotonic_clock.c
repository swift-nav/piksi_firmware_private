#include <libpal/chrono/monotonic_clock.h>
#include <libpal/error.h>
#include <libpal/impl/chrono/monotonic_clock.h>
#include <pal_private.h>
#include <stddef.h>

#include "not_implemented.h"

static enum pal_error default_monotonic_clock_now(uint64_t *now) {
  NOT_IMPLEMENTED();
  (void)now;
  return PAL_INVALID;
}

static struct pal_impl_monotonic_clock monotonic_clock_impl = {
    .monotonic_clock_now = default_monotonic_clock_now};

void pal_set_impl_monotonic_clock(struct pal_impl_monotonic_clock *impl) {
  assert(impl != NULL);
  assert(impl->monotonic_clock_now != NULL);

  monotonic_clock_impl = *impl;
}

void pal_reset_impl_monotonic_clock() {
  struct pal_impl_monotonic_clock impl = {
      .monotonic_clock_now = default_monotonic_clock_now,
  };
  pal_set_impl_monotonic_clock(&impl);
}

bool pal_has_impl_monotonic_clock(void) {
  return monotonic_clock_impl.monotonic_clock_now !=
         default_monotonic_clock_now;
}

enum pal_error pal_monotonic_clock_now(uint64_t *now) {
  return monotonic_clock_impl.monotonic_clock_now(now);
}
