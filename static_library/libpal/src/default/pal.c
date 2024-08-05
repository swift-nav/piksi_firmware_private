#include <libpal/impl/impl.h>
#include <pal_private.h>
#include <stdbool.h>

static bool pal_initialized = false;

void pal_init(void) {
  if (!pal_initialized) {
    pal_impl_init();
    pal_initialized = true;
  }
}

void pal_deinit(void) {
  if (!pal_initialized) {
    return;
  }
  pal_impl_deinit();
  pal_reset_impl_io();
  pal_reset_impl_file();
  pal_reset_impl_tcp();
  pal_reset_impl_serial();
  pal_reset_impl_stdstream();
  pal_reset_impl_mem();
  pal_reset_impl_mq();
  pal_reset_impl_mutex();
  pal_reset_impl_cv();
  pal_reset_impl_thread();
  pal_reset_impl_monotonic_clock();
  pal_initialized = false;
}
