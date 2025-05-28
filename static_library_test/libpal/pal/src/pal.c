#include <libpal/impl/impl.h>
#include <pal_private.h>
#include <stdbool.h>

static size_t pal_init_count = 0;
static bool pal_init_completed = false;

static bool pal_initialized(void) { return pal_init_count > 0; }
static void reset_all(void) {
  pal_reset_impl_file();
  pal_reset_impl_tcp();
  pal_reset_impl_udp();
  pal_reset_impl_serial();
  pal_reset_impl_stdstream();
  pal_reset_impl_mem();
  pal_reset_impl_mq();
  pal_reset_impl_mutex();
  pal_reset_impl_cv();
  pal_reset_impl_thread();
  pal_reset_impl_monotonic_clock();
  pal_reset_impl_watchdog();
  pal_reset_impl_identifier();
}

enum pal_error pal_init(void) {
  enum pal_error ret = PAL_SUCCESS;
  if (!pal_initialized()) {
    ret = pal_impl_init();
  }
  if (ret == PAL_SUCCESS) {
    pal_init_count++;
  } else {
    reset_all();
  }
  return ret;
}

enum pal_error pal_deinit(bool force) {
  if (!pal_initialized()) {
    return PAL_SUCCESS;
  }
  if (force) {
    // should there be some kind of warning here?
    pal_init_count =
        0;  // force init count to zero ignoring further deinit calls
  } else {
    pal_init_count--;
  }
  if (pal_init_count > 0) {
    return PAL_SUCCESS;
  }
  enum pal_error ret = pal_impl_deinit();
  reset_all();
  pal_init_completed = false;
  return ret;
}

enum pal_error pal_init_complete(void) {
  enum pal_error ret = PAL_SUCCESS;
  if (!pal_initialized()) {
    return ret;
  }
  if (pal_init_completed) {
    return ret;
  }
  ret = pal_impl_init_complete();
  pal_init_completed = true;
  return ret;
}
