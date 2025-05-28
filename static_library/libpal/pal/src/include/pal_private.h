#ifndef LIBPAL_DEFAULT_PAL_PRIVATE_H
#define LIBPAL_DEFAULT_PAL_PRIVATE_H

#include <assert.h>
#include <libpal/impl/impl.h>
#include <libpal/pal.h>

#define INIT_OUTPUT_PARAM(p, v) \
  if ((p) != NULL) {            \
    *(p) = v;                   \
  }

void pal_reset_impl_file();

void pal_reset_impl_tcp();

void pal_reset_impl_udp();

void pal_reset_impl_serial();

void pal_reset_impl_stdstream();

void pal_reset_impl_mem();

void pal_reset_impl_mq();

void pal_reset_impl_mutex();

void pal_reset_impl_cv();

void pal_reset_impl_thread();

void pal_reset_impl_monotonic_clock();

void pal_reset_impl_watchdog();

void pal_reset_impl_identifier();

#endif
