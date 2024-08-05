#ifndef LIBPAL_DEFAULT_PAL_PRIVATE_H
#define LIBPAL_DEFAULT_PAL_PRIVATE_H

#include <libpal/impl/impl.h>
#include <stdbool.h>

void pal_reset_impl_io();

void pal_reset_impl_file();

void pal_reset_impl_tcp();

void pal_reset_impl_serial();

void pal_reset_impl_stdstream();

void pal_reset_impl_mem();

void pal_reset_impl_mq();

void pal_reset_impl_mutex();

void pal_reset_impl_cv();

void pal_reset_impl_thread();

void pal_reset_impl_monotonic_clock();

void pal_reset_impl_watchdog();

#endif
