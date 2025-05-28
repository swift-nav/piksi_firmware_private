#ifndef LIBPAL_IMPL_IO_NETWORK_H
#define LIBPAL_IMPL_IO_NETWORK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// PAL IP protocol versions
enum pal_ip_version {
  // Use any available IP version
  PAL_IP_ANY,
  // Use only IP version 4
  PAL_IP_V4,
  // Use only IP version 6
  PAL_IP_V6,
};

static inline bool pal_validate_ip_version(enum pal_ip_version version) {
  switch (version) {
    case PAL_IP_ANY:
    case PAL_IP_V4:
    case PAL_IP_V6:
      return true;

    default:
      return false;
  }
}

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIBPAL_IMPL_IO_NETWORK_H
