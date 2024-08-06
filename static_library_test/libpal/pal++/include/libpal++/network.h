#ifndef LIBPAL_CPP_NETWORK_H
#define LIBPAL_CPP_NETWORK_H

#include <cassert>
#include <cstdint>

#include <libpal/impl/io/network.h>

namespace pal {
namespace Ip {
enum class Version { ANY, V4, V6 };

inline Version convert(pal_ip_version version) noexcept {
  switch (version) {
    case PAL_IP_ANY:
      return Version::ANY;
    case PAL_IP_V4:
      return Version::V4;
    case PAL_IP_V6:
      return Version::V6;
    default:
      assert(false && "Invalid libpal IP Version");
      return Version::ANY;
  }
}

inline pal_ip_version convert(Version version) noexcept {
  switch (version) {
    case Version::ANY:
      return PAL_IP_ANY;
    case Version::V4:
      return PAL_IP_V4;
    case Version::V6:
      return PAL_IP_V6;
    default:
      assert(false && "Invalid libpal++ IP Version");
      return PAL_IP_ANY;
  }
}

inline bool valid(Version version) noexcept {
  switch (version) {
    case Version::ANY:
    case Version::V4:
    case Version::V6:
      return true;
    default:
      return false;
  }
}

/**
 * Represents an IP Endpoint (aka IP address, port, version)
 */
struct Endpoint {
  // able to hold c-string of any IPv4 or IPv6 address
  static constexpr size_t kAddressSize = 46;

  char address[kAddressSize];
  uint16_t port;
  Version version;
};
}  // namespace Ip
}  // namespace pal

#endif  // LIBPAL_CPP_NETWORK_H
