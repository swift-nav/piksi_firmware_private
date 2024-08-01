#ifndef STARLING_UTIL_URI_H
#define STARLING_UTIL_URI_H

#include <mapbox/variant.hpp>

#include <cstddef>
#include <cstdint>

namespace starling {
namespace util {
/**
 * Offers the ability to parse a Swift Navigation URI down to its two
 * components:
 *
 *  - protocol
 *  - endpoint
 *
 * Class has been developed based on the parse_uri implementation in
 * starling-runner without its dependency to ContextBuilderDevice and no dynamic
 * memory allocation. Because there can be no dynamic memory allocation, one
 * must be careful to remember that:
 *
 *  1. all const char* refer to sub sections of the passed in uri
 *  2. all const char* strings are not NULL terminated
 *
 * It would have been best if we could use std::string_view class to develop the
 * class, however that isn't currently possible.
 *
 * Below is a small code snippet to illustrate how to use the class:
 *
 * \code
 * starling::util::Uri uri;
 * bool success = starling::util::Uri::parse("sbp://tcpcli:192.168.1.2:5002",
 * &uri);
 *
 * assert(uri.protocol == starling::util::Uri::Protocol::SBP);
 * assert(uri.endpoint.is<starling::util::Uri::TcpClient>());
 *
 * starling::util::Uri::TcpClient& tcp_client =
 * uri.endpoint.get<starling::util::Uri::TcpClient>();
 *
 * assert(tcp_client.hostname_len == 11);
 * assert(memcmp(tcp_client.hostname, "192.168.1.2", tcp_client.hostname_len) ==
 * 0); assert(tcp_client.port == 5002); \endcode
 */
class Uri {
 public:
  enum class Protocol { SBP, RTCM };

  struct FileRead {
    const char *path;
    size_t path_len;
  };

  struct FileWrite {
    const char *path;
    size_t path_len;
  };

  struct TcpClient {
    const char *hostname;
    size_t hostname_len;
    uint16_t port;
  };

  using Endpoint = mapbox::util::variant<FileRead, FileWrite, TcpClient>;

 public:
  static bool parse(const char *uri, Uri *obj);

 public:
  Uri() = default;

 public:
  Protocol protocol;
  Endpoint endpoint;
};
}  // namespace util
}  // namespace starling

#endif  // STARLING_UTIL_URI_H