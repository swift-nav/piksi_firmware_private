#ifndef LIBPAL_CPP_STD_STREAM_DEVICE_H
#define LIBPAL_CPP_STD_STREAM_DEVICE_H

#include <libpal++/io_device.h>
#include <libpal/impl/io/stdstream.h>
#include <libpal/io/stdstream.h>

namespace pal {

class StdIODeviceIn final : public IODevice {
 public:
  pal_error close() noexcept override { return PAL_SUCCESS; }
  bool eof() noexcept override;
  bool is_open() const noexcept override { return true; }

 protected:
  pal_error read_impl(uint8_t *buffer, std::size_t max_count,
                      std::size_t *read_count, pal_blocking_mode mode,
                      uint64_t timeout_us) noexcept override;
  pal_error write_impl(const uint8_t *buffer, std::size_t max_count,
                       std::size_t *written_count, pal_blocking_mode mode,
                       uint64_t timeout_us) noexcept override;
};

class StdIODeviceOut final : public IODevice {
 public:
  pal_error close() noexcept override { return PAL_SUCCESS; }
  bool eof() noexcept override { return false; }
  bool is_open() const noexcept override { return true; }

  /**
   * Flushes any buffered data to the stream.
   *
   * @return platform error code
   */
  pal_error flush() noexcept;

 protected:
  pal_error read_impl(uint8_t *buffer, std::size_t max_count,
                      std::size_t *read_count, pal_blocking_mode mode,
                      uint64_t timeout_us) noexcept override;
  pal_error write_impl(const uint8_t *buffer, std::size_t max_count,
                       std::size_t *written_count, pal_blocking_mode mode,
                       uint64_t timeout_us) noexcept override;
};

class StdIODeviceErr final : public IODevice {
 public:
  pal_error close() noexcept override { return PAL_SUCCESS; }
  bool eof() noexcept override { return false; }
  bool is_open() const noexcept override { return true; }

  /**
   * Flushes any buffered data to the stream.
   *
   * @return platform error code
   */
  pal_error flush() noexcept;

 protected:
  pal_error read_impl(uint8_t *buffer, std::size_t max_count,
                      std::size_t *read_count, pal_blocking_mode mode,
                      uint64_t timeout_us) noexcept override;
  pal_error write_impl(const uint8_t *buffer, std::size_t max_count,
                       std::size_t *written_count, pal_blocking_mode mode,
                       uint64_t timeout_us) noexcept override;
};

static StdIODeviceIn std_input;
static StdIODeviceOut std_output;
static StdIODeviceErr std_error;
}  // namespace pal

#endif  // LIBPAL_CPP_STD_STREAM_DEVICE_H
