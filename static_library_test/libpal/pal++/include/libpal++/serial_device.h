#ifndef LIBPAL_CPP_SERIAL_DEVICE_H
#define LIBPAL_CPP_SERIAL_DEVICE_H

#include <libpal++/io_device.h>

#include <utility>

namespace pal {
/**
 * Provides the IO interface for accessing serial devices.
 */
class SerialDevice final : public IODevice {
 public:
  SerialDevice() noexcept : IODevice(), pal_handle_() {}

  SerialDevice(const SerialDevice &other) = delete;
  SerialDevice(SerialDevice &&other) noexcept
      : IODevice(std::move(other)), pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  SerialDevice &operator=(const SerialDevice &other) = delete;
  SerialDevice &operator=(SerialDevice &&other) noexcept {
    IODevice::operator=(std::move(other));
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~SerialDevice() override { close(); }

  /**
   * Attempts to open the device. If the device successfully opens, caller
   * may then proceed with reading and/or writing to device.
   *
   * An error will be returned if an attempt is made to open an already opened
   * serial device.
   *
   * @param path
   * @param mode
   * @param brate
   * @param bsize
   * @param parity
   * @param stop
   * @param rts
   *
   * @return platform error code
   */
  pal_error open(const char *path, pal_access_mode mode, int brate,
                 uint8_t bsize, char parity, uint8_t stop, bool rts) noexcept;

  pal_error close() noexcept override;
  bool eof() noexcept override;
  bool is_open() const noexcept override { return pal_handle_ != nullptr; }

  /**
   * Flushes any buffered data to the serial device.
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

 private:
  pal_serial_t pal_handle_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_SERIAL_DEVICE_H
