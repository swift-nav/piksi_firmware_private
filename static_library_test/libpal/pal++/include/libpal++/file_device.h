#ifndef LIBPAL_CPP_FILE_DEVICE_H
#define LIBPAL_CPP_FILE_DEVICE_H

#include <libpal++/io_device.h>

#include <utility>

namespace pal {
/**
 * Provides the IO interface for accessing file within platform's
 * filesystem.
 */
class FileDevice final : public IODevice {
 public:
  FileDevice() noexcept : IODevice(), pal_handle_() {}

  FileDevice(const FileDevice &other) = delete;
  FileDevice(FileDevice &&other) noexcept
      : IODevice(std::move(other)), pal_handle_() {
    std::swap(pal_handle_, other.pal_handle_);
  }

  FileDevice &operator=(const FileDevice &other) = delete;
  FileDevice &operator=(FileDevice &&other) noexcept {
    IODevice::operator=(std::move(other));
    std::swap(pal_handle_, other.pal_handle_);
    return *this;
  }

  ~FileDevice() override { close(); }

  /**
   * Attempts to open the file with the specified mode. If the file
   * successfully opens, caller may then proceed with reading and/or writing
   * to file.
   *
   * An error will be returned if an attempt is made to open an already opened
   * file.
   *
   * @param path filesystem path to the file
   * @param mode specifies the manner in which the file is to be opened
   *
   * @return platform error code
   */
  pal_error open(const char *path, pal_access_mode mode) noexcept;

  pal_error close() noexcept override;
  bool eof() noexcept override;
  bool is_open() const noexcept override { return pal_handle_ != nullptr; }

  /**
   * Flushes any buffered data to the file.
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
  pal_file_t pal_handle_;
};
}  // namespace pal

#endif  // LIBPAL_CPP_FILE_DEVICE_H
