#ifndef LIBPAL_IMPL_IO_ACCESS_MODE_H
#define LIBPAL_IMPL_IO_ACCESS_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * IO device access mode. To be specified when opening an IO device for the
 * first time
 */
enum pal_access_mode {
  /**
   * Invalid, do not use
   */
  PAL_ACCESS_INVALID,

  /**
   * Open in read only mode. Write operations will return an error
   */
  PAL_ACCESS_RDONLY,

  /**
   * Open in write only mode. Read operations will return an error
   */
  PAL_ACCESS_WRONLY,

  /**
   * Open in read/write mode. Calls to pal_io_read and pal_io_write are valid
   * for this device
   */
  PAL_ACCESS_RDWR,
};

static inline bool pal_validate_access_mode(enum pal_access_mode mode) {
  switch (mode) {
    case PAL_ACCESS_RDONLY:
    case PAL_ACCESS_WRONLY:
    case PAL_ACCESS_RDWR:
      return true;
    case PAL_ACCESS_INVALID:
    default:
      return false;
  }
}

#ifdef __cplusplus
}
#endif

#endif
