#ifndef LIBPAL_CPP_VALIDATION_H
#define LIBPAL_CPP_VALIDATION_H

#include <cassert>
#include <utility>

#include <libpal/error.h>
#include <libpal/pal.h>

namespace pal {
namespace detail {

template <typename T>
inline void init_output_parameter(T *p, const T &v) {
  if (p != nullptr) {
    *p = v;
  }
}

}  // namespace detail
}  // namespace pal

#endif
