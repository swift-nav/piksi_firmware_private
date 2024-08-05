#ifndef NOT_IMPLEMENTED_H
#define NOT_IMPLEMENTED_H

#include <assert.h>

#define NOT_IMPLEMENTED()           \
  do {                              \
    assert(0 && "NOT IMPLEMENTED"); \
  } while (0)

#endif
