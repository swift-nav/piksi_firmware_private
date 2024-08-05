/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_STACK_SIZE_INCREASE_H
#define LIBSWIFTNAV_STACK_SIZE_INCREASE_H

#include <sys/resource.h>

// Class which has the sole purpose of increasing stack size so that we don't
// crash
class Init {
 public:
  Init() {
    const rlim_t kStackSize = 80 * 1024L * 1024L;  // min stack size = 80 Mb
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0) {
      if (rl.rlim_cur < kStackSize) {
        rl.rlim_cur = kStackSize;
        if (kStackSize > rl.rlim_max) {
          fprintf(stderr, "Max stack size is limited to %llu bytes\n",
                  static_cast<unsigned long long>(rl.rlim_max));
          rl.rlim_cur = rl.rlim_max - 1;
        }

        result = setrlimit(RLIMIT_STACK, &rl);
        if (result != 0) {
          fprintf(stderr, "setrlimit returned result = %d\n", result);
        }
      }
    }
  }
};

// Needs to be a global variable so that the constructor is called first and the
// stack size increased before it is used by other instantiations
Init my_init __attribute__((unused));

#endif  // LIBSWIFTNAV_STACK_SIZE_INCREASE_H
