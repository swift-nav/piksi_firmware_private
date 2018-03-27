#include <stdio.h>
#include <string.h>
#include <cmath>

#include <libswiftnav/logging.h>

#include "gtest/gtest.h"
#include "nav_msg/nav_msg_glo.h"

#undef log_error
#define log_error(...)

#define LOW_TOL 1e-6
#define HIGH_TOL 1e-1
