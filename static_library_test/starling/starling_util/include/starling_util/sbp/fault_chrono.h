//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#ifndef STARLING_FAULT_CHRONO_H
#define STARLING_FAULT_CHRONO_H

#include <swiftnav/gnss_time.h>

namespace starling {
namespace fault_injector {

//---------------------------------------------
// Interface for basic timer for fault injector
//---------------------------------------------
class FaultChrono {
 protected:
  FaultChrono() = default;

 public:
  //--------------------------------------------------------------------------
  virtual ~FaultChrono() = default;

  //--------------------------------------------------------------------------
  // Should be called for each incoming GPS_TIME
  virtual void handle_tick(const gps_time_t &current_gnss_time) = 0;
};
}  // namespace fault_injector
}  // namespace starling

#endif  // STARLING_FAULT_CHRONO_H
