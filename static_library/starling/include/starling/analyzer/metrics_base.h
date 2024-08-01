///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
///////////////////////////////////////////////////////////////////////////////

#ifndef STARLING_METRICS_BASE_H
#define STARLING_METRICS_BASE_H

namespace starling {
namespace analyzer {

// a bit silly but makes us remember the units in use...
constexpr double MS_PER_S = 1000.;
constexpr double MM_PER_M = 1000.;

class MetricsBase {
 public:
  virtual ~MetricsBase(){};

  virtual const int whoami() { return 0; }
};
}  // namespace analyzer
}  // namespace starling
#endif  // STARLING_METRICS_BASE_H
