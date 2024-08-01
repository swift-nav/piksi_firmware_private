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

#ifndef SENSORFUSION_INTEGRATION_DR_APPLICATION_H_
#define SENSORFUSION_INTEGRATION_DR_APPLICATION_H_

//------------------------------------------------------------------------------
// About:
//
// Convenience object which wraps up several common use-cases of the
// sensorfusion library.
//
// A note: there are several raw pointer arguments. This is because
// we are not really intending to interact with dynamic objects. You
// manage the lifetime outside of this class.
//------------------------------------------------------------------------------

#include <libpal++/io_device.h>
#include <libsbp/cpp/state.h>

#include <memory>

#include "sensorfusion/core/error_types.h"
#include "sensorfusion/integration/sbp_io.h"

namespace sensorfusion {
namespace deadreckoning {

class Application {
 public:
  static Application *instance() { return instance_; }

 public:
  Application();
  ~Application();

  // The application will attempt various configuration strategies.
  MaybeError Configure();

  // The application will attempt to configure from the given file.
  MaybeError Configure(const char *yamlfile);

  // Reference to the Output SBP state.
  sbp::State &OutputSBP();

  // Connect an output destination for SBP.
  MaybeError ConnectOutput(pal::IODevice *output);

  // Connect an input source of SBP.
  MaybeError ConnectInput(pal::IODevice *input);

  // Run and block the current thread until complete.
  MaybeError Run();

  // TODO(STAR-964) This method is deprecated.
  // Connect an output destination for SBP.
  MaybeError ConnectOutput(io::sbp::Output *output);

  // TODO(STAR-964) This method is deprecated.
  // Connect an input source of SBP.
  MaybeError ConnectInput(io::sbp::Input *input);

  // TODO(STAR-964) This method is deprecated.
  // Run and block the current thread until complete.
  MaybeError RunLegacy();

 private:
  static Application *instance_;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace deadreckoning
}  // namespace sensorfusion

#endif
