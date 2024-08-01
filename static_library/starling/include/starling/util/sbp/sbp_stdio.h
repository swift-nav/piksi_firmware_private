//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the licence found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#include <iomanip>
#include <iostream>

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>
#include <pvt_engine/optional.h>
#include <starling/util/sbp/unpackers.h>
#include <swiftnav/gnss_time.h>

#ifndef STARLING_SBP_STDIO_H
#define STARLING_SBP_STDIO_H

class SbpStreamReader : public sbp::IReader {
 public:
  explicit SbpStreamReader(std::istream &in_stream) : in_stream_(in_stream) {}

  s32 read(u8 *buffer, u32 buffer_length) override {
    if (in_stream_.read(reinterpret_cast<char *>(buffer),  // NOLINT
                        buffer_length)) {                  // NOLINT
      return static_cast<s32>(in_stream_.gcount());        // NOLINT
    } else if (in_stream_.fail()) {                        // NOLINT
      return -1;
    } else {          // NOLINT
      assert(false);  // something's wrong with the stream
    }
  }

 private:
  std::istream &in_stream_;
};

class SbpStreamWriter : public sbp::IWriter {
 public:
  explicit SbpStreamWriter(std::ostream &out_stream)
      : out_stream_(out_stream) {}

  s32 write(const u8 *buffer, u32 buffer_length) override {
    if (out_stream_.write(reinterpret_cast<const char *>(buffer),  // NOLINT
                          buffer_length)) {
      return static_cast<s32>(buffer_length);  // NOLINT
    } else if (out_stream_.fail()) {           // NOLINT
      return -1;
    } else {          // NOLINT
      assert(false);  // something's wrong with the stream
    }
  }

 private:
  std::ostream &out_stream_;
};

#endif  // STARLING_SBP_STDIO_H
