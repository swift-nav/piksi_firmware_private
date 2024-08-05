/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_UTIL_SBP_OBERVATIONCOMBINER_H
#define STARLING_UTIL_SBP_OBERVATIONCOMBINER_H

#include "libsbp/observation.h"
#include "starling/observation.h"

namespace starling {
namespace util {
namespace sbp {

// A simple struct to make the results of the `unpack_obs_header()` function
// more ergonomic
struct ObservationHeader {
 public:
  // Constructor simply calls `unpack_obs_header()` including all members
  explicit ObservationHeader(const observation_header_t &msg);

  gps_time_t time;
  u8 total;
  u8 count;

  // Checks to see if the header indicates that the message is the first in the
  // sequence
  bool is_first() const;
  // Checks to see if the time value in the header is valid, simply calls
  // `gps_time_valid()`
  bool is_time_valid() const;
};

// A class that coordinates combining multiple SBP observation messages into a
// single observation array that starling can use.
// The entire recombining process is hidden, the user of this class simply gives
// it memory to unpack messages into, gives it messages until it says it's
// complete, and then releases the buffer from the class and use it as needed.
// Rinse and repeat from there.
class ObservationCombiner {
 public:
  ObservationCombiner();

  // Gives the combiner memory to store the unpacked and combined messages
  // Note: calling this when the combiner still has an output location set will
  // cause the memory to be leaked
  void set_output_location(obs_array_t *newLocation);

  // Tells the combiner to release the memory to be managed elsewhere. The
  // memory isn't freed, but ownership is released. A pointer to the released
  // memory is returned, so that someone else may free it.
  obs_array_t *release_output_location();

  // Checks to see if the combiner has an output location set
  bool has_output_location() const;

  // Attempts to processes a SBP observation message and unpack it's contents
  // into the output buffer
  void process_obs_message(uint16_t sender_id, uint8_t len, uint8_t *bytes);
  void process_osr_message(uint16_t sender_id, uint8_t len, uint8_t *bytes);

  // Checks to see if the combiner has successfully completed processing a
  // sequence
  bool is_complete() const;

  // Gets the time of resception for the completed sequence. Note: this function
  // will fail if `is_complete()` returns `false`.
  // Returns `true` and sets the value pointed to by `tor` upon success,
  // upon failure it returns `false` and sets the value pointed to by `tor` to
  // `GPS_TIME_UNKNOWN`
  bool get_time(gps_time_t *tor) const;

  // Resets the internal state of the object. Does NOT release the output memory
  // location.
  void reset();

 private:
  bool update_and_validate_sequence(const ObservationHeader &header,
                                    uint16_t sender_id);

  obs_array_t *output_;
  int sequence_final_;
  int sequence_most_recent_;
  uint16_t sequence_sender_;
  gps_time_t sequence_tor_;
};

}  // namespace sbp
}  // namespace util
}  // namespace starling

#endif  // STARLING_UTIL_SBP_OBERVATIONCOMBINER_H
