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

#ifndef PVT_DRIVER_RUNNER_OBERVATIONCOMBINER_H
#define PVT_DRIVER_RUNNER_OBERVATIONCOMBINER_H

#include <limits>

#include <libsbp/v4/observation.h>
#include <pvt_common/observations.h>

namespace starling {
namespace util {

/**
 * @brief A simple struct to make the results of the `unpack_obs_header()`
 * function more ergonomic.
 */
struct ObservationHeader final {
 public:
  /**
   * @brief Constructor which unpacks the observation header into the
   * corresponding member variables.
   *
   * @param msg Header of a GNSS observation message.
   */
  explicit ObservationHeader(const sbp_observation_header_t &msg) noexcept;

  /**
   * @brief Time of reception of the GNSS observations associated with the
   * header.
   */
  gps_time_t reception_time = GPS_TIME_UNKNOWN;

  /**
   * @brief Total number of messages (size) in the sequence.
   */
  uint8_t total = 0U;

  /**
   * @brief Current message index in the sequence of messages.
   */
  uint8_t count = 0U;

  /**
   * @brief Checks to see if the header indicates that the message is the first
   * in the sequence.
   *
   * @return True if the message is indicated to be the first in the sequence,
   * false otherwise.
   */
  bool is_first() const noexcept;

  /**
   * @brief Checks to see if the time value in the header is valid, simply calls
   * `gps_time_valid()`.
   *
   * @return True if the time value is valid, false otherwise.
   */
  bool is_time_valid() const noexcept;
};

/**
 * @brief Coordinates the combination of multiple SBP observation messages into
 * a single observation array that starling can use.
 *
 * @details The entire recombining process is hidden, the user of this class
 * provides a location to unpack messages to, feeds messages until an
 * observation sequence is complete, and then releases the location from the
 * class and uses it as needed. Rinse and repeat from there.
 */
class ObservationCombiner final {
 public:
  /**
   * @brief Default constructor.
   */
  ObservationCombiner() noexcept = default;

  /**
   * @brief Sets a memory location to store the unpacked and combined
   * messages.
   *
   * @details Memory is not owned and the caller must make sure it is freed
   * in the end. A call to this when the combiner still has an output location
   * set causes the memory to be leaked.
   *
   * @param newLocation Location where objects of type obs_array_t are output.
   */
  void set_output_location(obs_array_t *newLocation) noexcept;

  /**
   * @brief Tells the combiner to release the memory to be managed elsewhere.
   *
   * @details The memory isn't freed, but ownership is released. A pointer to
   * the released memory is returned, so that someone else may free it.
   *
   * @return The output location previously set, or nullptr if not yet set.
   */
  obs_array_t *release_output_location() noexcept;

  /**
   * @brief Checks to see if the combiner has an output location set.
   *
   * @return True if there is a location set, false otherwise.
   */
  bool has_output_location() const noexcept;

  /**
   * @brief Attempts to process a SBP observation message and unpack its
   * content into the output location, combining it with any previously
   * received messages.
   *
   * @details The messages received typically come from the rover. This method
   * checks the consistency of the streamed messages.
   *
   * @param sender_id Identification of the sender.
   * @param msg SBP_MSG_OBS observation message.
   */
  void process_obs_message(uint16_t sender_id,
                           const sbp_msg_obs_t &msg) noexcept;

  /**
   * @brief Attempts to process a SBP observation message and unpack its
   * content into the output buffer, combining it with any previously received
   * messages.
   *
   * @details The messages received typically come from the base. This method
   * checks the consistency of the streamed messages.
   *
   * @param sender_id Identification of the sender.
   * @param msg SBP_MSG_OSR observation message.
   */
  void process_osr_message(uint16_t sender_id,
                           const sbp_msg_osr_t &msg) noexcept;

  /**
   * @brief Checks to see if the combiner has successfully completed processing
   * a sequence.
   *
   * @return True if a sequence has been successfully processed, false
   * otherwise.
   */
  bool is_complete() const noexcept;

  /**
   * @brief Checks to see if the combiner is currently in the progress of
   * combining a sequence.
   *
   * @return True if there is a sequence in progress, false otherwise.
   */
  bool is_in_progress() const noexcept;

  /**
   * @brief Gets the time of reception for the completed sequence.
   *
   * @details This function fails if `is_complete()` returns false.
   *
   * @param tor Pointer to the time of reception for the completed sequence.
   *
   * @return True and sets the value pointed to by `tor` upon success, upon
   * failure it returns false and sets the value pointed to by `tor` to
   * `GPS_TIME_UNKNOWN`
   */
  bool get_time(gps_time_t *tor) const noexcept;

  /**
   * @brief Resets the internal state of the object. Does NOT release the output
   * memory location.
   */
  void reset() noexcept;

 private:
  /**
   * @brief Checks the consistency of the streamed messages (that they have the
   * same sender_id, that the message counter increases appropriately).
   *
   * @param header Header of a GNSS observation message.
   * @param sender_id Identification of the sender.
   *
   * @return True if the message received is consistent, false otherwise.
   */
  bool update_and_validate_sequence(const ObservationHeader &header,
                                    const uint16_t sender_id) noexcept;

  /**
   * @brief Constant representing an invalid sequence.
   */
  static constexpr int32_t SEQUENCE_VALUE_INVALID =
      std::numeric_limits<int32_t>::max();

  /**
   * @brief Location where the unpacked and combined messages are stored.
   */
  obs_array_t *output_ = nullptr;
  /**
   * @brief Index of the final sequence message.
   */
  int32_t sequence_final_ = SEQUENCE_VALUE_INVALID;
  /**
   * @brief Index of the most recent sequence message.
   */
  int32_t sequence_most_recent_ = SEQUENCE_VALUE_INVALID;
  /**
   * @brief Identification of the sequence sender.
   */
  uint16_t sequence_sender_ = 0U;
  /**
   * @brief Time of reception for the sequence.
   */
  gps_time_t sequence_tor_ = GPS_TIME_UNKNOWN;
};

}  // namespace util
}  // namespace starling

#endif  // PVT_DRIVER_RUNNER_OBERVATIONCOMBINER_H
