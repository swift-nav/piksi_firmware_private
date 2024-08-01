#ifndef SENSORFUSION_INTEGRATION_SBP_IO_H_
#define SENSORFUSION_INTEGRATION_SBP_IO_H_

//------------------------------------------------------------------------------
// About:
//
// Interfaces used when connecting SBP into other high-level components.
// You will typically provide these as a client of the sensorfusion software.
//------------------------------------------------------------------------------

#include <cstdint>

#include "sensorfusion/core/error_types.h"

namespace sensorfusion {
namespace io {
namespace sbp {

//------------------------------------------------------------------------------
/// SBP sender ID used for sensorfusion
constexpr uint16_t kSenderId = 789;

//------------------------------------------------------------------------------
// Interface for an object which acts as a provider of SBP messages.
class Input {
 public:
  virtual ~Input() = default;

  // Callback type used by this interface.
  struct CallbackInfo {
    uint16_t msg_id;
    void *context;
    void (*cb)(uint16_t id, uint8_t len, uint8_t *payload, void *context);
  };

  // Register callback for a given message type.
  virtual MaybeError RegisterCallback(const CallbackInfo &cbinfo) = 0;

  // Allow the input to take-over the current thread and begin "sourcing"
  // sbp messages.
  virtual MaybeError Run() = 0;
};

//------------------------------------------------------------------------------
// Interface for an object which acts as a sender of SBP messages.
class Output {
 public:
  virtual ~Output() = default;

  // Send a message.
  virtual MaybeError SendMessage(uint16_t id, uint8_t len,
                                 const uint8_t *buf) = 0;
  virtual MaybeError SendMessage(uint16_t id, uint8_t len, const uint8_t *buf,
                                 uint16_t sender) = 0;
};

}  // namespace sbp
}  // namespace io
}  // namespace sensorfusion

#endif
