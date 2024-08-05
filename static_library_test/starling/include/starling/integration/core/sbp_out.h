#ifndef STARLING_INTEGRATION_CORE_SBP_OUT_H
#define STARLING_INTEGRATION_CORE_SBP_OUT_H

#include <libsbp/sbp.h>

namespace starling {
namespace integration {

using send_msg_t = s8 (*)(u16 msg_id, u8 len, u8 *buff);
using send_msg_with_sender_t = s8 (*)(u16 msg_id, u16 sender_id, u8 len,
                                      u8 *buff);

/**
 * Install callbacks to allow starling to output SBP messages
 *
 * These callbacks will be invoked everytime starling integration outputs
 * an SBP formatted message. The callbacks must be set up before starling
 * is started so to allow it to output messages. Running starling via the
 * integration layer without providing these callbacks will result in an
 * assert
 *
 * @param send Callback function to send an SBP encoded message
 * @param send_with_sender Callback function to send an SBP encoded message with
 * sender id parameter
 */
void setup_sbp_out(send_msg_t send, send_msg_with_sender_t send_with_sender);

}  // namespace integration
}  // namespace starling

#endif
