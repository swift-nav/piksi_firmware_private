/*
 * Copyright (C) 2015 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_DECODE_H
#define SWIFTNAV_DECODE_H

#include <libswiftnav/common.h>
#include <libswiftnav/signal.h>

/** \addtogroup decoding
 * \{ */

/** Decoder interface function template. */
typedef void (*decoder_interface_function_t)(
    const u8 *channel_id, void *decoder_data);

/** Interface to a decoder implementation. */
typedef struct {
  bool busy;
  u8 channel_id;
  code_t code;
  decoder_interface_function_t init;
  decoder_interface_function_t process;
  decoder_interface_function_t disable;
  void *decoder_data;
} decoder_interface_t;

/** \} */

#ifdef __cplusplus
extern "C" {
#endif

void decode_setup(void);
void decoder_interface_register(const u8 channel_id, decoder_interface_t *element);

bool decoder_channel_available(u8 tracking_channel,
                               const me_gnss_signal_t mesid);
bool decoder_channel_init(u8 tracking_channel, const me_gnss_signal_t mesid);
bool decoder_channel_disable(u8 tracking_channel);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_DECODE_H */
