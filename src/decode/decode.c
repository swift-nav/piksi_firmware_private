/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode.h"
#include <assert.h>
#include <ch.h>
#include <libswiftnav/glo_map.h>
#include <libswiftnav/logging.h>
#include "board/v3/platform_signal.h"
#include "signal_db/signal_db.h"
#include "track/track_decode.h"

#define DECODE_THREAD_STACK (4 * 1024)
#define DECODE_THREAD_PRIORITY (NORMALPRIO - 1)

static decoder_interface_t const *decoder_interface[NUM_DECODER_CHANNELS];

static THD_WORKING_AREA(wa_decode_thread, DECODE_THREAD_STACK);

static void decode_thread(void *arg);
static const decoder_interface_t *decoder_interface_get(
    const me_gnss_signal_t mesid);
static decoder_channel_t *decoder_channel_get(u8 tracking_channel);
static bool available_decoder_get(const decoder_interface_t *interface,
                                  void **decoder);

static void interface_function(decoder_channel_t *d,
                               decoder_interface_function_t func);

static const decoder_interface_t decoder_interface_default = {
    .busy = false,
    .code = CODE_INVALID,
    .init = NULL,
    .process = NULL,
    .disable = NULL,
    .decoder_data = NULL};

/** Set up the decoding module. */
void decode_setup(void) {
  platform_decode_setup();

  chThdCreateStatic(wa_decode_thread,
                    sizeof(wa_decode_thread),
                    DECODE_THREAD_PRIORITY,
                    decode_thread,
                    NULL);
}

/** Register a decoder interface to enable decoding for a constellation / band.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void decoder_interface_register(const u8 channel_id, decoder_interface_t *element) {
  /* p_next = address of next pointer which must be updated */
  decoder_interface[channel_id] = element;
}

/** Determine if a decoder channel is available for the specified tracking
 * channel and ME sid.
 *
 * \param channel_id  Tracking channel to use.
 * \param mesid       ME signal to be decoded.
 *
 * \return true if a decoder channel is available, false otherwise.
 */
bool decoder_channel_available(u8 channel_id, const me_gnss_signal_t mesid) {
  decoder_channel_t *d = decoder_channel_get(channel_id);
  if (d->busy) {
    log_info_mesid(
        mesid,
        "decoder_channel_available() channel_id %d is busy",
        channel_id);
    return false;
  }

  const decoder_interface_t *interface = decoder_interface_get(mesid);
  if (!available_decoder_get(interface, NULL)) {
    log_info_mesid(mesid, "no available decoder for the interface");
    return false;
  }

  return true;
}

/** Initialize a decoder channel to process telemetry for ME sid from the
 * specified tracking channel.
 *
 * \param tracking_channel  Tracking channel to use.
 * \param mesid             ME signal to be decoded.
 *
 * \return true if a decoder channel was initialized, false otherwise.
 */
bool decoder_channel_init(u8 channel_id, const me_gnss_signal_t mesid) {
  decoder_channel_t *d = decoder_channel_get(channel_id);
  if (d->busy) {
    log_error_mesid(mesid, "decoder_channel_init() channel_id %d is busy", channel_id);
    return false;
  }

  const decoder_interface_t *interface = decoder_interface_get(mesid);
  void *decoder;
  if (!available_decoder_get(interface, &decoder)) {
    log_error_mesid(mesid, "!available_decoder_get()");
    return false;
  }

  /* Set up channel */
  d->info.channel_id = channel_id;
  d->info.mesid = mesid;
  d->decoder = decoder;

  event(d, EVENT_ENABLE_REQUEST);
  return true;
}

/** Disable the decoder channel associated with the specified
 * tracking channel.
 *
 * \param channel_id  Tracking channel to use.
 *
 * \return true if a decoder channel was disabled, false otherwise.
 */
bool decoder_channel_disable(u8 channel_id) {
  decoder_channel_t *d = decoder_channel_get(channel_id);
  const decoder_interface_t *interface = decoder_interface_get(d->info.mesid);
  interface_function(d, interface->disable);

  /* Request disable */
  event(d, EVENT_DISABLE);
  return true;
}

/** Flush the navigation data FIFO
 *
 * \param channel_id  Tracking channel to use.
 */
static void nav_fifo_flush(const u8 channel_id) {
  while (tracker_nav_bit_get(channel_id, NULL)) {
    ;
  }
}

/** Decoder thread
 */
static void decode_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("decode");

  while (true) {
    for (u8 i = 0; i < NUM_DECODER_CHANNELS; i++) {
      decoder_channel_t *d = &decoder_channels[i];
      const decoder_interface_t *interface =
          decoder_interface_get(d->info.mesid);

      if (d->busy) {
        interface_function(d, interface->process);
      }

      /*switch (decoder_channel_state_get(d)) {
        case DECODER_CHANNEL_STATE_ENABLE_REQUESTED: {
           Empty the nav bit FIFO
          nav_fifo_flush(d->info.channel_id);
          interface_function(d, interface->init);
          event(d, EVENT_ENABLE);
        } break;

        case DECODER_CHANNEL_STATE_ENABLED: {
          interface_function(d, interface->process);
        } break;

        case DECODER_CHANNEL_STATE_DISABLED:
           Do nothing
          break;

        default:
          assert(!"Invalid state");
          break;
      }*/
    }

    chThdSleep(MS2ST(MAX_NAV_BIT_LATENCY_MS/20));
  }
}

/** Retrieve the decoder interface for the specified sid.
 *
 * \param mesid ME signal to be decoded.
 *
 * \return Associated decoder interface. May be the default interface.
 */
static const decoder_interface_t *decoder_interface_get(
    const me_gnss_signal_t mesid) {
  const decoder_interface_t *e = decoder_interface[mesid.code];

  if (NULL == e) {
    return &decoder_interface_default;
  }

  return e;
}

/** Retrieve the decoder channel associated with the specified tracking channel.
 *
 * \param channel_id  Tracking channel to use.
 *
 * \return Associated decoder channel.
 */
static decoder_channel_t *decoder_channel_get(u8 channel_id) {
  /* TODO: Decouple tracking / decoder channels somewhat.
   * Just need to make sure that only a single decoder channel can be allocated
   * to a given tracking channel.
   */
  assert(channel_id < NUM_DECODER_CHANNELS);
  return &decoder_channels[channel_id];
}

/** Find an inactive decoder instance for the specified decoder interface.
 *
 * \param interface       Decoder interface to use.
 * \param decoder         Output inactive decoder instance.
 *
 * \return true if *decoder points to an inactive decoder instance,
 * false otherwise.
 */
static bool available_decoder_get(const decoder_interface_t *interface,
                                  void **decoder) {
  /* Search for a free decoder */
  for (u8 j = 0; j < interface->num_decoders; j++) {
    if (NULL != (interface->decoders[j])) {
      if (NULL != decoder) {
        *decoder = interface->decoders[j];
      }
      return true;
    }
  }

  return false;
}


/** Execute an interface function on a decoder channel.
 *
 * \param d         Decoder channel to use.
 * \param func      Interface function to execute.
 */
static void interface_function(decoder_channel_t *d,
                               decoder_interface_function_t func) {
  return func(&d->info, d->decoder);
}

/** Update the state of a decoder channel and its associated decoder instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param d       Decoder channel to use.
 * \param event   Event to process.
 */
static void event(decoder_channel_t *d, event_t event) {
  switch (event) {
    case EVENT_ENABLE_REQUEST: {
      assert(d->decoder->active == false);
      assert(d->state == DECODER_CHANNEL_STATE_DISABLED);

      asm volatile("" : : : "memory"); /* Prevent compiler reordering */
      d->state = DECODER_CHANNEL_STATE_ENABLE_REQUESTED;
    } break;

    case EVENT_ENABLE: {
      assert(d->decoder->active == false);
      assert(d->state == DECODER_CHANNEL_STATE_ENABLE_REQUESTED);

      asm volatile("" : : : "memory"); /* Prevent compiler reordering */
      d->decoder->active = true;
      d->state = DECODER_CHANNEL_STATE_ENABLED;
    } break;

    case EVENT_DISABLE: {
      assert(d->decoder->active == true);
      assert(d->state == DECODER_CHANNEL_STATE_ENABLED);
      /* Sequence point for disable is setting channel state = STATE_DISABLED
       * and/or decoder active = false (order of these two is irrelevant here)
       */
      asm volatile("" : : : "memory"); /* Prevent compiler reordering */
      d->decoder->active = false;
      d->state = DECODER_CHANNEL_STATE_DISABLED;
    } break;

    default:
      assert(!"Invalid state");
      break;
  }
}

/** \} */
