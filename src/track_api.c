/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_api.h"
#include "track_internal.h"
#include "track.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include <ch.h>
#include <assert.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"
#include "decode.h"

/** \defgroup track_api Tracking API
 * API functions used by tracking channel implementations.
 * \{ */

#define GPS_WEEK_LENGTH_ms (1000 * WEEK_SECS)

static s32 normalize_tow(s32 tow)
{
  assert(tow >= 0);
  return tow % GPS_WEEK_LENGTH_ms;
}

/** Register a tracker interface to enable tracking for a code type.
 *
 * \note element and all subordinate data must be statically allocated!
 *
 * \param element   Struct describing the interface to register.
 */
void tracker_interface_register(tracker_interface_list_element_t *element)
{
  /* p_next = address of next pointer which must be updated */
  tracker_interface_list_element_t **p_next = tracker_interface_list_ptr_get();

  while (*p_next != 0)
    p_next = &(*p_next)->next;

  element->next = 0;
  *p_next = element;
}

/** Read correlations from the NAP for a tracker channel.
 *
 * \param context         Tracker context.
 * \param cs              Output array of correlations.
 * \param sample_count    Output sample count.
 * \param code_phase      Output code phase (chips).
 * \param carrier_phase   Output carrier phase (cycles).
 */
void tracker_correlations_read(tracker_context_t *context,
                               corr_t *cs,
                               u32 *sample_count,
                               double *code_phase,
                               double *carrier_phase)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Read NAP CORR register */
  nap_track_read_results(channel_info->nap_channel,
                         sample_count,
                         cs,
                         code_phase,
                         carrier_phase);
}

/** Write the NAP update register for a tracker channel.
 *
 * \param context             Tracker context.
 * \param doppler_freq_hz     Doppler frequency (Hz).
 * \param code_phase_rate     Code phase rate (chips/s).
 * \param chips_to_correlate  Number of code chips to integrate over.
 */
void tracker_retune(tracker_context_t *context,
                    double doppler_freq_hz,
                    double code_phase_rate,
                    u32 chips_to_correlate)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Write NAP UPDATE register. */
  nap_track_update(channel_info->nap_channel,
                   channel_info->mesid,
                   doppler_freq_hz,
                   code_phase_rate,
                   chips_to_correlate,
                   0);
}

/** Adjust TOW for FIFO delay.
 *
 * \param to_tracker     nav_data_sync_t struct to use
 * \param internal_data  tracker internal data.
 *
 * \return Updated TOW (ms).
 */
static s32 adjust_tow_by_bit_fifo_delay(const nav_data_sync_t to_tracker,
                                        const tracker_internal_data_t *internal_data)
{
  s32 TOW_ms = TOW_INVALID;
  /* Compute time since the pending data was read from the FIFO */
  nav_bit_fifo_index_t fifo_length =
    NAV_BIT_FIFO_INDEX_DIFF(internal_data->nav_bit_fifo.write_index,
                            to_tracker.read_index);
  u32 fifo_time_diff_ms = fifo_length * internal_data->bit_sync.bit_length;

  /* Add full bit times + fractional bit time to the specified TOW */
  TOW_ms = to_tracker.TOW_ms + fifo_time_diff_ms +
           internal_data->nav_bit_TOW_offset_ms;

  TOW_ms = normalize_tow(TOW_ms);

  return TOW_ms;
}

/** Update the TOW for a tracker channel.
 *
 * \param context           Tracker context.
 * \param current_TOW_ms    Current TOW (ms).
 * \param int_ms            Integration period (ms).
 * \param[out] decoded_tow  Decoded TOW indicator
 *
 * \return Updated TOW (ms).
 */
s32 tracker_tow_update(tracker_context_t *context,
                       s32 current_TOW_ms,
                       u32 int_ms,
                       bool *decoded_tow)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Latch TOW from nav message if pending */
  s32 TOW_ms = TOW_INVALID;
  nav_data_sync_t to_tracker;
  if (nav_data_sync_get(&to_tracker, &internal_data->nav_data_sync)) {

    TOW_ms = adjust_tow_by_bit_fifo_delay(to_tracker, internal_data);

    /* Warn if updated TOW does not match the current value */
    if ((current_TOW_ms != TOW_INVALID) && (current_TOW_ms != TOW_ms)) {
      log_warn_mesid(channel_info->mesid,
                     "TOW mismatch: %" PRId32 ", %" PRId32,
                     current_TOW_ms, TOW_ms);
    }
    current_TOW_ms = TOW_ms;
    if (internal_data->bit_polarity != to_tracker.bit_polarity) {
      /* Reset carrier phase offset on bit polarity change */
      internal_data->reset_cpo = true;
      internal_data->bit_polarity = to_tracker.bit_polarity;
    }
    if ((GLO_ORBIT_SLOT_UNKNOWN != internal_data->glo_orbit_slot) &&
        (internal_data->glo_orbit_slot != to_tracker.glo_orbit_slot)) {
      log_warn_mesid(channel_info->mesid, "Unexpected GLO orbit slot change");
    }
    internal_data->glo_orbit_slot = to_tracker.glo_orbit_slot;
  }

  if (NULL != decoded_tow) {
    *decoded_tow = (TOW_ms != TOW_INVALID);
  }

  internal_data->nav_bit_TOW_offset_ms += int_ms;

  if (current_TOW_ms != TOW_INVALID) {
    /* Have a valid time of week - increment it. */
    current_TOW_ms += int_ms;
    current_TOW_ms = normalize_tow(current_TOW_ms);
    /* TODO: maybe keep track of week number in channel state, or
       derive it from system time */
  }

  return current_TOW_ms;
}

/** Set bit sync phase reference
 *
 * \param context           Tracker context.
 * \param bit_phase_ref     Bit phase reference.
 */
void tracker_bit_sync_set(tracker_context_t *context, s8 bit_phase_ref)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  bit_sync_set(&internal_data->bit_sync, bit_phase_ref);
}

/** Update bit sync and output navigation message bits for a tracker channel.
 *
 * \param context           Tracker context.
 * \param int_ms            Integration period (ms).
 * \param corr_prompt_real  Real part of the prompt correlation.
 * \param sensitivity_mode  Flag indicating tracking channel sensitivity mode.
 */
void tracker_bit_sync_update(tracker_context_t *context,
                             u32 int_ms,
                             s32 corr_prompt_real,
                             bool sensitivity_mode)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Update bit sync */
  s32 bit_integrate;
  if (bit_sync_update(&internal_data->bit_sync, corr_prompt_real, int_ms,
                      &bit_integrate)) {
    /* Skip FIFO writes for signals which do not require decoder. */
    if (!code_requires_decoder(channel_info->mesid.code)) {
      return;
    }
    s8 soft_bit = nav_bit_quantize(bit_integrate);

    /* write to FIFO */
    nav_bit_fifo_element_t element = { .soft_bit = soft_bit,
                                       .sensitivity_mode = sensitivity_mode };
    if (nav_bit_fifo_write(&internal_data->nav_bit_fifo, &element)) {

      /* warn if the FIFO has become full */
      if (nav_bit_fifo_full(&internal_data->nav_bit_fifo)) {
        log_warn_mesid(channel_info->mesid, "nav bit FIFO full");
      }
    }

    /* clear nav bit TOW offset */
    internal_data->nav_bit_TOW_offset_ms = 0;
  }
}

/** Get the bit length for a tracker channel.
 *
 * \param context     Tracker context.
 *
 * \return Bit length
 */
u8 tracker_bit_length_get(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return internal_data->bit_sync.bit_length;
}

/** Get the bit alignment state for a tracker channel.
 *
 * \param context     Tracker context.
 *
 * \return true if bit sync has been established and the most recent
 *         integration is bit aligned, false otherwise.
 */
bool tracker_bit_aligned(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return (internal_data->bit_sync.bit_phase ==
            internal_data->bit_sync.bit_phase_ref);
}

/**
 * Tests if the bit sync is established in a tracker channel.
 *
 * \param context     Tracker context.
 *
 * \retval true  Bit sync has been established
 * \retval false Bit sync is not established.
 */
bool tracker_has_bit_sync(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return (BITSYNC_UNSYNCED != internal_data->bit_sync.bit_phase_ref);
}

/**
 * Tests if the bit sync is established in a tracker channel.
 *
 * \param context     Tracker context.
 * \param int_ms      Next integration period in milliseconds.
 *
 * \retval true  Bit sync has been established and the next integration is bit
 *               aligned.
 * \retval false bit sync is not established or the next integration is not bit
 *               aligned.
 */
bool tracker_next_bit_aligned(tracker_context_t *context, u32 int_ms)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  s32 next_bit_phase = internal_data->bit_sync.bit_phase + int_ms;
  next_bit_phase %= internal_data->bit_sync.bit_length;

  return (next_bit_phase == internal_data->bit_sync.bit_phase_ref);
}

/** Sets a channel's carrier phase ambiguity to unknown.
 * Changes the lock counter to indicate to the consumer of the tracking channel
 * observations that the carrier phase ambiguity may have changed. Also
 * invalidates the half cycle ambiguity, which must be resolved again by the navigation
 *  message processing. Should be called if a cycle slip is suspected.
 *
 * \param context     Tracker context.
 */
void tracker_ambiguity_unknown(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  internal_data->bit_polarity = BIT_POLARITY_UNKNOWN;
  internal_data->lock_counter =
      tracking_lock_counter_increment(channel_info->mesid);
  internal_data->reset_cpo = true;
}

/** Checks channel's carrier phase ambiguity status.
 *
 * \param context Tracker context.
 *
 * \return false if ambiguity unknown, true if it is known.
 */
bool tracker_ambiguity_resolved(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return internal_data->bit_polarity != BIT_POLARITY_UNKNOWN;
}

/** Set channel's carrier phase ambiguity status.
 *
 * \param context  Tracker context.
 * \param polarity Polarity of the half-cycle ambiguity
 *
 * \return None
 */
void tracker_ambiguity_set(tracker_context_t *context, s8 polarity)
{
  if (BIT_POLARITY_UNKNOWN == polarity) {
    return;
  }
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  internal_data->bit_polarity = polarity;
}

/** Get the channel's GLO orbital slot information.
 *
 * \param context  Tracker context.
 *
 * \return GLO orbital slot
 */
u16 tracker_glo_orbit_slot_get(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return internal_data->glo_orbit_slot;
}

/** Output a correlation data message for a tracker channel.
 *
 * \param context     Tracker context.
 * \param cs          Array of correlations to send.
 */
void tracker_correlations_send(tracker_context_t *context, const corr_t *cs)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  /* Output I/Q correlations using SBP if enabled for this channel */
  if (internal_data->output_iq) {
    msg_tracking_iq_t msg = {
      .channel = channel_info->nap_channel,
    };
    /* TODO GLO: Handle GLO orbit slot properly. */
    if (is_glo_sid(channel_info->mesid)) {
      return;
    }
    gnss_signal_t sid = mesid2sid(channel_info->mesid,
                                  internal_data->glo_orbit_slot);
    msg.sid = sid_to_sbp(sid);
    for (u32 i = 0; i < 3; i++) {
      msg.corrs[i].I = cs[i].I;
      msg.corrs[i].Q = cs[i].Q;
    }
    sbp_send_msg(SBP_MSG_TRACKING_IQ, sizeof(msg), (u8*)&msg);
  }
}

/**
 * The function checks if PRN fail (decoded prn from L2C data stream
 * is not correspond to SVID) flag set or not.
 * Called from Tracking task.
 * \param[in] context Tracker context.
 * \return    TRUE if PRN fail flag is set, otherwise FAIL
 */
bool tracker_check_prn_fail_flag(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return internal_data->prn_check_fail;
}

/**
 * Checks if the tracker has cross-correlation flag set.
 *
 * Tracker can use this method to check if a cross-correlation flag is set by
 * external thread.
 *
 * \param[in] context Tracker context.
 *
 * \return Cross-correlation flag value-
 */
bool tracker_check_xcorr_flag(tracker_context_t *context)
{
  const tracker_channel_info_t *channel_info;
  tracker_internal_data_t *internal_data;
  tracker_internal_context_resolve(context, &channel_info, &internal_data);

  return internal_data->xcorr_flag;
}

/** \} */
