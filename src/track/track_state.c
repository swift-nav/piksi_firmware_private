/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_state.h"
#include "board/nap/track_channel.h"
#include "ndb/ndb.h"
#include "platform_signal.h"
#include "position/position.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "simulator/simulator.h"
#include "timing/timing.h"
#include "track_api.h"
#include "track_common.h"
#include "track_flags.h"
#include "track_interface.h"
#include "track_sbp.h"
#include "track_utils.h"

#define NAP_TRACK_IRQ_THREAD_PRIORITY (HIGHPRIO - 1)
#define NAP_TRACK_IRQ_THREAD_STACK (32 * 1024)

#define CHANNEL_DISABLE_WAIT_TIME_MS 100

#define MAX_VAL_CN0 (255.0 / 4.0)

u16 max_pll_integration_time_ms = 20;

static THD_WORKING_AREA(wa_nap_track_irq, NAP_TRACK_IRQ_THREAD_STACK);

void nap_track_irq_thread(void *arg);

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE,
  EVENT_DISABLE_WAIT_COMPLETE
} event_t;

static tracker_t trackers[NUM_TRACKER_CHANNELS];

/** send_trk_detailed setting is a stop gap to suppress this
  * bandwidth intensive msg until a more complete "debug"
  * strategy is designed and implemented. */
static bool send_trk_detailed = 0;

static u16 iq_output_mask = 0;

/** Parse the IQ output enable bitfield. */
static bool track_iq_output_notify(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NUM_TRACKER_CHANNELS; i++) {
      tracker_t *tracker_channel = tracker_get(i);
      tracker_channel->output_iq = (iq_output_mask & (1 << i)) != 0;
    }
    return true;
  }
  return false;
}

/** Max PLL integration time change notification callback */
static bool max_pll_integration_time_notify(struct setting *s,
                                            const char *val) {
  /* update global max_pll_integration_time_ms using the default notify */
  bool ret = settings_default_notify(s, val);
  if (max_pll_integration_time_ms < 1) {
    max_pll_integration_time_ms = 1; /* 1ms integration time is the smallest */
  }
  log_info("Max configured PLL integration time update: %" PRIu16 " ms",
           max_pll_integration_time_ms);

  return ret;
}

/** Set up the tracking module. */
void track_setup(void) {
  SETTING_NOTIFY("track",
                 "iq_output_mask",
                 iq_output_mask,
                 TYPE_INT,
                 track_iq_output_notify);
  SETTING("track", "send_trk_detailed", send_trk_detailed, TYPE_BOOL);
  SETTING_NOTIFY("track",
                 "max_pll_integration_time_ms",
                 max_pll_integration_time_ms,
                 TYPE_INT,
                 max_pll_integration_time_notify);

  track_internal_setup();

  for (u32 i = 0; i < NUM_TRACKER_CHANNELS; i++) {
    trackers[i].state = STATE_DISABLED;
    chMtxObjectInit(&trackers[i].mutex);
    chMtxObjectInit(&trackers[i].mutex_pub);
  }

  track_cn0_params_init();
  tp_init();

  platform_track_setup();

  chThdCreateStatic(wa_nap_track_irq,
                    sizeof(wa_nap_track_irq),
                    NAP_TRACK_IRQ_THREAD_PRIORITY,
                    nap_track_irq_thread,
                    /* arg = */ NULL);
}

/** Retrieve the tracker channel associated with a tracker channel ID.
 *
 * \param tracker_channel_id ID of the tracker channel to be retrieved.
 *
 * \return Associated tracker channel.
 */
tracker_t *tracker_get(tracker_id_t id) {
  assert(id < NUM_TRACKER_CHANNELS);
  return &trackers[id];
}

/** Determine if a tracker channel is available to track the specified sid.
 *
 * \param id      ID of the tracker channel to be checked.
 * \param mesid   Signal to be tracked.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
bool tracker_available(tracker_id_t id, const me_gnss_signal_t mesid) {
  const tracker_t *tracker_channel = tracker_get(id);

  if (!nap_track_supports(id, mesid)) {
    return false;
  }

  return tracker_runnable(tracker_channel);
}

/**
 * Computes tracking channel public information.
 *
 * This function must be called from tracking channel lock scope.
 *
 * \param[in,out] tracker_channel  Tracking channel as a data source
 * \param[out]    info             Optional destination for generic data block.
 * \param[out]    time_info        Optional destination for timing data block.
 * \param[out]    freq_info        Optional destination for frequency and phase
 *                                 data block.
 * \param[out]    ctrl_params      Optional destination for tracking loop
 *                                 controller data block.
 * \param[out]    reset_cpo        Optional destination for carrier phase
 *                                 ambiguity reset flag.
 *
 * \return None
 */

static void tracking_channel_compute_values(tracker_t *tracker_channel,
                                            tracker_info_t *info,
                                            tracker_time_info_t *time_info,
                                            tracker_freq_info_t *freq_info,
                                            tracker_ctrl_info_t *ctrl_params,
                                            bool *reset_cpo) {
  if (NULL != info) {
    /* Tracker identifier */
    info->id = (tracker_id_t)(tracker_channel - &trackers[0]);
    /* Translate/expand flags from tracker internal scope */
    info->flags = tracker_channel->flags;
    /* Signal identifier */
    info->mesid = tracker_channel->mesid;
    /* GLO slot ID */
    info->glo_orbit_slot = tracker_channel->glo_orbit_slot;
    /* Current C/N0 [dB/Hz] */
    info->cn0 = tracker_channel->cn0;
    /* Current time of week for a tracker channel [ms] */
    info->tow_ms = tracker_channel->TOW_ms;
    /* Current time of week residual for tow_ms of the tracker channel [ns] */
    info->tow_residual_ns = tracker_channel->TOW_residual_ns;
    /* Tracking channel init time [ms] */
    info->init_timestamp_ms = tracker_channel->init_timestamp_ms;
    /* Tracking channel update time [ms] */
    info->update_timestamp_ms = tracker_channel->update_timestamp_ms;
    /* Lock counter */
    info->lock_counter = tracker_channel->lock_counter;
    /* Sample counter */
    info->sample_count = tracker_channel->sample_count;
    /* Cross-correlation doppler frequency [hz] */
    info->xcorr_freq = tracker_channel->xcorr_freq;
  }
  if (NULL != time_info) {
    time_info->cn0_drop_ms = update_count_diff(
        tracker_channel, &tracker_channel->cn0_above_drop_thres_count);
    time_info->cn0_usable_ms = update_count_diff(
        tracker_channel, &tracker_channel->cn0_below_use_thres_count);

    if (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK)) {
      time_info->ld_pess_locked_ms = update_count_diff(
          tracker_channel, &tracker_channel->ld_pess_change_count);
    } else {
      time_info->ld_pess_locked_ms = 0;
    }

    /* The time in ms for which the FLL/PLL pessimistic lock detector has
     * reported
     * being unlocked for a tracker channel.
     *
     * If tracker channel is run by FLL, then time of absence of FLL pessimistic
     * lock is reported.
     * If tracker channel is run by PLL, then time of absence of PLL pessimistic
     * lock is reported.
     */
    if (0 != (tracker_channel->flags & TRACKER_FLAG_HAS_PLOCK) ||
        0 != (tracker_channel->flags & TRACKER_FLAG_HAS_FLOCK)) {
      time_info->ld_pess_unlocked_ms = 0;
    } else {
      time_info->ld_pess_unlocked_ms = update_count_diff(
          tracker_channel, &tracker_channel->ld_pess_change_count);
    }
  }
  if (NULL != freq_info) {
    /* Current carrier frequency for a tracker channel. */
    freq_info->carrier_freq = tracker_channel->carrier_freq;
    /* Carrier frequency snapshot at the moment of latest PLL/FLL pessimistic
     * lock
     * condition for a tracker channel.
     *
     * The returned carrier frequency is not necessarily the latest reading of
     * the
     * carrier frequency. It is the latest carrier frequency snapshot, when the
     * tracking channel was in PLL/FLL pessimistic lock state.
     */
    freq_info->carrier_freq_at_lock = tracker_channel->carrier_freq_at_lock;
    /* Current carrier frequency for a tracker channel. */
    freq_info->carrier_phase = tracker_channel->carrier_phase;
    /* Code phase in chips */
    freq_info->code_phase_chips = tracker_channel->code_phase_prompt;
    /* Code phase rate in chips/s */
    freq_info->code_phase_rate = tracker_channel->code_phase_rate;
    /* Acceleration [g] */
    freq_info->acceleration = tracker_channel->acceleration;
  }
  if (NULL != ctrl_params) {
    /* Copy loop controller parameters */
    tp_profile_t *profile = &tracker_channel->profile;
    ctrl_params->pll_bw = profile->loop_params.carr_bw;
    ctrl_params->fll_bw = profile->loop_params.fll_bw;
    ctrl_params->dll_bw = profile->loop_params.code_bw;
    ctrl_params->int_ms = tp_get_dll_ms(tracker_channel->tracking_mode);
  }
  if (NULL != reset_cpo) {
    *reset_cpo = tracker_channel->reset_cpo;
    tracker_channel->reset_cpo = false;
  }
}

/** Clear the error flags for a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void error_flags_clear(tracker_t *tracker_channel) {
  tracker_channel->flags &= ~TRACKER_FLAG_ERROR;
}

/**
 * Method atomically updates tracking channel public informational block.
 *
 * The channel locks public informational block and updates it according to
 * input parameters.
 *
 * \note Carrier phase offset can't be updated by this method. It can be only
 *       reset to 0 if \a reset_cpo is set to \a true.
 *
 * \param[in,out] tracker_channel Tracker channel data
 * \param[in]     info        Generic information block.
 * \param[in]     time_info   Timing information block.
 * \param[in]     freq_info   Frequency and phase information block.
 * \param[in]     ctrl_params Control loop information block.
 * \param[in]     reset_cpo   Flag, if carrier phase offset shall be reset.
 *
 * \return None
 *
 * \sa tracking_channel_get_values
 * \sa tracking_channel_set_carrier_phase_offset
 * \sa tracking_channel_carrier_phase_offsets_adjust
 */
static void tracking_channel_update_values(
    tracker_t *tracker_channel,
    const tracker_info_t *info,
    const tracker_time_info_t *time_info,
    const tracker_freq_info_t *freq_info,
    const tracker_ctrl_info_t *ctrl_params,
    bool reset_cpo) {
  tracker_pub_data_t *pub_data = &tracker_channel->pub_data;

  chMtxLock(&tracker_channel->mutex_pub);
  pub_data->gen_info = *info;
  pub_data->time_info = *time_info;
  pub_data->freq_info = *freq_info;
  if (reset_cpo) {
    /* Do CPO reset */
    /* no need to update timestamp for zero offset as it keeps count
       time on this channel at zero anyways */
    pub_data->misc_info.carrier_phase_offset.value = 0;
  }
  pub_data->ctrl_info = *ctrl_params;
  chMtxUnlock(&tracker_channel->mutex_pub);
}

/** Update the state of a tracker channel and its associated tracker instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param event             Event to process.
 */
static void event(tracker_t *tracker_channel, event_t event) {
  switch (event) {
    case EVENT_ENABLE: {
      assert(tracker_channel->state == STATE_DISABLED);
      /* Sequence point for enable is setting channel state = STATE_ENABLED */
      COMPILER_BARRIER(); /* Prevent compiler reordering */
      tracker_channel->state = STATE_ENABLED;
    } break;

    case EVENT_DISABLE_REQUEST: {
      assert(tracker_channel->state == STATE_ENABLED);
      tracker_channel->state = STATE_DISABLE_REQUESTED;
    } break;

    case EVENT_DISABLE: {
      assert(tracker_channel->state == STATE_DISABLE_REQUESTED);
      tracker_channel->state = STATE_DISABLE_WAIT;
    } break;

    case EVENT_DISABLE_WAIT_COMPLETE: {
      assert(tracker_channel->state == STATE_DISABLE_WAIT);
      /* Sequence point for disable is setting channel state = STATE_DISABLED
       * and/or tracker active = false (order of these two is irrelevant here)
       */
      COMPILER_BARRIER(); /* Prevent compiler reordering */
      tracker_channel->state = STATE_DISABLED;
    } break;

    default: { assert(!"Invalid event"); } break;
  }
}

/** Initialize a tracker channel to track the specified mesid.
 *
 * \param id                    ID of the tracker channel to be initialized.
 * \param mesid                 ME signal to be tracked.
 * \param glo_orbit_slot        GLO orbital slot.
 * \param ref_sample_count      NAP sample count at which code_phase was
 * acquired.
 * \param code_phase            Code phase
 * \param carrier_freq          Carrier frequency Doppler (Hz).
 * \param chips_to_correlate    Chips to correlate.
 * \param cn0_init              Initial C/N0 estimate (dBHz).
 *
 * \return true if the tracker channel was initialized, false otherwise.
 */
bool tracker_init(tracker_id_t id,
                  const me_gnss_signal_t mesid,
                  u16 glo_orbit_slot,
                  u64 ref_sample_count,
                  double code_phase,
                  float carrier_freq,
                  u32 chips_to_correlate,
                  float cn0_init) {
  tracker_t *tracker_channel = tracker_get(id);

  if (!tracker_runnable(tracker_channel)) {
    return false;
  }

  /* Channel public data blocks */
  tracker_info_t info;
  tracker_time_info_t time_info;
  tracker_freq_info_t freq_info;
  tracker_ctrl_info_t ctrl_params;

  tracker_lock(tracker_channel);
  {
    tracker_cleanup(tracker_channel);

    /* Set up channel */
    tracker_channel->mesid = mesid;
    tracker_channel->nap_channel = id;

    tracker_channel->TOW_ms = TOW_INVALID;
    tracker_channel->TOW_ms_prev = TOW_INVALID;

    /* Calculate code phase rate with carrier aiding. */
    tracker_channel->code_phase_rate =
        (1.0 + carrier_freq / mesid_to_carr_freq(mesid)) *
        code_to_chip_rate(mesid.code);
    tracker_channel->carrier_freq = carrier_freq;

    tracker_channel->sample_count = ref_sample_count;
    tracker_channel->cn0 = cn0_init;
    u32 now = timing_getms();
    tracker_channel->init_timestamp_ms = now;
    tracker_channel->update_timestamp_ms = now;
    tracker_channel->updated_once = false;
    tracker_channel->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
    tracker_channel->cp_sync.synced = false;
    tracker_channel->health = GLO_SV_HEALTHY;

    tracker_channel->bit_polarity = BIT_POLARITY_UNKNOWN;
    tracker_channel->glo_orbit_slot = glo_orbit_slot;

    nav_bit_fifo_init(&tracker_channel->nav_bit_fifo);
    nav_data_sync_init(&tracker_channel->nav_data_sync);
    bit_sync_init(&tracker_channel->bit_sync, mesid);

    tracker_interface_lookup(mesid)->init(tracker_channel);

    /* Clear error flags before starting NAP tracking channel */
    error_flags_clear(tracker_channel);

    /* Change the channel state to ENABLED. */
    event(tracker_channel, EVENT_ENABLE);

    /* Load channel public data while in channel lock */
    tracking_channel_compute_values(
        tracker_channel, &info, &time_info, &freq_info, &ctrl_params, NULL);
  }
  tracker_unlock(tracker_channel);

  nap_track_init(tracker_channel->nap_channel,
                 mesid,
                 ref_sample_count,
                 carrier_freq,
                 code_phase,
                 chips_to_correlate);

  /* Update channel public data outside of channel lock */
  tracking_channel_update_values(
      tracker_channel, &info, &time_info, &freq_info, &ctrl_params, true);

  return true;
}

/** Disable the specified tracker channel.
 *
 * \param id      ID of the tracker channel to be disabled.
 *
 * \return true if the tracker channel was disabled, false otherwise.
 */
bool tracker_disable(tracker_id_t id) {
  /* Request disable */
  tracker_t *tracker_channel = tracker_get(id);
  event(tracker_channel, EVENT_DISABLE_REQUEST);
  return true;
}

/** Determine if a tracker channel can be started to track the specified mesid.
 *
 * \param tracker_channel_id    ID of the tracker channel to be checked.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
bool tracker_runnable(const tracker_t *tracker_channel) {
  return (tracker_state_get(tracker_channel) == STATE_DISABLED);
}

/** Return the state of a tracker channel.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param tracker_channel   Tracker channel to use.
 *
 * \return state of the decoder channel.
 */
state_t tracker_state_get(const tracker_t *tracker_channel) {
  state_t state = tracker_channel->state;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return state;
}

/** Disable the NAP tracking channel associated with a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void nap_channel_disable(const tracker_t *tracker_channel) {
  nap_track_disable(tracker_channel->nap_channel);
}

/** Add an error flag to a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param error_flag        Error flag to add.
 */
static void error_flags_add(tracker_t *tracker_channel,
                            error_flag_t error_flag) {
  if (error_flag != ERROR_FLAG_NONE) {
    tracker_channel->flags |= TRACKER_FLAG_ERROR;
  }
}

/** Check the state of a tracker channel and generate events as required.
 * \param tracker_channel   Tracker channel to use.
 * \param update_required   True when correlations are pending for the
 *                          tracking channel.
 */
static void tracker_channel_process(tracker_t *tracker, bool update_required) {
  switch (tracker_state_get(tracker)) {
    case STATE_ENABLED: {
      if (update_required) {
        /* Channel public data blocks for transferring between locks */
        tracker_info_t info;
        tracker_time_info_t time_info;
        tracker_freq_info_t freq_info;
        tracker_ctrl_info_t ctrl_params;
        bool reset_cpo;

        tracker_lock(tracker);
        {
          tracker_interface_lookup(tracker->mesid)->update(tracker);

          /* Read channel public data while in channel lock */
          tracking_channel_compute_values(
              tracker, &info, &time_info, &freq_info, &ctrl_params, &reset_cpo);
        }
        tracker_unlock(tracker);

        /* Update channel public data outside of channel lock */
        tracking_channel_update_values(
            tracker, &info, &time_info, &freq_info, &ctrl_params, reset_cpo);
      }
    } break;

    case STATE_DISABLE_REQUESTED: {
      nap_channel_disable(tracker);
      tracker_lock(tracker);
      {
        tracker_interface_lookup(tracker->mesid)->disable(tracker);
        piksi_systime_get(&tracker->disable_time);
        event(tracker, EVENT_DISABLE);
      }
      tracker_unlock(tracker);
    } break;

    case STATE_DISABLE_WAIT: {
      if (piksi_systime_elapsed_since_ms(&tracker->disable_time) >=
          CHANNEL_DISABLE_WAIT_TIME_MS) {
        event(tracker, EVENT_DISABLE_WAIT_COMPLETE);
      }
    } break;

    case STATE_DISABLED: {
      if (update_required) {
        /* Tracking channel is not owned by the update thread, but the update
         * register must be written to clear the interrupt flag. Set error
         * flag to indicate that NAP is in an unknown state. */
        nap_channel_disable(tracker);
        error_flags_add(tracker, ERROR_FLAG_INTERRUPT_WHILE_DISABLED);
      }
    } break;

    default: { assert(!"Invalid tracking channel state"); } break;
  }
}

/** Handles pending IRQs and background tasks for tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        an IRQ is pending.
 */
void trackers_update(u64 channels_mask) {
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_t *tracker_channel = tracker_get(channel);
    bool update_required = (channels_mask & 1) ? true : false;
    tracker_channel_process(tracker_channel, update_required);
    channels_mask >>= 1;
  }
}

/** Sets the missed update error for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        a missed update error has occurred.
 */
void trackers_missed(u64 channels_mask) {
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_t *tracker_channel = tracker_get(channel);
    bool error = (channels_mask & 1) ? true : false;
    if (error) {
      error_flags_add(tracker_channel, ERROR_FLAG_MISSED_UPDATE);
    }
    channels_mask >>= 1;
  }
}

/** Send tracking state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_state(void) {
  tracking_channel_state_t states[nap_track_n_channels];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {
    u8 num_sats = simulation_current_num_sats();
    for (u8 i = 0; i < num_sats; i++) {
      states[i] = simulation_current_tracking_state(i);
    }
    if (num_sats < nap_track_n_channels) {
      for (u8 i = num_sats; i < nap_track_n_channels; i++) {
        states[i].sid = (sbp_gnss_signal_t){
            .sat = 0, .code = 0,
        };
        states[i].cn0 = 0;
      }
    }

  } else {
    u8 uMaxObs =
        (SBP_FRAMING_MAX_PAYLOAD_SIZE / sizeof(tracking_channel_state_t));
    for (u8 i = 0; (i < nap_track_n_channels) && (i < uMaxObs); i++) {
      tracker_t *tracker_channel = tracker_get(i);
      bool running;
      bool confirmed;
      me_gnss_signal_t mesid;
      u16 glo_slot_id;
      float cn0;

      running = (tracker_state_get(tracker_channel) == STATE_ENABLED);
      mesid = tracker_channel->mesid;
      glo_slot_id = tracker_channel->glo_orbit_slot;
      cn0 = tracker_channel->cn0;
      confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));

      if (!running || !confirmed) {
        states[i].sid = (sbp_gnss_signal_t){
            .sat = 0, .code = 0,
        };
        states[i].fcn = 0;
        states[i].cn0 = 0;
      } else {
        /* TODO GLO: Handle GLO orbit slot properly. */
        if (IS_GLO(mesid)) {
          states[i].sid.sat = glo_slot_id;
          states[i].sid.code = mesid.code;
          states[i].fcn = mesid.sat;
        } else {
          states[i].sid.sat = mesid.sat;
          states[i].sid.code = mesid.code;
          states[i].fcn = 0;
        }
        cn0 = (cn0 <= 0) ? 0 : cn0;
        cn0 = (cn0 >= MAX_VAL_CN0) ? MAX_VAL_CN0 : cn0;
        states[i].cn0 = rintf(cn0 * 4.0);
      }
    }
  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8 *)states);
}

/** Send tracking detailed state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_detailed_state(void) {
  if (!send_trk_detailed) {
    return;
  }

  last_good_fix_t lgf;
  last_good_fix_t *plgf = &lgf;

  if ((NDB_ERR_NONE != ndb_lgf_read(&lgf)) || !lgf.position_solution.valid) {
    plgf = NULL;
  }

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_info_t channel_info;
    tracker_freq_info_t freq_info;
    tracker_time_info_t time_info;
    tracker_ctrl_info_t ctrl_info;
    tracker_misc_info_t misc_info;
    msg_tracking_state_detailed_t sbp;

    tracker_get_values(
        i, &channel_info, &time_info, &freq_info, &ctrl_info, &misc_info);

    if (0 == (channel_info.flags & TRACKER_FLAG_ACTIVE) ||
        0 == (channel_info.flags & TRACKER_FLAG_CONFIRMED)) {
      continue;
    }

    /* TODO GLO: Handle GLO orbit slot properly. */
    if (IS_GLO(channel_info.mesid)) {
      continue;
    }

    track_sbp_get_detailed_state(&sbp,
                                 &channel_info,
                                 &freq_info,
                                 &time_info,
                                 &ctrl_info,
                                 &misc_info,
                                 plgf);

    sbp_send_msg(SBP_MSG_TRACKING_STATE_DETAILED, sizeof(sbp), (u8 *)&sbp);
  }
}
