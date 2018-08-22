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
#include "acq/manage.h"
#include "board/nap/track_channel.h"
#include "ndb/ndb.h"
#include "platform_signal.h"
#include "position/position.h"
#include "sbp_utils.h"
#include "settings/settings.h"
#include "signal_db/signal_db.h"
#include "simulator/simulator.h"
#include "timing/timing.h"
#include "track_api.h"
#include "track_common.h"
#include "track_flags.h"
#include "track_interface.h"
#include "track_utils.h"

#define NAP_TRACK_IRQ_THREAD_PRIORITY (HIGHPRIO - 1)
#define NAP_TRACK_IRQ_THREAD_STACK (32 * 1024)

#define CHANNEL_DISABLE_WAIT_TIME_MS 100

#define MAX_VAL_CN0 (255.0 / 4.0)

static THD_WORKING_AREA(wa_nap_track_irq, NAP_TRACK_IRQ_THREAD_STACK);

void nap_track_irq_thread(void *arg);

static tracker_t trackers[NUM_TRACKER_CHANNELS];

static u16 iq_output_mask = 0;

/** Parse the IQ output enable bitfield. */
static bool track_iq_output_notify(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NUM_TRACKER_CHANNELS; i++) {
      tracker_t *tracker = tracker_get(i);
      tracker->output_iq = (iq_output_mask & (1 << i)) != 0;
    }
    return true;
  }
  return false;
}

static const char *const tracker_mode_enum[] = {"rover", "base station", NULL};

static bool set_tracker_mode(struct setting *s, const char *val) {
  int value = 0;
  bool ret = s->type->from_string(s->type->priv, &value, s->len, val);
  if (!ret) {
    return ret;
  }
  enum tracker_mode mode = value;
  switch (mode) {
    case TRACKER_MODE_BASE:
      tp_set_base_station_mode();
      break;
    case TRACKER_MODE_ROVER:
    default:
      tp_set_rover_mode();
      break;
  }
  *(enum tracker_mode *)s->addr = mode;
  return ret;
}

/** Set up the tracking module. */
void track_setup(void) {
  SETTING_NOTIFY("track",
                 "iq_output_mask",
                 iq_output_mask,
                 TYPE_INT,
                 track_iq_output_notify);

  static struct setting_type tracker_mode_setting = {0};

  int TYPE_TRACKER_MODE =
      settings_type_register_enum(tracker_mode_enum, &tracker_mode_setting);

  /* define and apply the default tracking mode */
  static enum tracker_mode tracker_mode = TRACKER_MODE_BASE;
  tp_set_rover_mode();
  tp_tracker_apply_new_mode();

  SETTING_NOTIFY(
      "track", "mode", tracker_mode, TYPE_TRACKER_MODE, set_tracker_mode);

  track_internal_setup();

  for (u32 i = 0; i < NUM_TRACKER_CHANNELS; i++) {
    trackers[i].busy = false;
    chMtxObjectInit(&trackers[i].mutex);
  }

  track_cn0_params_init();

  platform_track_setup();

  chThdCreateStatic(wa_nap_track_irq,
                    sizeof(wa_nap_track_irq),
                    NAP_TRACK_IRQ_THREAD_PRIORITY,
                    nap_track_irq_thread,
                    /* arg = */ NULL);
}

/** Retrieve the tracker channel associated with a tracker channel ID.
 *
 * \param id ID of the tracker channel to be retrieved.
 *
 * \return Associated tracker channel.
 */
tracker_t *tracker_get(u8 id) {
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
bool tracker_available(const u8 id, const me_gnss_signal_t mesid) {
  tracker_t *tracker = tracker_get(id);

  if (!nap_track_supports(id, mesid)) {
    return false;
  }

  return !(tracker->busy);
}

/**
 * Atomically loads tracking channel measurement data.
 *
 * \param[in]  id           Tracking channel identifier.
 * \param[out] info         generic information.
 * \param[out] time_info    timing information.
 * \param[out] freq_info    frequency and phase information.
 * \param[out] misc_info    misc information.
 *
 * \return None
 */
void tracker_get_state(u8 id,
                       tracker_info_t *info,
                       tracker_time_info_t *time_info,
                       tracker_freq_info_t *freq_info,
                       tracker_misc_info_t *misc_info) {
  assert(info);
  assert(time_info);
  assert(freq_info);
  assert(misc_info);

  tracker_t *tracker = tracker_get(id);

  tracker_lock(tracker);

  /* Tracker identifier */
  info->id = (u8)(tracker - &trackers[0]);
  /* Translate/expand flags from tracker internal scope */
  info->flags = tracker->flags;
  /* Signal identifier */
  info->mesid = tracker->mesid;
  /* GLO slot ID */
  info->glo_orbit_slot = tracker->glo_orbit_slot;
  /* Current C/N0 [dB/Hz] */
  info->cn0 = tracker->cn0;
  /* Current time of week for a tracker channel [ms] */
  info->tow_ms = tracker->TOW_ms;
  /* Current time of week residual for tow_ms of the tracker channel [ns] */
  info->tow_residual_ns = tracker->TOW_residual_ns;
  /* Tracking channel init time [ms] */
  info->init_timestamp_ms = tracker->init_timestamp_ms;
  /* Tracking channel update time [ms] */
  info->update_timestamp_ms = tracker->update_timestamp_ms;
  /* Lock counter */
  info->lock_counter = tracker->lock_counter;
  /* Sample counter */
  info->sample_count = tracker->sample_count;
  /* Cross-correlation doppler frequency [hz] */
  info->xcorr_freq = tracker->xcorr_freq;

  time_info->cn0_drop_ms =
      update_count_diff(tracker, &tracker->cn0_above_drop_thres_count);

  if (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK)) {
    time_info->ld_pess_locked_ms =
        update_count_diff(tracker, &tracker->ld_pess_change_count);
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
  if (0 != (tracker->flags & TRACKER_FLAG_HAS_PLOCK) ||
      0 != (tracker->flags & TRACKER_FLAG_HAS_FLOCK)) {
    time_info->ld_pess_unlocked_ms = 0;
  } else {
    time_info->ld_pess_unlocked_ms =
        update_count_diff(tracker, &tracker->ld_pess_change_count);
  }

  /* Current carrier frequency for a tracker channel. */
  freq_info->carrier_freq = tracker->carrier_freq;
  /* Carrier frequency snapshot at the moment of latest PLL/FLL pessimistic
   * lock
   * condition for a tracker channel.
   *
   * The returned carrier frequency is not necessarily the latest reading of
   * the
   * carrier frequency. It is the latest carrier frequency snapshot, when the
   * tracking channel was in PLL/FLL pessimistic lock state.
   */
  freq_info->carrier_freq_at_lock = tracker->carrier_freq_at_lock;
  /* Current carrier frequency for a tracker channel. */
  freq_info->carrier_phase = tracker->carrier_phase;
  /* Code phase in chips */
  freq_info->code_phase_chips = tracker->code_phase_prompt;
  /* Code phase rate in chips/s */
  freq_info->code_phase_rate = tracker->code_phase_rate;

  *misc_info = tracker->misc_info;
  if (tracker->reset_cpo) {
    tracker->misc_info.carrier_phase_offset.value = 0;
    tracker->reset_cpo = false;
  }

  tracker_unlock(tracker);
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
bool tracker_init(const u8 id,
                  const me_gnss_signal_t mesid,
                  u16 glo_orbit_slot,
                  u64 ref_sample_count,
                  double code_phase,
                  float carrier_freq,
                  u32 chips_to_correlate,
                  float cn0_init) {
  tracker_t *tracker = tracker_get(id);

  if (tracker->busy) {
    return false;
  }

  tracker_lock(tracker);
  {
    tracker_cleanup(tracker);

    /* Set up channel */
    tracker->mesid = mesid;
    tracker->nap_channel = id;

    tracker->TOW_ms = TOW_INVALID;
    tracker->TOW_ms_prev = TOW_INVALID;

    /* Calculate code phase rate with carrier aiding. */
    tracker->code_phase_rate =
        (1.0 + carrier_freq / mesid_to_carr_freq(mesid)) *
        code_to_chip_rate(mesid.code);
    tracker->carrier_freq = carrier_freq;

    tracker->sample_count = ref_sample_count;
    tracker->cn0 = cn0_init;
    u32 now = timing_getms();
    tracker->init_timestamp_ms = now;
    tracker->settle_time_ms = code_requires_direct_acq(mesid.code)
                                  ? TRACK_INIT_FROM_ACQ_MS
                                  : TRACK_INIT_FROM_HANDOVER_MS;
    tracker->update_timestamp_ms = now;
    tracker->updated_once = false;
    tracker->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
    tracker->cp_sync.synced = false;

    tracker->bit_polarity = BIT_POLARITY_UNKNOWN;
    tracker->glo_orbit_slot = glo_orbit_slot;

    nav_bit_fifo_init(&tracker->nav_bit_fifo, mesid.code);

    nav_data_sync_init(&tracker->nav_data_sync);

    bit_sync_init(&tracker->bit_sync, mesid);

    tracker_interface_lookup(mesid.code)->init(tracker);

    /* First profile may have different first integration time depending
       on signal type and CN0. */
    tp_tm_e tracking_mode = tracker->profile.loop_params.mode;
    u8 first_int_ms = tp_get_current_cycle_duration(tracking_mode, 0);
    assert((1 == first_int_ms) || (2 == first_int_ms));
    if (tp_get_cycle_count(tracking_mode) > 1) {
      /* nap_track_init() expects first 2 integration times be equal */
      u8 second_int_ms = tp_get_current_cycle_duration(tracking_mode, 1);
      assert(second_int_ms == first_int_ms);
    }
    chips_to_correlate = code_to_chip_rate(mesid.code) * 1e-3 * first_int_ms;

    /* (tracker->update_timestamp_ms = now) must be executed strictly before
       (tracker->busy = true). Otherwise stale_trackers_cleanup() may kick in
       and kill the tracker as stale. */
    COMPILER_BARRIER();

    tracker->busy = true;
  }
  tracker_unlock(tracker);

  nap_track_init(tracker->nap_channel,
                 mesid,
                 ref_sample_count,
                 carrier_freq,
                 code_phase,
                 chips_to_correlate);

  return true;
}

/** Disable the specified tracker channel.
 *
 * \param id      ID of the tracker channel to be disabled.
 *
 */
void tracker_disable(const u8 id) {
  tracker_t *tracker = tracker_get(id);
  assert(tracker->busy);

  /* disable NAP */
  nap_track_disable(tracker->nap_channel);

  /* set the tracker state to disabled */
  tracker->busy = false;

  /* this does restore_acq() and tracker_cleanup() */
  tracker_lock(tracker);
  tracker_interface_lookup(tracker->mesid.code)->disable(tracker);
  tracker_unlock(tracker);
}

/** Add an error flag to a tracker channel.
 *
 * \param tracker   Tracker channel to use.
 * \param error_flag        Error flag to add.
 */
static void error_flags_add(tracker_t *tracker, error_flag_t error_flag) {
  if (error_flag != ERROR_FLAG_NONE) {
    tracker_flag_drop(tracker, CH_DROP_REASON_ERROR);
  }
}

/** Check the state of a tracker channel and generate events as required.
 * \param tracker   Tracker channel to use.
 * \param update_required   True when correlations are pending for the
 *                          tracking channel.
 */
static void serve_nap_request(tracker_t *tracker) {
  if (tracker->busy) {
    tracker_lock(tracker);
    tracker_interface_lookup(tracker->mesid.code)->update(tracker);
    tracker_unlock(tracker);
  } else {
    log_error_mesid(tracker->mesid, "tracker is disabled");
  }
}

/* Unconditionally drops all active tracking channels */
void trackers_drop_all(void) {
  for (u8 ci = 0; ci < nap_track_n_channels; ci++) {
    tracker_t *tracker = tracker_get(ci);
    if (!(tracker->busy)) {
      continue;
    }
    if (0 == (tracker->flags & TRACKER_FLAG_ACTIVE)) {
      continue;
    }
    drop_channel(tracker, CH_DROP_REASON_NEW_MODE);
  }
}

/** Handles pending IRQs and background tasks for tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        an IRQ is pending.
 * \param c0              Channel offset.
 */
void trackers_update(u32 channels_mask, const u8 c0) {
  const u64 now_ms = timing_getms();

  /* experiment 3: like this we always clean a tracker if it has to...
   * and don't clean trackers that are not to be served by NAP */
  for (u8 ci = 0; channels_mask && ((c0 + ci) < nap_track_n_channels); ci++) {
    tracker_t *tracker = tracker_get(c0 + ci);
    bool update_required = (channels_mask & 1) ? true : false;
    /* if NAP has something to do, serve this channel */
    /* due to a chance for a race condition between tracking thread and NAP
       we may end up here for an inactive tracker, which was just dropped.
       So we check the validity of the tracker by looking at its busy flag */
    if (update_required && tracker->busy) {
      serve_nap_request(tracker);
      sanitize_tracker(tracker, now_ms);
    }
    channels_mask >>= 1;
  }
}

/** Sets the missed update error for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        a missed update error has occurred.
 * \param c0              Channel offset.
 */
void trackers_missed(u32 channels_mask, const u8 c0) {
  for (u8 ci = c0; channels_mask && (ci < nap_track_n_channels); ci++) {
    tracker_t *tracker = tracker_get(ci);
    bool error = (channels_mask & 1) ? true : false;
    if (error) {
      error_flags_add(tracker, ERROR_FLAG_MISSED_UPDATE);
    }
    channels_mask >>= 1;
  }
}

/** Send tracking state SBP message.
 * Send information on each tracking channel to host.
 */
static void tracking_send_state_63(void) {
  tracking_channel_state_t states[nap_track_n_channels];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {
    u8 num_sats = simulation_current_num_sats();
    for (u8 i = 0; i < num_sats; i++) {
      states[i] = simulation_current_tracking_state(i);
    }
    for (u8 i = num_sats; i < nap_track_n_channels; i++) {
      states[i].sid = (sbp_gnss_signal_t){
          .sat = 0, .code = 0,
      };
      states[i].cn0 = 0;
    }
  } else {
    u8 max_obs =
        (SBP_FRAMING_MAX_PAYLOAD_SIZE / sizeof(tracking_channel_state_t));
    for (u8 i = 0; (i < nap_track_n_channels) && (i < max_obs); i++) {
      tracker_t *tracker = tracker_get(i);
      bool running = tracker->busy;
      me_gnss_signal_t mesid = tracker->mesid;
      u16 glo_slot_id = tracker->glo_orbit_slot;
      float cn0 = tracker->cn0;
      bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));

      if (!running || !confirmed) {
        states[i].sid = (sbp_gnss_signal_t){
            .sat = 0, .code = 0,
        };
        states[i].fcn = 0;
        states[i].cn0 = 0;
        continue;
      }
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

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8 *)states);
}

/** Send measurement state SBP message.
 * Send information on each tracking channel to host.
 */
static void tracking_send_state_85(void) {
  static bool odd_run = false;
  measurement_state_t meas_states[nap_track_n_channels];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {
    u8 num_sats = simulation_current_num_sats();
    for (u8 i = 0; i < num_sats; i++) {
      meas_states[i] = simulation_measurement_state(i);
    }
    for (u8 i = num_sats; i < nap_track_n_channels; i++) {
      meas_states[i].mesid = (sbp_gnss_signal_t){
          .sat = 0, .code = 0,
      };
      meas_states[i].cn0 = 0;
    }
  } else {
    u8 max_obs = (SBP_FRAMING_MAX_PAYLOAD_SIZE / sizeof(measurement_state_t));
    for (u8 i = 0; (i < nap_track_n_channels) && (i < max_obs); i++) {
      tracker_t *tracker = tracker_get(i);
      bool running = tracker->busy;
      me_gnss_signal_t mesid = tracker->mesid;
      u16 glo_slot_id = tracker->glo_orbit_slot;
      float cn0 = tracker->cn0;
      bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));

      if (!running || !confirmed) {
        meas_states[i].mesid = (sbp_gnss_signal_t){
            .sat = 0, .code = 0,
        };
        meas_states[i].cn0 = 0;
        continue;
      }
      if (IS_GLO(mesid)) {
        /* the odd_run flag below will alternate between `100+FCN` and `SLOT`
         * (as per SBP specs) mainly so that the console can display both and
         * maintain the current user experience */
        meas_states[i].mesid.sat =
            odd_run ? glo_slot_id : (100 + mesid.sat - GLO_FCN_OFFSET);
        meas_states[i].mesid.code = mesid.code;
      } else {
        meas_states[i].mesid.sat = mesid.sat;
        meas_states[i].mesid.code = mesid.code;
      }
      cn0 = (cn0 <= 0) ? 0 : cn0;
      cn0 = (cn0 >= MAX_VAL_CN0) ? MAX_VAL_CN0 : cn0;
      meas_states[i].cn0 = rintf(cn0 * 4.0);
    }
  }
  odd_run = !odd_run;
  sbp_send_msg(
      SBP_MSG_MEASUREMENT_STATE, sizeof(meas_states), (u8 *)meas_states);
}

/** Send either MSG_TRACKING_STATE or MSG_MEASUREMENT_STATE message.
 * Send information on each tracking channel to host,
 * calls the legacy function if the number of channels is lower than 63
 * so that we can incrementally move to 85 channels.
 */
void tracking_send_state(void) {
  if (nap_track_n_channels < 64) {
    tracking_send_state_63();
  } else {
    tracking_send_state_85();
  }
}

/** Goes through all the NAP tracker channels and cleans the stale ones
 * (there should NEVER be any!)
 */
void stale_trackers_cleanup(void) {
  u64 now_ms = timing_getms();

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_t *tracker = tracker_get(i);
    if (!tracker->busy) continue;
    u64 deadline_ms = NAP_CORR_LENGTH_MAX_MS + tracker->update_timestamp_ms;
    if ((now_ms > deadline_ms) && (tracker->updated_once)) {
      log_error_mesid(tracker->mesid, "hit deadline_ms: %" PRIu64, deadline_ms);
      tracker_flag_drop(tracker, CH_DROP_REASON_NO_UPDATES);
      sanitize_tracker(tracker, now_ms);
    }
  }
}
