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

#include <inttypes.h>

#include "acq/manage.h"
#include "board/nap/track_channel.h"
#include "ndb/ndb.h"
#include "platform_signal.h"
#include "position/position.h"
#include "sbp/sbp_utils.h"
#include "settings/settings_client.h"
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

static tracker_t trackers[ME_CHANNELS];

static u16 iq_output_mask = 0;

/** Parse the IQ output enable bitfield. */
static int track_iq_output_notify(void *ctx) {
  (void)ctx;

  for (u8 i = 0; i < ME_CHANNELS; i++) {
    tracker_t *tracker = tracker_get(i);
    tracker->output_iq = (iq_output_mask & (1 << i)) != 0;
  }
  return SETTINGS_WR_OK;
}

static const char *const tracker_mode_enum[] = {"rover", "base station", NULL};

/* define and apply the default tracking mode */
static int tracker_mode = TRACKER_MODE_ROVER;

static int set_tracker_mode(void *ctx) {
  (void)ctx;

  switch (tracker_mode) {
    case TRACKER_MODE_BASE:
      tp_set_base_station_mode();
      break;
    case TRACKER_MODE_ROVER:
    default:
      tp_set_rover_mode();
      break;
  }

  return SETTINGS_WR_OK;
}

/** Set up the tracking module. */
void track_setup(void) {
  SETTING_NOTIFY("track",
                 "iq_output_mask",
                 iq_output_mask,
                 SETTINGS_TYPE_INT,
                 track_iq_output_notify);

  settings_type_t settings_type_mode;
  settings_api_register_enum(tracker_mode_enum, &settings_type_mode);

  SETTING_NOTIFY(
      "track", "mode", tracker_mode, settings_type_mode, set_tracker_mode);

  track_internal_setup();

  for (u8 i = 0; i < ME_CHANNELS; i++) {
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
  assert(id < ME_CHANNELS);
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
  if (!nap_track_supports(id, mesid)) {
    return false;
  }

  tracker_t *tracker = tracker_get(id);
  return !(tracker->busy);
}

/**
 * Atomically loads tracking channel measurement data.
 *
 * \param[in]  id           Tracking channel identifier.
 * \param[out] info         generic information.
 * \param[out] freq_info    frequency and phase information.
 *
 * \return None
 */
void tracker_get_state(u8 id,
                       tracker_info_t *info,
                       tracker_freq_info_t *freq_info) {
  assert(info);
  assert(freq_info);

  tracker_t *tracker = tracker_get(id);

  tracker_lock(tracker);

  /* Tracker identifier */
  info->id = id;
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
  /* Lock counter */
  info->lock_counter = tracker->lock_counter;
  /* Sample counter */
  info->sample_count = tracker->sample_count;

  /* Current doppler frequency for a tracker channel. */
  freq_info->doppler_hz = tracker->doppler_hz;
  /* Current carrier phase for a tracker channel. */
  freq_info->carrier_phase = tracker->carrier_phase;
  /* Code phase in chips */
  freq_info->code_phase_chips = tracker->code_phase_prompt;
  /* Code phase rate in chips/s */
  freq_info->code_phase_rate = tracker->code_phase_rate;

  freq_info->cpo = tracker->cpo;
  if (tracker->reset_cpo) {
    tracker->cpo.value = 0;
    tracker->reset_cpo = false;
  }

  tracker_unlock(tracker);
}

/** Initialize a tracker channel to track the specified mesid.
 *
 * \param id                  ID of the tracker channel to be initialized.
 * \param mesid               ME signal to be tracked.
 * \param glo_orbit_slot      GLO orbital slot.
 * \param ref_sample_count    NAP sample count at which code_phase was acquired.
 * \param code_phase          Code phase
 * \param doppler_hz          Doppler frequency (Hz).
 * \param chips_to_correlate  Chips to correlate.
 * \param cn0_init            Initial C/N0 estimate (dBHz).
 *
 * \return true if the tracker channel was initialized, false otherwise.
 */
bool tracker_init(const u8 id,
                  const me_gnss_signal_t mesid,
                  u16 glo_orbit_slot,
                  u64 ref_sample_count,
                  double code_phase,
                  float doppler_hz,
                  u32 chips_to_correlate,
                  float cn0_init) {
  tracker_t *tracker = tracker_get(id);

  if (tracker->busy) {
    return false;
  }

  noise_update_mesid_status(mesid, /*intrack=*/true);

  tracker_lock(tracker);
  {
    tracker_cleanup(tracker);

    /* Set up channel */
    tracker->mesid = mesid;
    tracker->nap_channel = id;

    tracker->TOW_ms = TOW_INVALID;
    tracker->TOW_ms_prev = TOW_INVALID;

    /* Calculate code phase rate with carrier aiding. */
    tracker->code_phase_rate = (1.0 + doppler_hz / mesid_to_carr_freq(mesid)) *
                               code_to_chip_rate(mesid.code);
    tracker->doppler_hz = doppler_hz;

    tracker->sample_count = ref_sample_count;
    /* First profile selection is based on initial CN0 estimate. */
    tracker->cn0 = cn0_init;

    tracker_timer_init(&tracker->unlocked_timer);
    tracker_timer_arm(&tracker->unlocked_timer, /*deadline_ms=*/-1);

    tracker_timer_init(&tracker->cn0_below_drop_thres_timer);

    tracker_timer_init(&tracker->age_timer);
    tracker_timer_arm(&tracker->age_timer, /*deadline_ms=*/-1);

    s64 deadline_ms = (s64)tracker_time_now_ms();
    if (code_requires_direct_acq(mesid.code)) {
      deadline_ms += TRACK_INIT_FROM_ACQ_MS;
    } else {
      deadline_ms += TRACK_INIT_FROM_HANDOVER_MS;
    }
    tracker_timer_arm(&tracker->init_settle_timer, deadline_ms);

    tracker_timer_init(&tracker->update_timer);
    tracker_timer_arm(&tracker->update_timer, /*deadline_ms=*/-1);
    tracker->updated_once = false;

    tracker_timer_init(&tracker->doppler_age_timer);
    tracker_timer_arm(&tracker->doppler_age_timer, -1);

    tracker_timer_init(&tracker->profile.profile_settle_timer);

    tracker->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
    tracker->cp_sync.synced = false;

    tracker->bit_polarity = BIT_POLARITY_UNKNOWN;
    tracker->glo_orbit_slot = glo_orbit_slot;

    nav_bit_fifo_init(&tracker->nav_bit_fifo, mesid.code);

    nav_data_sync_init(&tracker->nav_data_sync);

    bit_sync_init(&tracker->bit_sync, mesid);

    tracker_interface_lookup(mesid.code)->init(tracker);

    /* Set CN0 below drop threshold for quick rejection of bad signals. */
    tracker->cn0 =
        TP_HARD_CN0_DROP_THRESHOLD_DBHZ - TP_TRACKER_CN0_CONFIRM_DELTA;

    /* First profile may have different first integration time depending
       on signal type and CN0. */
    tp_tm_e tracking_mode = tracker->profile.loop_params.mode;
    u8 first_int_ms = tp_get_cycle_duration(tracking_mode, 0);
    /* First entry should have short integration time,
     * so that loop update is quickly applied. */
    assert((1 == first_int_ms) || (2 == first_int_ms));
    if (tp_get_cycle_count(tracking_mode) > 1) {
      /* nap_track_init() expects first 2 integration times be equal */
      u8 second_int_ms = tp_get_cycle_duration(tracking_mode, 1);
      assert(second_int_ms == first_int_ms);
    }
    chips_to_correlate =
        (u32)lrint(code_to_chip_rate(mesid.code) * 1e-3 * first_int_ms);

    COMPILER_BARRIER();

    tracker->busy = true;
  }
  tracker_unlock(tracker);

  nap_track_init(tracker->nap_channel,
                 mesid,
                 ref_sample_count,
                 doppler_hz,
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
  tracker_lock(tracker);
  tracker_interface_lookup(tracker->mesid.code)->update(tracker);
  tracker_unlock(tracker);
}

/** Handles pending IRQs and background tasks for tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        an IRQ is pending.
 * \param c0              Channel offset.
 */
void trackers_update(u32 channels_mask, const u8 c0) {
  tracker_t *pt_tracker = tracker_get(c0);
  for (u8 ci = c0; channels_mask && (ci < ME_CHANNELS); ci++) {
    bool update_required = (channels_mask & 1) ? true : false;
    /* if NAP has something to do, serve this channel */
    /* due to a chance for a race condition between tracking thread and NAP
       we may end up here for an inactive tracker, which was just dropped.
       So we check the validity of the tracker by looking at its busy flag */
    if (update_required && pt_tracker->busy) {
      serve_nap_request(pt_tracker);
      sanitize_tracker(pt_tracker);
    }
    pt_tracker++;
    channels_mask >>= 1;
  }
}

/** Sets the missed update error for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        a missed update error has occurred.
 * \param c0              Channel offset.
 */
void trackers_missed(u32 channels_mask, const u8 c0) {
  tracker_t *pt_tracker = tracker_get(c0);
  for (u8 ci = c0; channels_mask && (ci < ME_CHANNELS); ci++) {
    bool error = (channels_mask & 1) ? true : false;
    if (error) {
      error_flags_add(pt_tracker, ERROR_FLAG_MISSED_UPDATE);
    }
    pt_tracker++;
    channels_mask >>= 1;
  }
}

/** Send measurement state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_state(void) {
  static bool odd_run = false;
  measurement_state_t meas_states[ME_CHANNELS];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {
    u8 num_sats = simulation_current_num_sats();
    for (u8 i = 0; i < num_sats; i++) {
      meas_states[i] = simulation_measurement_state(i);
    }
    for (u8 i = num_sats; i < ME_CHANNELS; i++) {
      meas_states[i].mesid = (sbp_gnss_signal_t){
          .sat = 0,
          .code = 0,
      };
      meas_states[i].cn0 = 0;
    }
  } else {
    u8 max_obs = (SBP_FRAMING_MAX_PAYLOAD_SIZE / sizeof(measurement_state_t));
    for (u8 i = 0; (i < ME_CHANNELS) && (i < max_obs); i++) {
      tracker_t *tracker = tracker_get(i);
      bool running = tracker->busy;
      me_gnss_signal_t mesid = tracker->mesid;
      u16 glo_slot_id = tracker->glo_orbit_slot;
      float cn0 = tracker->cn0;
      bool confirmed = (0 != (tracker->flags & TRACKER_FLAG_CONFIRMED));

      if (!running || !confirmed) {
        meas_states[i].mesid = (sbp_gnss_signal_t){
            .sat = 0,
            .code = 0,
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
      cn0 = (cn0 <= 0) ? 0.0f : cn0;
      cn0 = (cn0 >= MAX_VAL_CN0) ? (float)MAX_VAL_CN0 : cn0;
      meas_states[i].cn0 = (u8)lrintf(cn0 * 4.0f);
    }
  }
  odd_run = !odd_run;
  sbp_send_msg(
      SBP_MSG_MEASUREMENT_STATE, sizeof(meas_states), (u8 *)meas_states);
}

/** Goes through all the NAP tracker channels and cleans the stale ones
 * (there should NEVER be any!)
 */
void stale_trackers_cleanup(void) {
  for (u8 i = 0; i < ME_CHANNELS; i++) {
    tracker_t *tracker = tracker_get(i);
    if (!tracker->busy) {
      continue;
    }
    if (!tracker->updated_once) {
      continue;
    }
    u64 delay_ms = tracker_timer_ms(&tracker->update_timer);
    if (delay_ms > NAP_CORR_LENGTH_MAX_MS) {
      log_error_mesid(tracker->mesid, "hit delay_ms: %" PRIu64, delay_ms);
      tracker_flag_drop(tracker, CH_DROP_REASON_NO_UPDATES);
      sanitize_tracker(tracker);
    }
  }
}
