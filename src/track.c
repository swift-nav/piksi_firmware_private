/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <ch.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>

#include "board/nap/track_channel.h"
#include "manage.h"
#include "nap/nap_constants.h"
#include "ndb.h"
#include "position.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings.h"
#include "signal.h"
#include "simulator.h"
#include "timing.h"
#include "track.h"
#include "track/track_cn0.h"
#include "track/track_sbp.h"
#include "track/track_sid_db.h"

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

#define COMPILER_BARRIER() asm volatile("" : : : "memory")

#define CHANNEL_DISABLE_WAIT_TIME_MS 100

#define MAX_VAL_CN0 (255.0 / 4.0)

#define POW_TWO_P31 2147483648.0
#define POW_TWO_P32 4294967296.0

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE,
  EVENT_DISABLE_WAIT_COMPLETE
} event_t;

static tracker_channel_t tracker_channels[NUM_TRACKER_CHANNELS];

static const tracker_interface_t tracker_interface_default = {
    .code = CODE_INVALID, .init = 0, .disable = 0, .update = 0,
};

static u16 iq_output_mask = 0;
/** send_trk_detailed setting is a stop gap to suppress this
  * bandwidth intensive msg until a more complete "debug"
  * strategy is designed and implemented. */
static bool send_trk_detailed = 0;
u16 max_pll_integration_time_ms = 20;

static WORKING_AREA_CCM(wa_nap_track_irq, 32000);

static void tracker_channel_process(tracker_channel_t *tracker_channel,
                                    bool update_required);

static bool track_iq_output_notify(struct setting *s, const char *val);
static bool max_pll_integration_time_notify(struct setting *s, const char *val);
static void nap_channel_disable(const tracker_channel_t *tracker_channel);

static const tracker_interface_t *tracker_interface_lookup(
    const me_gnss_signal_t mesid);
static bool tracker_channel_runnable(const tracker_channel_t *tracker_channel);
static state_t tracker_channel_state_get(
    const tracker_channel_t *tracker_channel);
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t *func);
static void event(tracker_channel_t *d, event_t event);
static void tracker_channel_lock(tracker_channel_t *tracker_channel);
static void tracker_channel_unlock(tracker_channel_t *tracker_channel);
static void error_flags_clear(tracker_channel_t *tracker_channel);
static void error_flags_add(tracker_channel_t *tracker_channel,
                            error_flag_t error_flag);

static void tracking_channel_compute_values(
    tracker_channel_t *tracker_channel,
    tracking_channel_info_t *info,
    tracking_channel_time_info_t *time_info,
    tracking_channel_freq_info_t *freq_info,
    tracking_channel_ctrl_info_t *ctrl_params,
    bool *reset_cpo);

static void tracking_channel_update_values(
    tracker_channel_t *tracker_channel,
    const tracking_channel_info_t *info,
    const tracking_channel_time_info_t *time_info,
    const tracking_channel_freq_info_t *freq_info,
    const tracking_channel_ctrl_info_t *ctrl_params,
    bool reset_cpo);

void nap_track_irq_thread(void *arg);

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
    tracker_channels[i].state = STATE_DISABLED;
    chMtxObjectInit(&tracker_channels[i].mutex);
    chMtxObjectInit(&tracker_channels[i].mutex_pub);
  }

  track_cn0_params_init();
  tp_init();

  platform_track_setup();

  chThdCreateStatic(wa_nap_track_irq,
                    sizeof(wa_nap_track_irq),
                    HIGHPRIO - 1,
                    nap_track_irq_thread,
                    /* arg = */ NULL);
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
        states[i].sid = (gnss_signal16_t){
            .sat = 0, .code = 0,
        };
        states[i].cn0 = 0;
      }
    }

  } else {
    u8 uMaxObs =
        (SBP_FRAMING_MAX_PAYLOAD_SIZE / sizeof(tracking_channel_state_t));
    for (u8 i = 0; (i < nap_track_n_channels) && (i < uMaxObs); i++) {
      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      bool running;
      bool confirmed;
      me_gnss_signal_t mesid;
      u16 glo_slot_id;
      float cn0;

      running = (tracker_channel_state_get(tracker_channel) == STATE_ENABLED);
      mesid = tracker_channel->mesid;
      glo_slot_id = tracker_channel->glo_orbit_slot;
      cn0 = tracker_channel->cn0;
      confirmed = (0 != (tracker_channel->flags & TRACKER_FLAG_CONFIRMED));

      if (!running || !confirmed) {
        states[i].sid = (gnss_signal16_t){
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
    tracking_channel_info_t channel_info;
    tracking_channel_freq_info_t freq_info;
    tracking_channel_time_info_t time_info;
    tracking_channel_ctrl_info_t ctrl_info;
    tracking_channel_misc_info_t misc_info;
    msg_tracking_state_detailed_t sbp;

    tracking_channel_get_values(
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

/** Handles pending IRQs and background tasks for tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        an IRQ is pending.
 */
void tracking_channels_update(u64 channels_mask) {
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(channel);
    bool update_required = (channels_mask & 1) ? true : false;
    tracker_channel_process(tracker_channel, update_required);
    channels_mask >>= 1;
  }
}

/** Sets the missed update error for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        a missed update error has occurred.
 */
void tracking_channels_missed_update_error(u64 channels_mask) {
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(channel);
    bool error = (channels_mask & 1) ? true : false;
    if (error) {
      error_flags_add(tracker_channel, ERROR_FLAG_MISSED_UPDATE);
    }
    channels_mask >>= 1;
  }
}

/** Determine if a tracker channel is available to track the specified sid.
 *
 * \param id      ID of the tracker channel to be checked.
 * \param mesid   Signal to be tracked.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
bool tracker_channel_available(tracker_channel_id_t id,
                               const me_gnss_signal_t mesid) {
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);

  if (!nap_track_supports(id, mesid)) {
    return false;
  }

  return tracker_channel_runnable(tracker_channel);
}

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param mesid        ME signal ID.
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
double propagate_code_phase(const me_gnss_signal_t mesid,
                            double code_phase,
                            double carrier_freq,
                            u32 n_samples) {
  /* Calculate the code phase rate with carrier aiding. */
  double code_phase_rate = (1.0 + carrier_freq / mesid_to_carr_freq(mesid)) *
                           code_to_chip_rate(mesid.code);
  code_phase += n_samples * code_phase_rate / NAP_FRONTEND_SAMPLE_RATE_Hz;
  u32 cp_int = floor(code_phase);
  code_phase -= cp_int - (cp_int % code_to_chip_count(mesid.code));
  return code_phase;
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
bool tracker_channel_init(tracker_channel_id_t id,
                          const me_gnss_signal_t mesid,
                          u16 glo_orbit_slot,
                          u64 ref_sample_count,
                          double code_phase,
                          float carrier_freq,
                          u32 chips_to_correlate,
                          float cn0_init) {
  tracker_channel_t *tracker_channel = tracker_channel_get(id);

  if (!tracker_channel_runnable(tracker_channel)) {
    return false;
  }

  /* Channel public data blocks */
  tracking_channel_info_t info;
  tracking_channel_time_info_t time_info;
  tracking_channel_freq_info_t freq_info;
  tracking_channel_ctrl_info_t ctrl_params;

  tracker_channel_lock(tracker_channel);
  {
    tracker_cleanup(tracker_channel);

    /* Set up channel */
    tracker_channel->mesid = mesid;
    tracker_channel->nap_channel = id;

    const tracker_interface_t *tracker_interface;
    tracker_interface = tracker_interface_lookup(mesid);
    tracker_channel->interface = tracker_interface;

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
    tracker_channel->cp_sync.counter = 0;
    tracker_channel->cp_sync.polarity = BIT_POLARITY_UNKNOWN;
    tracker_channel->cp_sync.synced = false;
    tracker_channel->health = GLO_SV_HEALTHY;

    tracker_channel->bit_polarity = BIT_POLARITY_UNKNOWN;
    tracker_channel->glo_orbit_slot = glo_orbit_slot;

    nav_bit_fifo_init(&tracker_channel->nav_bit_fifo);
    nav_data_sync_init(&tracker_channel->nav_data_sync);
    bit_sync_init(&tracker_channel->bit_sync, mesid);

    interface_function(tracker_channel, tracker_interface->init);

    /* Clear error flags before starting NAP tracking channel */
    error_flags_clear(tracker_channel);

    /* Change the channel state to ENABLED. */
    event(tracker_channel, EVENT_ENABLE);

    /* Load channel public data while in channel lock */
    tracking_channel_compute_values(
        tracker_channel, &info, &time_info, &freq_info, &ctrl_params, NULL);
  }
  tracker_channel_unlock(tracker_channel);

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
bool tracker_channel_disable(tracker_channel_id_t id) {
  /* Request disable */
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  event(tracker_channel, EVENT_DISABLE_REQUEST);
  return true;
}

/**
 * The function sets or clears PRN fail flag.
 * Called from Decoder task.
 * \param[in] mesid  ME SV ID
 * \param[in] val prn fail flag value. TRUE if decoded prn from L2C data stream
 *            is not correspond to SVID, otherwise FALSE
 */
void tracking_channel_set_prn_fail_flag(const me_gnss_signal_t mesid,
                                        bool val) {
  /* Find SV ID for L1CA and L2CM and set the flag  */
  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; id++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracker_channel_lock(tracker_channel);
    if (IS_GPS(tracker_channel->mesid) &&
        tracker_channel->mesid.sat == mesid.sat) {
      tracker_channel->prn_check_fail = val;
    }
    tracker_channel_unlock(tracker_channel);
  }
}

/**
 * Sets RAIM exclusion flag to a channel with a given signal identifier
 *
 * \param[in] sid signal identifier for channel to set
 *
 * \return None
 */
void tracking_channel_set_raim_flag(const gnss_signal_t sid) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    /* Find the corresponding channel and flag it. (Note that searching by sid
     * instead of mesid is a bit tricky.. */
    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_lock(tracker_channel);
    /* Is this channel's mesid + orbit slot combination valid? */
    bool can_compare = mesid_valid(tracker_channel->mesid);
    if (IS_GLO(tracker_channel->mesid)) {
      can_compare &= glo_slot_id_is_valid(tracker_channel->glo_orbit_slot);
    }
    if (can_compare && sid_is_equal(mesid2sid(tracker_channel->mesid,
                                              tracker_channel->glo_orbit_slot),
                                    sid)) {
      tracker_channel->flags |= TRACKER_FLAG_RAIM_EXCLUSION;
    }
    tracker_channel_unlock(tracker_channel);
  }
}

/**
 * Sets cross-correlation flag to a channel with a given ME signal identifier
 *
 * \param[in] mesid ME signal identifier for channel to set cross-correlation
 *                  flag.
 *
 * \return None
 */
void tracking_channel_set_xcorr_flag(const me_gnss_signal_t mesid) {
  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; ++id) {
    /* Find matching tracker and set the flag  */
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracker_channel_lock(tracker_channel);
    if (mesid_is_equal(tracker_channel->mesid, mesid)) {
      tracker_channel->xcorr_flag = true;
    }
    tracker_channel_unlock(tracker_channel);
  }
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

static void tracking_channel_compute_values(
    tracker_channel_t *tracker_channel,
    tracking_channel_info_t *info,
    tracking_channel_time_info_t *time_info,
    tracking_channel_freq_info_t *freq_info,
    tracking_channel_ctrl_info_t *ctrl_params,
    bool *reset_cpo) {
  if (NULL != info) {
    /* Tracker identifier */
    info->id = (tracker_channel_id_t)(tracker_channel - &tracker_channels[0]);
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
    ctrl_params->pll_bw = tracker_channel->ctrl_params.pll_bw;
    ctrl_params->fll_bw = tracker_channel->ctrl_params.fll_bw;
    ctrl_params->dll_bw = tracker_channel->ctrl_params.dll_bw;
    ctrl_params->int_ms = tracker_channel->ctrl_params.int_ms;
  }
  if (NULL != reset_cpo) {
    *reset_cpo = tracker_channel->reset_cpo;
    tracker_channel->reset_cpo = false;
  }
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
    tracker_channel_t *tracker_channel,
    const tracking_channel_info_t *info,
    const tracking_channel_time_info_t *time_info,
    const tracking_channel_freq_info_t *freq_info,
    const tracking_channel_ctrl_info_t *ctrl_params,
    bool reset_cpo) {
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

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

/**
 * Atomically loads tracking channel public informational block.
 *
 * The channel locks public informational block and loads data from it into
 * output parameters.
 *
 * \param[in]  id           Tracking channel identifier.
 * \param[out] info         Optional destination for generic information.
 * \param[out] time_info    Optional destination for timing information.
 * \param[out] freq_info    Optional destination for frequency and phase
 *                          information.
 * \param[out] ctrl_params  Optional destination for loop controller
 * information.
 * \param[out] misc_params  Optional destination for misc information.
 * \param[in] reset_stats   Reset channel statistics
 *
 * \return None
 *
 * \sa tracking_channel_update_values
 */
void tracking_channel_get_values(tracker_channel_id_t id,
                                 tracking_channel_info_t *info,
                                 tracking_channel_time_info_t *time_info,
                                 tracking_channel_freq_info_t *freq_info,
                                 tracking_channel_ctrl_info_t *ctrl_params,
                                 tracking_channel_misc_info_t *misc_params) {
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

  chMtxLock(&tracker_channel->mutex_pub);
  if (NULL != info) {
    *info = pub_data->gen_info;
  }
  if (NULL != time_info) {
    *time_info = pub_data->time_info;
  }
  if (NULL != freq_info) {
    *freq_info = pub_data->freq_info;
  }
  if (NULL != ctrl_params) {
    *ctrl_params = pub_data->ctrl_info;
  }
  if (NULL != misc_params) {
    *misc_params = pub_data->misc_info;
  }
  chMtxUnlock(&tracker_channel->mutex_pub);
}

/**
 * Atomically updates carrier phase offset.
 *
 * The method locates tracking channel object, locks it, and updates the
 * carrier phase offset only if the channel is still active, belongs to the
 * same signal and has the same lock counter.
 *
 * \param[in] info                 Generic tracking channel information block
 *                                 used for locating destination channel and
 *                                 checking integrity.
 * \param[in] carrier_phase_offset Carrier phase offset to set.
 *
 * \return None
 */
void tracking_channel_set_carrier_phase_offset(
    const tracking_channel_info_t *info, double carrier_phase_offset) {
  bool adjusted = false;
  tracker_channel_t *tracker_channel = tracker_channel_get(info->id);
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

  chMtxLock(&tracker_channel->mutex_pub);
  if (0 != (pub_data->gen_info.flags & TRACKER_FLAG_ACTIVE) &&
      mesid_is_equal(info->mesid, pub_data->gen_info.mesid) &&
      info->lock_counter == pub_data->gen_info.lock_counter) {
    pub_data->misc_info.carrier_phase_offset.value = carrier_phase_offset;
    pub_data->misc_info.carrier_phase_offset.timestamp_ms = timing_getms();
    adjusted = true;
  }
  chMtxUnlock(&tracker_channel->mutex_pub);

  if (adjusted) {
    log_debug_mesid(info->mesid,
                    "Adjusting carrier phase offset to %lf",
                    carrier_phase_offset);
  }
}
/**
 * Computes the lock time from tracking channel time info.
 *
 * \param[in]  time_info Time information block.
 * \param[in]  misc_info Miscellaneous information block.
 *
 * \return Lock time [s]
 */
double tracking_channel_get_lock_time(
    const tracking_channel_time_info_t *time_info,
    const tracking_channel_misc_info_t *misc_info) {
  u64 cpo_age_ms = 0;
  if (0 != misc_info->carrier_phase_offset.value) {
    u64 now_ms = timing_getms();
    assert(now_ms >= misc_info->carrier_phase_offset.timestamp_ms);
    cpo_age_ms = now_ms - misc_info->carrier_phase_offset.timestamp_ms;
  }

  u64 lock_time_ms = UINT64_MAX;

  lock_time_ms = MIN(lock_time_ms, time_info->ld_pess_locked_ms);
  lock_time_ms = MIN(lock_time_ms, cpo_age_ms);

  return (double)lock_time_ms / SECS_MS;
}

/**
 * Loads data relevant to cross-correlation processing
 *
 * The method loads information from all trackers for cross-correlation
 * algorithm.
 *
 * \param[out] cc_data Destination container
 *
 * \return Number of entries loaded
 *
 * \sa tracking_channel_cc_data_t
 */
u16 tracking_channel_load_cc_data(tracking_channel_cc_data_t *cc_data) {
  u16 cnt = 0;

  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; ++id) {
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracking_channel_cc_entry_t entry;

    entry.id = id;
    entry.mesid = tracker_channel->mesid;
    entry.flags = tracker_channel->flags;
    entry.freq = tracker_channel->xcorr_freq;
    entry.cn0 = tracker_channel->cn0;

    if (0 != (entry.flags & TRACKER_FLAG_ACTIVE) &&
        0 != (entry.flags & TRACKER_FLAG_CONFIRMED) &&
        0 != (entry.flags & TRACKER_FLAG_XCORR_FILTER_ACTIVE)) {
      cc_data->entries[cnt++] = entry;
    }
  }

  return cnt;
}

/**
 * Converts tracking channel data blocks into channel measurement structure.
 *
 * The method populates measurement fields according to provided values.
 *
 * \param[in]  ref_tc    Reference timing count.
 * \param[in]  info      Generic tracking channel information block.
 * \param[in]  freq_info Frequency and phase information block.
 * \param[in]  time_info Time information block.
 * \param[in]  misc_info Miscellaneous information block.
 * \param[out] meas      Pointer to output channel_measurement_t.
 *
 * \return None
 */
void tracking_channel_measurement_get(
    u64 ref_tc,
    const tracking_channel_info_t *info,
    const tracking_channel_freq_info_t *freq_info,
    const tracking_channel_time_info_t *time_info,
    const tracking_channel_misc_info_t *misc_info,
    channel_measurement_t *meas) {
  /* Update our channel measurement. */
  memset(meas, 0, sizeof(*meas));

  meas->sid = mesid2sid(info->mesid, info->glo_orbit_slot);
  meas->code_phase_chips = freq_info->code_phase_chips;
  meas->code_phase_rate = freq_info->code_phase_rate;
  meas->carrier_phase = freq_info->carrier_phase;
  meas->carrier_freq = freq_info->carrier_freq;
  meas->time_of_week_ms = info->tow_ms;
  meas->tow_residual_ns = info->tow_residual_ns;

  meas->rec_time_delta = (double)((s32)(info->sample_count - (u32)ref_tc)) /
                         NAP_FRONTEND_SAMPLE_RATE_Hz;

  meas->cn0 = info->cn0;
  meas->lock_time = tracking_channel_get_lock_time(time_info, misc_info);
  meas->time_in_track = time_info->cn0_usable_ms / 1000.0;
  meas->elevation = TRACKING_ELEVATION_UNKNOWN;
  meas->flags = 0;
}

/**
 * Computes raw pseudorange in [m]
 *
 * \param[in]  ref_tc Reference time
 * \param[in]  meas   Pre-populated channel measurement
 * \param[out] raw_pseudorange Computed pseudorange [m]
 *
 * \retval true Pseudorange is valid
 * \retval false Error in computation.
 */
bool tracking_channel_calc_pseudorange(u64 ref_tc,
                                       const channel_measurement_t *meas,
                                       double *raw_pseudorange) {
  navigation_measurement_t nav_meas, *p_nav_meas = &nav_meas;
  gps_time_t rec_time = napcount2gpstime(ref_tc);
  s8 nm_ret = calc_navigation_measurement(1, &meas, &p_nav_meas, &rec_time);
  if (nm_ret != 0) {
    log_warn_sid(meas->sid,
                 "calc_navigation_measurement() returned an error: %" PRId8,
                 nm_ret);
    return false;
  }
  *raw_pseudorange = nav_meas.raw_pseudorange;
  return true;
}

/** Adjust all carrier phase offsets with a receiver clock correction.
 * Note that as this change to carrier is equal to the change caused to
 * pseudoranges by the clock correction, the code-carrier difference does
 * not change and thus we do not reset the lock counter.
 *
 * \param dt      Receiver clock change (s)
 */
void tracking_channel_carrier_phase_offsets_adjust(double dt) {
  /* Carrier phase offsets are adjusted for all signals matching SPP criteria */
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    me_gnss_signal_t mesid;
    double carrier_phase_offset = 0;
    bool adjusted = false;

    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;
    volatile tracking_channel_misc_info_t *misc_info = &pub_data->misc_info;

    chMtxLock(&tracker_channel->mutex_pub);
    if (0 != (pub_data->gen_info.flags & TRACKER_FLAG_ACTIVE)) {
      carrier_phase_offset = misc_info->carrier_phase_offset.value;

      /* touch only channels that have the initial offset set */
      if (carrier_phase_offset != 0.0) {
        mesid = pub_data->gen_info.mesid;
        carrier_phase_offset -= mesid_to_carr_freq(mesid) * dt;
        misc_info->carrier_phase_offset.value = carrier_phase_offset;
        /* Note that because code-carrier difference does not change here,
         * we do not reset the lock time carrier_phase_offset.timestamp_ms */
        adjusted = true;
      }
    }
    chMtxUnlock(&tracker_channel->mutex_pub);

    if (adjusted) {
      log_debug_mesid(
          mesid, "Adjusting carrier phase offset to %f", carrier_phase_offset);
    }
  }
}

/** Utility function to find tracking channel allocated to the given mesid.
 *
 * \param[in] mesid ME signal identifier.
 *
 * \return tracker channel container for the requested mesid.
 */
tracker_channel_t *tracker_channel_get_by_mesid(const me_gnss_signal_t mesid) {
  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    if (mesid_is_equal(tracker_channel->mesid, mesid)) {
      return tracker_channel;
    }
  }
  return NULL;
}

/** Drop the L2CL tracker when it is no longer needed.
 *  This function can be called from both L2CM and L2CL trackers.
 *
 * \param[in] mesid ME signal identifier.
 *
 * \return None
 */
void tracking_channel_drop_l2cl(const me_gnss_signal_t mesid) {
  me_gnss_signal_t mesid_L2CL = construct_mesid(CODE_GPS_L2CL, mesid.sat);
  tracker_channel_t *tracker_channel = tracker_channel_get_by_mesid(mesid_L2CL);
  if (NULL == tracker_channel) {
    return;
  }
  /*! The barrier in manage_track() in manage.c should take care of
    * this anyway
    */
  if (STATE_ENABLED != tracker_channel_state_get(tracker_channel)) {
    return;
  }
  tracker_channel->flags |= TRACKER_FLAG_L2CL_AMBIGUITY_RESOLVED;
}

/** Drop unhealthy GLO signal.
 *
 *  Both L1CA and L2CA decode the health information independently.
 *  In case one channel does not contain valid data,
 *  it cannot detect unhealthy status.
 *
 *  If one channel is marked unhealthy,
 *  then also drop the other channel.
 *
 *  This function is called from both GLO L1 and L2 trackers.
 *
 * \param[in] mesid ME signal to be dropped.
 *
 * \return None
 */
void tracking_channel_drop_unhealthy_glo(const me_gnss_signal_t mesid) {
  assert(IS_GLO(mesid));
  tracker_channel_t *tracker_channel = tracker_channel_get_by_mesid(mesid);
  if (tracker_channel == NULL) {
    return;
  }
  /* Double-check that channel is in enabled state.
   * Similar check exists in manage_track() in manage.c
   */
  if (STATE_ENABLED != tracker_channel_state_get(tracker_channel)) {
    return;
  }
  tracker_channel->flags |= TRACKER_FLAG_GLO_HEALTH_DECODED;
  tracker_channel->health = GLO_SV_UNHEALTHY;
}

/**
 * Check validity of handover code phase.
 *
 * The code phase is expected to be near zero at the moment of handover.
 * If the code has rolled over just recently, then it is [0, TOLERANCE]
 * If the code hasn't rolled over yet, then it is [MAX_CHIPS - TOL, MAX_CHIPS)
 * Code phase should never be negative.
 *
 * \param[in] code_phase_chips code phase [chips].
 * \param[in] max_chips        code length of the signal
 *                             from where handover is done [chips].
 *
 * \return true if the code phase is valid, false otherwise.
 */
bool handover_valid(double code_phase_chips, double max_chips) {
  if ((code_phase_chips < 0) ||
      ((code_phase_chips > HANDOVER_CODE_PHASE_THRESHOLD) &&
       (code_phase_chips < (max_chips - HANDOVER_CODE_PHASE_THRESHOLD)))) {
    return false;
  }
  return true;
}

/** Set the azimuth and elevation angles for SV by sid.
 *
 * \param[in] sid       Signal identifier for which the elevation should be set.
 * \param[in] azimuth   Azimuth angle [degrees].
 * \param[in] elevation Elevation angle [degrees].
 * \param[in] timestamp Azimuth and elevation evaluation time [ticks].
 *
 * \retval true  Elevation has been successfully updated.
 * \retval false Elevation has not been updated because GNSS constellation is
 *               not supported.
 *
 * \sa sv_elevation_degrees_get
 */
bool sv_azel_degrees_set(gnss_signal_t sid,
                         u16 azimuth,
                         s8 elevation,
                         u64 timestamp) {
  tp_azel_entry_t entry = {.azimuth_d = azimuth,
                           .elevation_d = elevation,
                           .timestamp_tk = timestamp};
  return track_sid_db_update_azel(sid, &entry);
}

/** Return the azimuth angle for a satellite.
 *
 * \param[in] sid Signal identifier for which the azimuth should be returned.
 *
 * \return SV elevation in degrees, or #TRACKING_AZIMUTH_UNKNOWN.
 * \retval TRACKING_AZIMUTH_UNKNOWN Azimuth is not present in the cache,
 *                                  cache entry is too old, or GNSS
 *                                  constellation is not supported.
 *
 * \sa sv_azimuth_degrees_set
 */
u16 sv_azimuth_degrees_get(gnss_signal_t sid) {
  u16 result = TRACKING_AZIMUTH_UNKNOWN;
  tp_azel_entry_t entry = {0};
  if (track_sid_db_load_elevation(sid, &entry)) {
    /* If azimuth cache entry is loaded, do the entry age check */
    if (TRACKING_AZIMUTH_UNKNOWN != entry.azimuth_d &&
        nap_timing_count() - entry.timestamp_tk < SEC2TICK(MAX_AZ_EL_AGE_SEC)) {
      result = entry.azimuth_d;
    }
  }
  return result;
}

/** Return the elevation angle for a satellite.
 *
 * \param[in] sid Signal identifier for which the elevation should be returned.
 *
 * \return SV elevation in degrees, or #TRACKING_ELEVATION_UNKNOWN.
 * \retval TRACKING_ELEVATION_UNKNOWN Elevation is not present in the cache,
 *                                    cache entry is too old, or GNSS
 *                                    constellation is not supported.
 *
 * \sa sv_elevation_degrees_set
 */
s8 sv_elevation_degrees_get(gnss_signal_t sid) {
  s8 result = TRACKING_ELEVATION_UNKNOWN;

  tp_azel_entry_t entry = {0};
  if (track_sid_db_load_elevation(sid, &entry)) {
    /* If elevation cache entry is loaded, do the entry age check */
    if (TRACKING_ELEVATION_UNKNOWN != entry.elevation_d &&
        nap_timing_count() - entry.timestamp_tk < SEC2TICK(MAX_AZ_EL_AGE_SEC)) {
      result = entry.elevation_d;
    }
  }
  return result;
}

/** Read the next pending nav bit for a tracker channel.
 *
 * \note This function should should be called from the same thread as
 * tracking_channel_time_sync().
 *
 * \param id       ID of the tracker channel to read from.
 * \param nav_bit  Struct containing nav_bit data.
 *
 * \return true if valid nav_bit is available, false otherwise.
 */
bool tracking_channel_nav_bit_get(tracker_channel_id_t id,
                                  nav_bit_fifo_element_t *nav_bit) {
  tracker_channel_t *tracker_channel = tracker_channel_get(id);

  nav_bit_fifo_element_t element;
  if (nav_bit_fifo_read(&tracker_channel->nav_bit_fifo, &element)) {
    *nav_bit = element;
    return true;
  }
  return false;
}

/** Initializes the data structure used to sync data between decoder and tracker
 *
 * \param data_sync struct used for sync
 */
void tracking_channel_data_sync_init(nav_data_sync_t *data_sync) {
  memset(data_sync, 0, sizeof(*data_sync));
  data_sync->glo_orbit_slot = GLO_ORBIT_SLOT_UNKNOWN;
  data_sync->glo_health = GLO_SV_UNHEALTHY;
  data_sync->sync_flags = SYNC_ALL;
}

/** Propagate decoded time of week, bit polarity and optional glo orbit slot
 *  back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id           ID of the tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
static void tracking_channel_data_sync(tracker_channel_id_t id,
                                       nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  from_decoder->read_index = tracker_channel->nav_bit_fifo.read_index;
  if (!nav_data_sync_set(&tracker_channel->nav_data_sync, from_decoder)) {
    log_warn_mesid(tracker_channel->mesid, "Data sync failed");
  }
}

/** Propagate decoded GPS time of week and bit polarity back to a tracker
 * channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id           ID of the GPS tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
void tracking_channel_gps_data_sync(tracker_channel_id_t id,
                                    nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  if ((from_decoder->TOW_ms < 0) ||
      (BIT_POLARITY_UNKNOWN == from_decoder->bit_polarity)) {
    return;
  }
  tracking_channel_data_sync(id, from_decoder);
}

/** Propagate decoded GLO time of week, bit polarity and glo orbit slot
 *  back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id           ID of the GLO tracker channel to synchronize.
 * \param from_decoder struct to sync tracker with.
 */
void tracking_channel_glo_data_sync(tracker_channel_id_t id,
                                    nav_data_sync_t *from_decoder) {
  assert(from_decoder);

  tracking_channel_data_sync(id, from_decoder);
}

/** Check the state of a tracker channel and generate events as required.
 * \param tracker_channel   Tracker channel to use.
 * \param update_required   True when correlations are pending for the
 *                          tracking channel.
 */
static void tracker_channel_process(tracker_channel_t *tracker_channel,
                                    bool update_required) {
  switch (tracker_channel_state_get(tracker_channel)) {
    case STATE_ENABLED: {
      if (update_required) {
        /* Channel public data blocks for transferring between locks */
        tracking_channel_info_t info;
        tracking_channel_time_info_t time_info;
        tracking_channel_freq_info_t freq_info;
        tracking_channel_ctrl_info_t ctrl_params;
        bool reset_cpo;

        tracker_channel_lock(tracker_channel);
        {
          interface_function(tracker_channel,
                             tracker_channel->interface->update);

          /* Read channel public data while in channel lock */
          tracking_channel_compute_values(tracker_channel,
                                          &info,
                                          &time_info,
                                          &freq_info,
                                          &ctrl_params,
                                          &reset_cpo);
        }
        tracker_channel_unlock(tracker_channel);

        /* Update channel public data outside of channel lock */
        tracking_channel_update_values(tracker_channel,
                                       &info,
                                       &time_info,
                                       &freq_info,
                                       &ctrl_params,
                                       reset_cpo);
      }
    } break;

    case STATE_DISABLE_REQUESTED: {
      nap_channel_disable(tracker_channel);
      tracker_channel_lock(tracker_channel);
      {
        interface_function(tracker_channel,
                           tracker_channel->interface->disable);
        piksi_systime_get_x(&tracker_channel->disable_time);
        event(tracker_channel, EVENT_DISABLE);
      }
      tracker_channel_unlock(tracker_channel);
    } break;

    case STATE_DISABLE_WAIT: {
      if (piksi_systime_elapsed_since_ms_x(&tracker_channel->disable_time) >=
          CHANNEL_DISABLE_WAIT_TIME_MS) {
        event(tracker_channel, EVENT_DISABLE_WAIT_COMPLETE);
      }
    } break;

    case STATE_DISABLED: {
      if (update_required) {
        /* Tracking channel is not owned by the update thread, but the update
         * register must be written to clear the interrupt flag. Set error
         * flag to indicate that NAP is in an unknown state. */
        nap_channel_disable(tracker_channel);
        error_flags_add(tracker_channel, ERROR_FLAG_INTERRUPT_WHILE_DISABLED);
      }
    } break;

    default: { assert(!"Invalid tracking channel state"); } break;
  }
}

/** Return the unsigned difference between update_count and *val for a
 * tracker channel.
 *
 * \note This function allows some margin to avoid glitches in case values
 * are not read atomically from the tracking channel data.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param val               Pointer to the value to be subtracted
 *                          from update_count.
 *
 * \return The unsigned difference between update_count and *val.
 */
update_count_t update_count_diff(const tracker_channel_t *tracker_channel,
                                 const update_count_t *val) {
  update_count_t result =
      (update_count_t)(tracker_channel->update_count - *val);
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  /* Allow some margin in case values were not read atomically.
   * Treat a difference of [-10000, 0) as zero. */
  if (result > (update_count_t)(UINT32_MAX - 10000))
    return 0;
  else
    return result;
}

/** Parse the IQ output enable bitfield. */
static bool track_iq_output_notify(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NUM_TRACKER_CHANNELS; i++) {
      tracker_channel_t *tracker_channel = tracker_channel_get(i);
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

/** Disable the NAP tracking channel associated with a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void nap_channel_disable(const tracker_channel_t *tracker_channel) {
  nap_track_disable(tracker_channel->nap_channel);
}

/** Retrieve the tracker channel associated with a tracker channel ID.
 *
 * \param tracker_channel_id ID of the tracker channel to be retrieved.
 *
 * \return Associated tracker channel.
 */
tracker_channel_t *tracker_channel_get(tracker_channel_id_t id) {
  assert(id < NUM_TRACKER_CHANNELS);
  return &tracker_channels[id];
}

/** Look up the tracker interface for the specified mesid.
 *
 * \param mesid ME signal to be tracked.
 *
 * \return Associated tracker interface. May be the default interface.
 */
static const tracker_interface_t *tracker_interface_lookup(
    const me_gnss_signal_t mesid) {
  const tracker_interface_list_element_t *e = *tracker_interface_list_ptr_get();
  while (e != 0) {
    const tracker_interface_t *interface = e->interface;
    if (interface->code == mesid.code) {
      return interface;
    }
    e = e->next;
  }

  return &tracker_interface_default;
}

/** Determine if a tracker channel can be started to track the specified mesid.
 *
 * \param tracker_channel_id    ID of the tracker channel to be checked.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
static bool tracker_channel_runnable(const tracker_channel_t *tracker_channel) {
  return (tracker_channel_state_get(tracker_channel) == STATE_DISABLED);
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
static state_t tracker_channel_state_get(
    const tracker_channel_t *tracker_channel) {
  state_t state = tracker_channel->state;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return state;
}

/** Execute an interface function on a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param func              Interface function to execute.
 */
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t *func) {
  func(tracker_channel);
}

/** Update the state of a tracker channel and its associated tracker instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param event             Event to process.
 */
static void event(tracker_channel_t *tracker_channel, event_t event) {
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

/** Lock a tracker channel for exclusive access.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void tracker_channel_lock(tracker_channel_t *tracker_channel) {
  chMtxLock(&tracker_channel->mutex);
}

/** Unlock a locked tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void tracker_channel_unlock(tracker_channel_t *tracker_channel) {
  chMtxUnlock(&tracker_channel->mutex);
}

/** Clear the error flags for a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void error_flags_clear(tracker_channel_t *tracker_channel) {
  tracker_channel->flags &= ~TRACKER_FLAG_ERROR;
}

/** Add an error flag to a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param error_flag        Error flag to add.
 */
static void error_flags_add(tracker_channel_t *tracker_channel,
                            error_flag_t error_flag) {
  if (error_flag != ERROR_FLAG_NONE) {
    tracker_channel->flags |= TRACKER_FLAG_ERROR;
  }
}

static bool verify_bit_alignment(tracker_channel_t *trk_ch, u32 cycle_flags) {
  if (0 == (cycle_flags & TP_CFLAG_BSYNC_UPDATE)) {
    return FALSE;
  }

  if (!tracker_bit_aligned(trk_ch)) {
    return FALSE;
  }

  return TRUE;
}

static void read_tow_cache(tracker_channel_t *trk_ch,
                           tp_tow_entry_t *tow_entry,
                           u64 sample_time_tk) {
  gnss_signal_t sid = construct_sid(trk_ch->mesid.code, trk_ch->mesid.sat);
  track_sid_db_load_tow(sid, tow_entry);

  if (TOW_UNKNOWN == tow_entry->TOW_ms) {
    return;
  }

  /* There is a cached value */
  double error_ms = 0;
  u64 delta_tk = sample_time_tk - tow_entry->sample_time_tk;

  u8 align = tracker_bit_length_get(trk_ch);

  s32 ToW_ms = tp_tow_compute(tow_entry->TOW_ms, delta_tk, align, &error_ms);

  if (TOW_UNKNOWN == ToW_ms) {
    return;
  }

  log_debug_mesid(trk_ch->mesid,
                  "[+%" PRIu32
                  "ms]"
                  " Initializing TOW from cache [%" PRIu8
                  "ms] "
                  "delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                  trk_ch->update_count,
                  align,
                  nap_count_to_ms(delta_tk),
                  ToW_ms,
                  error_ms);

  trk_ch->TOW_ms = ToW_ms;

  if (tp_tow_is_sane(trk_ch->TOW_ms)) {
    trk_ch->flags |= TRACKER_FLAG_TOW_VALID;
  } else {
    log_error_mesid(trk_ch->mesid,
                    "[+%" PRIu32 "ms] Error TOW propagation %" PRId32,
                    trk_ch->update_count,
                    trk_ch->TOW_ms);
    trk_ch->TOW_ms = TOW_UNKNOWN;
    trk_ch->flags &= ~TRACKER_FLAG_TOW_VALID;
  }
}

/**
 * Verify ToW alignment GPS
 *
 * Current block assumes the bit sync has been reached and current
 * interval has closed a bit interval. ToW shall be aligned by bit
 * duration, which is 20ms for GPS L1 C/A / L2 C.
 *
 * \param[in]     trk_ch         Tracker channel data
 */
static void check_tow_alignment_gps(tracker_channel_t *trk_ch) {
  u8 symbol_len_ms = (CODE_GPS_L1CA == trk_ch->mesid.code)
                         ? GPS_L1CA_BIT_LENGTH_MS
                         : GPS_L2C_SYMBOL_LENGTH_MS;

  u8 tail = trk_ch->TOW_ms % symbol_len_ms;

  if (0 != tail) {
    s8 error_ms = tail < (symbol_len_ms >> 1) ? -tail : symbol_len_ms - tail;

    log_error_mesid(trk_ch->mesid,
                    "[+%" PRIu32
                    "ms] TOW error detected: "
                    "error=%" PRId8 "ms old_tow=%" PRId32,
                    trk_ch->update_count,
                    error_ms,
                    trk_ch->TOW_ms);

    /* This is rude, but safe. Do not expect it to happen normally. */
    trk_ch->flags |= TRACKER_FLAG_OUTLIER;
  }
}

/**
 * Performs ToW caching and propagation.
 *
 * GPS L1 C/A and L2 C use shared structure for ToW caching. When GPS L1 C/A
 * tracker is running, it is responsible for cache updates. Otherwise GPS L2 CM
 * tracker updates the cache. The time difference between signals is ignored
 * as small.
 *
 * GPS L2 CM tracker performs ToW update/propagation only on bit edge. This
 * makes it more robust to propagation errors.
 *
 * GPS L2 CL only reads ToW from cache and propagates it on bit edge.
 *
 * \param[in]     trk_ch         Tracker channel data
 * \param[in]     cycle_flags    Current cycle flags.
 *
 * \return None
 */
void update_tow_gps(tracker_channel_t *trk_ch, u32 cycle_flags) {
  assert(IS_GPS(trk_ch->mesid));

  if (!verify_bit_alignment(trk_ch, cycle_flags)) {
    return;
  }

  tp_tow_entry_t tow_entry;
  u64 sample_time_tk = nap_sample_time_to_count(trk_ch->sample_count);

  if (TOW_UNKNOWN != trk_ch->TOW_ms) {
    check_tow_alignment_gps(trk_ch);
  } else {
    /* TOW unkown, fetch from cache */
    read_tow_cache(trk_ch, &tow_entry, sample_time_tk);
  }

  if (CODE_GPS_L2CL == trk_ch->mesid.code) {
    /* GPS L2 CL only reads ToW from cache */
    return;
  }

  if (CODE_GPS_L2CM == trk_ch->mesid.code &&
      tracking_is_running(construct_mesid(CODE_GPS_L1CA, trk_ch->mesid.sat))) {
    /* There is GPS L1 C/A tracker for the same SV. */
    return;
  }

  if ((trk_ch->flags & TRACKER_FLAG_CONFIRMED) &&
      (TOW_UNKNOWN != trk_ch->TOW_ms) &&
      (trk_ch->cn0 >= CN0_TOW_CACHE_THRESHOLD)) {
    /* Update ToW cache:
     * - Tracker is confirmed
     * - bit edge is reached
     * - CN0 is OK
     */
    tow_entry.TOW_ms = trk_ch->TOW_ms;
    tow_entry.sample_time_tk = trk_ch->sample_count;
    gnss_signal_t sid = construct_sid(trk_ch->mesid.code, trk_ch->mesid.sat);
    track_sid_db_update_tow(sid, &tow_entry);
  }
}

static s32 propagate_tow_from_sid_db_glo(tracker_channel_t *tracker_channel,
                                         u64 sample_time_tk) {
  tracker_channel->TOW_residual_ns = 0;

  u16 glo_orbit_slot = tracker_glo_orbit_slot_get(tracker_channel);
  if (!glo_slot_id_is_valid(glo_orbit_slot)) {
    return TOW_UNKNOWN;
  }

  /* GLO slot ID is known */
  gnss_signal_t sid =
      construct_sid(tracker_channel->mesid.code, glo_orbit_slot);
  tp_tow_entry_t tow_entry;

  track_sid_db_load_tow(sid, &tow_entry);
  if (TOW_UNKNOWN == tow_entry.TOW_ms) {
    return TOW_UNKNOWN;
  }

  /* We have a cached GLO TOW */

  double error_ms = 0;
  u64 delta_tk = sample_time_tk - tow_entry.sample_time_tk;
  u8 half_bit = (GLO_L1CA_BIT_LENGTH_MS / 2);

  s32 TOW_ms = tp_tow_compute(tow_entry.TOW_ms, delta_tk, half_bit, &error_ms);

  if (TOW_UNKNOWN == TOW_ms) {
    return TOW_UNKNOWN;
  }

  log_debug_sid(sid,
                "[+%" PRIu32 "ms] Initializing TOW from cache [%" PRIu8
                "ms]"
                " delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                tracker_channel->update_count,
                half_bit,
                nap_count_to_ms(delta_tk),
                TOW_ms,
                error_ms);

  tracker_channel->TOW_residual_ns = tow_entry.TOW_residual_ns;
  if (tp_tow_is_sane(TOW_ms)) {
    tracker_channel->flags |= TRACKER_FLAG_TOW_VALID;
  } else {
    log_error_sid(sid,
                  "[+%" PRIu32 "ms] Error TOW propagation %" PRId32,
                  tracker_channel->update_count,
                  TOW_ms);
    TOW_ms = TOW_UNKNOWN;
    tracker_channel->flags &= ~TRACKER_FLAG_TOW_VALID;
  }

  return TOW_ms;
}

static void update_tow_in_sid_db_glo(tracker_channel_t *tracker_channel,
                                     u64 sample_time) {
  u16 glo_orbit_slot = tracker_glo_orbit_slot_get(tracker_channel);
  if (!glo_slot_id_is_valid(glo_orbit_slot)) {
    return;
  }

  gnss_signal_t sid =
      construct_sid(tracker_channel->mesid.code, glo_orbit_slot);

  /* Update ToW cache */
  tp_tow_entry_t tow_entry = {
      .TOW_ms = tracker_channel->TOW_ms,
      .TOW_residual_ns = tracker_channel->TOW_residual_ns,
      .sample_time_tk = sample_time};
  track_sid_db_update_tow(sid, &tow_entry);
}

/**
 * Verify ToW alignment GLO
 *
 * Current block assumes the meander sync has been reached and current
 * interval has closed a meander interval. ToW shall be aligned by meander
 * duration (half bit), which is 10ms for GLO L1CA.
 *
 * \param[in]     trk_ch         Tracker channel data
 */
static void check_tow_alignment_glo(tracker_channel_t *trk_ch) {
  u8 half_bit = (GLO_L1CA_BIT_LENGTH_MS / 2);
  u8 tail = trk_ch->TOW_ms % half_bit;
  if (0 != tail) {
    /* If this correction is needed, then there is something wrong
       either with the TOW cache update or with the meander sync */
    s8 error_ms = (tail < half_bit) ? -tail : (GLO_L1CA_BIT_LENGTH_MS - tail);

    log_error_mesid(trk_ch->mesid,
                    "[+%" PRIu32
                    "ms] TOW error detected: "
                    "error=%" PRId8 "ms old_tow=%" PRId32,
                    trk_ch->update_count,
                    error_ms,
                    trk_ch->TOW_ms);

    /* This is rude, but safe. Do not expect it to happen normally. */
    trk_ch->flags |= TRACKER_FLAG_OUTLIER;
  }
}

/**
 * Performs ToW caching and propagation.
 *
 * GLO L1 and L2 use shared structure for ToW caching. When GLO L1
 * tracker is running, it is responsible for cache updates. Otherwise GLO L2
 * tracker updates the cache. The time difference between signals is ignored
 * as small.
 *
 * \param[in]     tracker_channel Tracker channel data
 * \param[in]     cycle_flags    Current cycle flags.
 */
void update_tow_glo(tracker_channel_t *trk_ch, u32 cycle_flags) {
  assert(IS_GLO(trk_ch->mesid));

  if (!verify_bit_alignment(trk_ch, cycle_flags)) {
    return;
  }

  u64 sample_time_tk = nap_sample_time_to_count(trk_ch->sample_count);

  if (TOW_UNKNOWN != trk_ch->TOW_ms) {
    check_tow_alignment_glo(trk_ch);
  } else {
    trk_ch->TOW_ms = propagate_tow_from_sid_db_glo(trk_ch, sample_time_tk);
  }

  if (CODE_GLO_L2CA == trk_ch->mesid.code &&
      tracking_is_running(construct_mesid(CODE_GLO_L1CA, trk_ch->mesid.sat))) {
    /* corresponding GLO L1 is in track, it takes care of ToW updates, so
     * no need to continue */
    return;
  }

  if ((trk_ch->flags & TRACKER_FLAG_CONFIRMED) &&
      (TOW_UNKNOWN != trk_ch->TOW_ms) &&
      (trk_ch->cn0 >= CN0_TOW_CACHE_THRESHOLD)) {
    update_tow_in_sid_db_glo(trk_ch, sample_time_tk);
  }
}

/** \} */
