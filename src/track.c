/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
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

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ch.h>

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/run_stats.h>

#include "board/nap/track_channel.h"
#include "nap/nap_constants.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "track_api.h"
#include "track_internal.h"
#include "track/track_cn0.h"
#include "track/track_profiles.h"
#include "track/track_sid_db.h"
#include "simulator.h"
#include "settings.h"
#include "signal.h"
#include "timing.h"
#include "manage.h"
#include "position.h"
#include "ndb.h"
#include "track/track_sbp.h"

/** \defgroup tracking Tracking
 * Track satellites via interrupt driven updates to SwiftNAP tracking channels.
 * Initialize SwiftNAP tracking channels. Run loop filters and update
 * channels' code / carrier frequencies each integration period. Update
 * tracking measurements each integration period.
 * \{ */

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

#define CHANNEL_DISABLE_WAIT_TIME_ms 100
/** Maximum SV elevation age in sample ticks: 2 minutes is about 1 degree */
#define MAX_ELEVATION_AGE_TK (MINUTE_SECS * (u64)NAP_FRONTEND_SAMPLE_RATE_Hz)

typedef enum {
  STATE_DISABLED,
  STATE_ENABLED,
  STATE_DISABLE_REQUESTED,
  STATE_DISABLE_WAIT
} state_t;

typedef enum {
  EVENT_ENABLE,
  EVENT_DISABLE_REQUEST,
  EVENT_DISABLE,
  EVENT_DISABLE_WAIT_COMPLETE
} event_t;

/* Bitfield */
typedef enum {
  ERROR_FLAG_NONE =                         0x00,
  ERROR_FLAG_MISSED_UPDATE =                0x01,
  ERROR_FLAG_INTERRUPT_WHILE_DISABLED =     0x02,
} error_flag_t;

/**
 * Public data segment.
 *
 * Public data segment belongs to a tracking channel and is locked only for
 * a quick update or data fetch operations.
 *
 * The data is grouped according to functional blocks.
 */
typedef struct {
  /** Mutex used to permit atomic updates of public channel data. */
  mutex_t info_mutex;
  /** Generic info for externals */
  volatile tracking_channel_info_t      gen_info;
  /** Timing info for externals */
  volatile tracking_channel_time_info_t time_info;
  /** Frequency info for externals */
  volatile tracking_channel_freq_info_t freq_info;
  /** Controller parameters */
  volatile tracking_channel_ctrl_info_t ctrl_info;
  /** Miscellaneous parameters */
  volatile tracking_channel_misc_info_t misc_info;
  /** Carrier frequency products */
  running_stats_t                       carr_freq_stats;
  /** Pseudorange products */
  running_stats_t                       pseudorange_stats;
} tracker_channel_pub_data_t;

/** Top-level generic tracker channel. */
typedef struct {
  /** State of this channel. */
  state_t state;
  /** Time at which the channel was disabled. */
  systime_t disable_time;
  /** Error flags. May be set at any time by the tracking thread. */
  volatile error_flag_t error_flags;
  /** Info associated with this channel. */
  tracker_channel_info_t info;
  /** Data common to all tracker implementations. RW from channel interface
   * functions. RO from functions in this module. */
  tracker_common_data_t common_data;
  /** Data used by the API for all tracker implementations. RW from API
   * functions called within channel interface functions. RO from functions
   * in this module. */
  tracker_internal_data_t internal_data;
  /** Mutex used to permit atomic reads of channel data. */
  mutex_t mutex;
  /** Associated tracker interface. */
  const tracker_interface_t *interface;
  /** Associated tracker instance. */
  tracker_t *tracker;
  /** Publicly accessible data */
  tracker_channel_pub_data_t pub_data;
} tracker_channel_t;

static tracker_channel_t tracker_channels[NUM_TRACKER_CHANNELS];

static const tracker_interface_t tracker_interface_default = {
  .code =         CODE_INVALID,
  .init =         0,
  .disable =      0,
  .update =       0,
  .trackers =     0,
  .num_trackers = 0
};

static u16 iq_output_mask = 0;
static bool send_trk_detailed = 0;
/** send_trk_detailed setting is a stop gap to suppress this 
  * bandwidth intensive msg until a more complete "debug"
  * strategy is designed and implemented. */

static void tracker_channel_process(tracker_channel_t *tracker_channel,
                                     bool update_required);

static update_count_t update_count_diff(const tracker_channel_t *
                                        tracker_channel,
                                        const update_count_t *val);
static bool track_iq_output_notify(struct setting *s, const char *val);
static void nap_channel_disable(const tracker_channel_t *tracker_channel);

static tracker_channel_t * tracker_channel_get(tracker_channel_id_t id);
static const tracker_interface_t * tracker_interface_lookup(gnss_signal_t sid);
static bool tracker_channel_runnable(const tracker_channel_t *tracker_channel,
                                     gnss_signal_t sid, tracker_t **tracker,
                                     const tracker_interface_t **
                                     tracker_interface);
static bool available_tracker_get(const tracker_interface_t *tracker_interface,
                                  tracker_t **tracker);
static state_t tracker_channel_state_get(const tracker_channel_t *
                                         tracker_channel);
static bool tracker_active(const tracker_t *tracker);
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t *func);
static void event(tracker_channel_t *d, event_t event);
static void common_data_init(tracker_common_data_t *common_data,
                             u32 sample_count, float carrier_freq,
                             float cn0, code_t code);
static void public_data_init(tracker_channel_pub_data_t *pub_data);
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
                                tracker_channel_pub_data_t *pub_data,
                                const tracking_channel_info_t *info,
                                const tracking_channel_time_info_t *time_info,
                                const tracking_channel_freq_info_t *freq_info,
                                const tracking_channel_ctrl_info_t *ctrl_params,
                                bool reset_cpo);

static void tracking_channel_cleanup_values(tracker_channel_pub_data_t *pub_data);

static tracking_channel_flags_t tracking_channel_get_flags(const tracker_channel_t *tracker_channel);

/** Set up the tracking module. */
void track_setup(void)
{
  SETTING_NOTIFY("track", "iq_output_mask", iq_output_mask, TYPE_INT,
                 track_iq_output_notify);
  SETTING("track", "send_trk_detailed", send_trk_detailed, TYPE_BOOL);

  track_internal_setup();

  for (u32 i = 0; i < NUM_TRACKER_CHANNELS; i++) {
    tracker_channels[i].state = STATE_DISABLED;
    tracker_channels[i].tracker = 0;
    chMtxObjectInit(&tracker_channels[i].mutex);
    chMtxObjectInit(&tracker_channels[i].pub_data.info_mutex);
  }

  track_cn0_params_init();
  tp_init();

  platform_track_setup();
}

/** Send tracking state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_state()
{
  tracking_channel_state_t states[nap_track_n_channels];

  if (simulation_enabled_for(SIMULATION_MODE_TRACKING)) {

    u8 num_sats = simulation_current_num_sats();
    for (u8 i=0; i < num_sats; i++) {
      states[i] = simulation_current_tracking_state(i);
    }
    if (num_sats < nap_track_n_channels) {
      for (u8 i = num_sats; i < nap_track_n_channels; i++) {
        states[i].state = 0;
        states[i].sid = (sbp_gnss_signal_t){
          .code = 0,
          .sat = 0,
          .reserved = 0
        };
        states[i].cn0 = -1;
      }
    }

  } else {

    for (u8 i=0; i<nap_track_n_channels; i++) {

      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      const tracker_common_data_t *common_data = &tracker_channel->common_data;

      bool running;
      bool confirmed;
      gnss_signal_t sid;
      float cn0;

      tracker_channel_lock(tracker_channel);
      {
        running =
            (tracker_channel_state_get(tracker_channel) == STATE_ENABLED);
        sid = tracker_channel->info.sid;
        cn0 = common_data->cn0;
        confirmed = 0 != (common_data->flags & TRACK_CMN_FLAG_CONFIRMED);
      }
      tracker_channel_unlock(tracker_channel);

      if (!running || !confirmed) {
        states[i].state = 0;
        states[i].sid = (sbp_gnss_signal_t){
          .code = 0,
          .sat = 0,
          .reserved = 0
        };
        states[i].cn0 = -1;
      } else {
        states[i].state = 1;
        states[i].sid = sid_to_sbp(sid);
        states[i].cn0 = cn0;
      }
    }
  }

  sbp_send_msg(SBP_MSG_TRACKING_STATE, sizeof(states), (u8*)states);
}

/** Send tracking detailed state SBP message.
 * Send information on each tracking channel to host.
 */
void tracking_send_detailed_state(void)
{
  last_good_fix_t lgf;
  last_good_fix_t *plgf = &lgf;

  if ((NDB_ERR_NONE != ndb_lgf_read(&lgf)) && lgf.position_solution.valid) {
    plgf = NULL;
  }

  for (u8 i = 0; i < nap_track_n_channels; i++) {
    tracking_channel_info_t channel_info;
    tracking_channel_freq_info_t freq_info;
    tracking_channel_time_info_t time_info;
    tracking_channel_ctrl_info_t ctrl_info;
    tracking_channel_misc_info_t misc_info;
    msg_tracking_state_detailed_t sbp;

    tracking_channel_get_values(i,
                                &channel_info,
                                &time_info, /* time info */
                                &freq_info,
                                &ctrl_info,
                                &misc_info, /* misc parameters */
                                true);      /* reset statistics */

    if (0 == (channel_info.flags & TRACKING_CHANNEL_FLAG_ACTIVE) ||
        0 == (channel_info.flags & TRACKING_CHANNEL_FLAG_CONFIRMED)) {
      continue;
    }

    track_sbp_get_detailed_state(&sbp,
                                 &channel_info,
                                 &freq_info,
                                 &time_info,
                                 &ctrl_info,
                                 &misc_info,
                                 plgf);
    if (send_trk_detailed) {
      sbp_send_msg(SBP_MSG_TRACKING_STATE_DETAILED, sizeof(sbp), (u8*)&sbp);
    }
  }
}

/** Handles pending IRQs for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        an IRQ is pending.
 */
void tracking_channels_update(u32 channels_mask)
{
  /* For each tracking channel, call tracking_channel_process(). Indicate
   * that an update is required if the corresponding bit is set in
   * channels_mask.
   */
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(channel);
    bool update_required = (channels_mask & 1) ? true : false;
    if (update_required) {
      tracker_channel_process(tracker_channel, true);
    }
    channels_mask >>= 1;
  }
}

/** Handles background tasks for all tracking channels.
 */
void tracking_channels_process(void)
{
  for (u32 channel = 0; channel < nap_track_n_channels; channel++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(channel);
    tracker_channel_process(tracker_channel, false);
  }
}

/** Sets the missed update error for the specified tracking channels.
 * \param channels_mask   Bitfield indicating the tracking channels for which
 *                        a missed update error has occurred.
 */
void tracking_channels_missed_update_error(u32 channels_mask)
{
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
 * \param sid     Signal to be tracked.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
bool tracker_channel_available(tracker_channel_id_t id, gnss_signal_t sid)
{
  const tracker_channel_t *tracker_channel = tracker_channel_get(id);

  tracker_t *tracker;
  const tracker_interface_t *tracker_interface;
  return tracker_channel_runnable(tracker_channel, sid, &tracker,
                                  &tracker_interface);
}

/** Calculate the future code phase after N samples.
 * Calculate the expected code phase in N samples time with carrier aiding.
 *
 * \param code_phase   Current code phase in chips.
 * \param carrier_freq Current carrier frequency (i.e. Doppler) in Hz used for
 *                     carrier aiding.
 * \param n_samples    N, the number of samples to propagate for.
 *
 * \return The propagated code phase in chips.
 */
double propagate_code_phase(double code_phase, double carrier_freq,
                                   u32 n_samples, code_t code)
{
  /* Calculate the code phase rate with carrier aiding. */
  double code_phase_rate = (1.0 + carrier_freq / code_to_carr_freq(code)) *
                           code_to_chip_rate(code);
  code_phase += n_samples * code_phase_rate / NAP_FRONTEND_SAMPLE_RATE_Hz;
  u32 cp_int = floor(code_phase);
  code_phase -= cp_int - (cp_int % code_to_chip_count(code));
  return code_phase;
}

/** Initialize a tracker channel to track the specified sid.
 *
 * \param id                    ID of the tracker channel to be initialized.
 * \param sid                   Signal to be tracked.
 * \param ref_sample_count      NAP sample count at which code_phase was acquired.
 * \param code_phase            Code phase
 * \param carrier_freq          Carrier frequency Doppler (Hz).
 * \param chips_to_correlate    Chips to correlate.
 * \param cn0_init              Initial C/N0 estimate (dBHz).
 *
 * \return true if the tracker channel was initialized, false otherwise.
 */
bool tracker_channel_init(tracker_channel_id_t id, gnss_signal_t sid,
                          u32 ref_sample_count, double code_phase,
                          float carrier_freq, u32 chips_to_correlate,
                          float cn0_init)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);

  const tracker_interface_t *tracker_interface;
  tracker_t *tracker;
  if(!tracker_channel_runnable(tracker_channel, sid, &tracker,
                               &tracker_interface)) {
    return false;
  }

  /* Channel public data blocks */
  tracking_channel_info_t info;
  tracking_channel_time_info_t time_info;
  tracking_channel_freq_info_t freq_info;
  tracking_channel_ctrl_info_t ctrl_params;

  tracker_channel_lock(tracker_channel);
  {
    /* Set up channel */
    tracker_channel->info.sid = sid;
    tracker_channel->info.context = tracker_channel;
    tracker_channel->info.nap_channel = id;
    tracker_channel->interface = tracker_interface;
    tracker_channel->tracker = tracker;

    common_data_init(&tracker_channel->common_data, ref_sample_count,
                     carrier_freq, cn0_init, sid.code);

    public_data_init(&tracker_channel->pub_data);

    internal_data_init(&tracker_channel->internal_data, sid);
    interface_function(tracker_channel, tracker_interface->init);

    /* Clear error flags before starting NAP tracking channel */
    error_flags_clear(tracker_channel);

    /* Change the channel state to ENABLED. */
    event(tracker_channel, EVENT_ENABLE);

    /* Load channel public data while in channel lock */
    tracking_channel_compute_values(tracker_channel,
                                    &info,
                                    &time_info,
                                    &freq_info,
                                    &ctrl_params,
                                    NULL);

  }
  tracker_channel_unlock(tracker_channel);

  nap_track_init(tracker_channel->info.nap_channel, sid, ref_sample_count,
                 carrier_freq, code_phase, chips_to_correlate);

  /* Update channel public data outside of channel lock */
  tracking_channel_update_values(&tracker_channel->pub_data,
                                 &info,
                                 &time_info,
                                 &freq_info,
                                 &ctrl_params,
                                 true);

  return true;
}

/** Disable the specified tracker channel.
 *
 * \param id      ID of the tracker channel to be disabled.
 *
 * \return true if the tracker channel was disabled, false otherwise.
 */
bool tracker_channel_disable(tracker_channel_id_t id)
{
  /* Request disable */
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  event(tracker_channel, EVENT_DISABLE_REQUEST);
  return true;
}

/**
 * The function sets or clears PRN fail flag.
 * Called from Decoder task.
 * \param[in] sid  SV ID
 * \param[in] val prn fail flag value. TRUE if decoded prn from L2C data stream
 *            is not correspond to SVID, otherwise FALSE
 */
void tracking_channel_set_prn_fail_flag(gnss_signal_t sid, bool val)
{
  /* Find SV ID for L1CA and L2CM and set the flag  */
  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; id++) {
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracker_channel_lock(tracker_channel);
    if (sid_to_constellation(tracker_channel->info.sid) == CONSTELLATION_GPS
        && tracker_channel->info.sid.sat == sid.sat) {
      tracker_internal_data_t *internal_data = &tracker_channel->internal_data;
      internal_data->prn_check_fail = val;
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
                                      bool *reset_cpo)
{
  const tracker_common_data_t *common_data = &tracker_channel->common_data;

  if (NULL != info) {
    /* Tracker identifier */
    info->id = (tracker_channel_id_t)(tracker_channel - &tracker_channels[0]);
    /* Translate/expand flags from tracker internal scope */
    info->flags = tracking_channel_get_flags(tracker_channel);
    /* Signal identifier */
    info->sid = tracker_channel->info.sid;
    /* Current C/N0 [dB/Hz] */
    info->cn0 = common_data->cn0;
    /* Current time of week for a tracker channel [ms] */
    info->tow_ms = common_data->TOW_ms;
    /* Tracking channel init time [ms] */
    info->init_timestamp_ms = common_data->init_timestamp_ms;
    /* Tracking channel update time [ms] */
    info->update_timestamp_ms = common_data->update_timestamp_ms;
    /* Lock counter */
    info->lock_counter = tracker_channel->internal_data.lock_counter;
    /* Sample counter */
    info->sample_count = common_data->sample_count;
    /* Cross-correlation doppler frequency [hz] */
    info->xcorr_freq = common_data->xcorr_freq;
  }
  if (NULL != time_info) {
    time_info->cn0_drop_ms = update_count_diff(tracker_channel,
                                               &common_data->cn0_above_drop_thres_count);
    time_info->cn0_usable_ms = update_count_diff(tracker_channel,
                                                 &common_data->cn0_below_use_thres_count);
    time_info->last_mode_change_ms = update_count_diff(tracker_channel,
                                                       &common_data->mode_change_count);

    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_PLOCK)) {
      time_info->ld_pess_locked_ms = update_count_diff(tracker_channel,
                                                       &common_data->ld_pess_change_count);
    } else {
      time_info->ld_pess_locked_ms = 0;
    }

    /* The time in ms for which the FLL/PLL pessimistic lock detector has reported
     * being unlocked for a tracker channel.
     *
     * If tracker channel is run by FLL, then time of absence of FLL pessimistic
     * lock is reported.
     * If tracker channel is run by PLL, then time of absence of PLL pessimistic
     * lock is reported.
     */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_PLOCK) ||
        0 != (common_data->flags & TRACK_CMN_FLAG_HAS_FLOCK)) {
      time_info->ld_pess_unlocked_ms = 0;
    } else {
      time_info->ld_pess_unlocked_ms = update_count_diff(tracker_channel,
                                                         &common_data->ld_pess_change_count);
    }
  }
  if (NULL != freq_info) {
    /* Current carrier frequency for a tracker channel. */
    freq_info->carrier_freq = common_data->carrier_freq;
    /* Carrier frequency snapshot at the moment of latest PLL/FLL pessimistic lock
     * condition for a tracker channel.
     *
     * The returned carrier frequency is not necessarily the latest reading of the
     * carrier frequency. It is the latest carrier frequency snapshot, when the
     * tracking channel was in PLL/FLL pessimistic lock state.
     */
    freq_info->carrier_freq_at_lock = common_data->carrier_freq_at_lock;
    /* Current carrier frequency for a tracker channel. */
    freq_info->carrier_phase = common_data->carrier_phase;
    /* Code phase in chips */
    freq_info->code_phase_chips = common_data->code_phase_prompt;
    /* Code phase rate in chips/s */
    freq_info->code_phase_rate = common_data->code_phase_rate;
    /* Acceleration [g] */
    freq_info->acceleration = common_data->acceleration;
  }
  if (NULL != ctrl_params) {
    /* Copy loop controller parameters */
    ctrl_params->pll_bw = common_data->ctrl_params.pll_bw;
    ctrl_params->fll_bw = common_data->ctrl_params.fll_bw;
    ctrl_params->dll_bw = common_data->ctrl_params.dll_bw;
    ctrl_params->int_ms = common_data->ctrl_params.int_ms;
  }
  if (NULL != reset_cpo) {
    *reset_cpo = tracker_channel->internal_data.reset_cpo;
    tracker_channel->internal_data.reset_cpo = false;
  }
}

/**
 * Atomically cleans up all public data blocks.
 *
 * \param[in,out] pub_data Public data container.
 *
 * \return None.
 */
static void tracking_channel_cleanup_values(tracker_channel_pub_data_t *pub_data)
{
  chMtxLock(&pub_data->info_mutex);
  memset((void*)&pub_data->gen_info, 0, sizeof(pub_data->gen_info));
  memset((void*)&pub_data->time_info, 0, sizeof(pub_data->time_info));
  memset((void*)&pub_data->freq_info, 0, sizeof(pub_data->freq_info));
  memset((void*)&pub_data->ctrl_info, 0, sizeof(pub_data->ctrl_info));
  memset((void*)&pub_data->misc_info, 0, sizeof(pub_data->misc_info));
  memset((void*)&pub_data->carr_freq_stats, 0, sizeof(pub_data->carr_freq_stats));
  memset((void*)&pub_data->pseudorange_stats, 0, sizeof(pub_data->pseudorange_stats));
  chMtxUnlock(&pub_data->info_mutex);
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
 * \param[in,out] pub_data    Channel public data container.
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
                                tracker_channel_pub_data_t *pub_data,
                                const tracking_channel_info_t *info,
                                const tracking_channel_time_info_t *time_info,
                                const tracking_channel_freq_info_t *freq_info,
                                const tracking_channel_ctrl_info_t *ctrl_params,
                                bool reset_cpo)
{
  double raw_pseudorange = 0;

  if (0 != (info->flags & TRACKING_CHANNEL_FLAG_TOW) &&
      0 != (info->flags & TRACKING_CHANNEL_FLAG_ACTIVE) &&
      0 != (info->flags & TRACKING_CHANNEL_FLAG_NO_ERROR) &&
      time_quality >= TIME_FINE) {
    u64 ref_tc = nap_sample_time_to_count(info->sample_count);

    channel_measurement_t meas;
    const channel_measurement_t *c_meas = &meas;
    tracking_channel_measurement_get(ref_tc, info, freq_info, time_info,
             (tracking_channel_misc_info_t*)&pub_data->misc_info, &meas);
    tracking_channel_calc_pseudorange(ref_tc, c_meas, &raw_pseudorange);
  }

  chMtxLock(&pub_data->info_mutex);
  pub_data->gen_info = *info;
  pub_data->time_info = *time_info;
  pub_data->freq_info = *freq_info;
  running_stats_update(&pub_data->carr_freq_stats, freq_info->carrier_freq);
  if (reset_cpo) {
    /* Do CPO reset */
    /* no need to update timestamp for zero offset as it keeps count
       time on this channel at zero anyways */
    pub_data->misc_info.carrier_phase_offset.value = 0;
  }
  pub_data->ctrl_info = *ctrl_params;
  if (raw_pseudorange != 0) {
    pub_data->gen_info.flags |= TRACKING_CHANNEL_FLAG_PSEUDORANGE;
    running_stats_update(&pub_data->pseudorange_stats, raw_pseudorange);
  }
  pub_data->misc_info.pseudorange = raw_pseudorange;
  chMtxUnlock(&pub_data->info_mutex);
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
 * \param[out] ctrl_params  Optional destination for loop controller information.
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
                                 tracking_channel_misc_info_t *misc_params,
                                 bool reset_stats)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;
  running_stats_t carr_freq_stats;
  running_stats_t pseudorange_stats;

  chMtxLock(&pub_data->info_mutex);
  if (NULL != info) {
    *info = pub_data->gen_info;
  }
  if (NULL != time_info) {
    *time_info = pub_data->time_info;
  }
  if (NULL != freq_info) {
    carr_freq_stats = pub_data->carr_freq_stats;
    *freq_info = pub_data->freq_info;
  }
  if (NULL != ctrl_params) {
    *ctrl_params = pub_data->ctrl_info;
  }
  if (NULL != misc_params) {
    pseudorange_stats = pub_data->pseudorange_stats;
    *misc_params = pub_data->misc_info;
  }
  if (reset_stats) {
    running_stats_init(&pub_data->carr_freq_stats);
    running_stats_init(&pub_data->pseudorange_stats);
  }
  chMtxUnlock(&pub_data->info_mutex);

  if (NULL != freq_info) {
    running_stats_get_products(&carr_freq_stats,
                               NULL,
                               &freq_info->carrier_freq_std);
  }
  if (NULL != misc_params) {
    running_stats_get_products(&pseudorange_stats,
                               NULL,
                               &misc_params->pseudorange_std);
  }
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
void tracking_channel_set_carrier_phase_offset(const tracking_channel_info_t *info,
                                               double carrier_phase_offset)
{
  bool adjusted = false;
  tracker_channel_t *tracker_channel = tracker_channel_get(info->id);
  tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

  chMtxLock(&pub_data->info_mutex);
  if (0 != (pub_data->gen_info.flags & TRACKING_CHANNEL_FLAG_ACTIVE) &&
      sid_is_equal(info->sid, pub_data->gen_info.sid) &&
      info->lock_counter == pub_data->gen_info.lock_counter) {
    pub_data->misc_info.carrier_phase_offset.value = carrier_phase_offset;
    pub_data->misc_info.carrier_phase_offset.timestamp_ms = timing_getms();
    adjusted = true;
  }
  chMtxUnlock(&pub_data->info_mutex);

  if (adjusted) {
    log_debug_sid(info->sid, "Adjusting carrier phase offset to %lf",
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
  const tracking_channel_misc_info_t *misc_info)
{
  /* Carrier phase flag is set only once it has been TRACK_STABILIZATION_T
   * time since the last mode change */
  u64 stable_ms = 0;
  if (time_info->last_mode_change_ms > TRACK_STABILIZATION_T) {
    stable_ms = time_info->last_mode_change_ms - TRACK_STABILIZATION_T;
  }

  u64 cpo_age_ms = 0;
  if (0 != misc_info->carrier_phase_offset.value) {
    u64 now_ms = timing_getms();
    assert(now_ms >= misc_info->carrier_phase_offset.timestamp_ms);
    cpo_age_ms = now_ms - misc_info->carrier_phase_offset.timestamp_ms;
  }

  u64 lock_time_ms = UINT64_MAX;

  lock_time_ms = MIN(lock_time_ms, time_info->ld_pess_locked_ms);
  lock_time_ms = MIN(lock_time_ms, stable_ms);
  lock_time_ms = MIN(lock_time_ms, cpo_age_ms);

  return (double) lock_time_ms / SECS_MS;
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
u16 tracking_channel_load_cc_data(tracking_channel_cc_data_t *cc_data)
{
  u16 cnt = 0;

  for (tracker_channel_id_t id = 0; id < NUM_TRACKER_CHANNELS; ++id) {
    tracker_channel_t *tracker_channel = tracker_channel_get(id);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

    tracking_channel_cc_entry_t entry;

    entry.id = id;
    chMtxLock(&pub_data->info_mutex);
    entry.sid = pub_data->gen_info.sid;
    entry.flags = pub_data->gen_info.flags;
    entry.freq = pub_data->gen_info.xcorr_freq;
    entry.cn0 = pub_data->gen_info.cn0;
    entry.count = pub_data->gen_info.xcorr_count;
    entry.wl = pub_data->gen_info.xcorr_wl;

    chMtxUnlock(&pub_data->info_mutex);

    if (0 != (entry.flags & TRACKING_CHANNEL_FLAG_ACTIVE) &&
        0 != (entry.flags & TRACKING_CHANNEL_FLAG_CONFIRMED) &&
        0 != (entry.flags & TRACKING_CHANNEL_FLAG_XCORR_FILTER_ACTIVE)) {
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
void tracking_channel_measurement_get(u64 ref_tc,
                                      const tracking_channel_info_t *info,
                                      const tracking_channel_freq_info_t *freq_info,
                                      const tracking_channel_time_info_t *time_info,
                                      const tracking_channel_misc_info_t *misc_info,
                                      channel_measurement_t *meas)
{
  /* Update our channel measurement. */
  memset(meas, 0, sizeof(*meas));

  meas->sid = info->sid;
  meas->code_phase_chips = freq_info->code_phase_chips;
  meas->code_phase_rate = freq_info->code_phase_rate;
  meas->carrier_phase = freq_info->carrier_phase;
  meas->carrier_freq = freq_info->carrier_freq;
  meas->time_of_week_ms = info->tow_ms;
  meas->rec_time_delta = (double)((s32)(info->sample_count - (u32)ref_tc))
                             / NAP_FRONTEND_SAMPLE_RATE_Hz;
  meas->cn0 = info->cn0;
  meas->lock_time = tracking_channel_get_lock_time(time_info, misc_info);
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
                                       double *raw_pseudorange)
{
  navigation_measurement_t nav_meas, *p_nav_meas = &nav_meas;
  gps_time_t rec_time = napcount2gpstime(ref_tc);
  s8 nm_ret = calc_navigation_measurement(1,
                                          &meas,
                                          &p_nav_meas,
                                          &rec_time);
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
    gnss_signal_t sid;
    double        carrier_phase_offset = 0.;
    bool          adjusted = false;

    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;
    volatile tracking_channel_misc_info_t * misc_info = &pub_data->misc_info;

    chMtxLock(&pub_data->info_mutex);
    if (0 != (pub_data->gen_info.flags & TRACKING_CHANNEL_FLAG_ACTIVE)) {
      carrier_phase_offset = misc_info->carrier_phase_offset.value;

      /* touch only channels that have the initial offset set */
      if (carrier_phase_offset != 0.0) {
        sid = pub_data->gen_info.sid;
        carrier_phase_offset -= code_to_carr_freq(sid.code) * dt;
        misc_info->carrier_phase_offset.value = carrier_phase_offset;
        /* Note that because code-carrier difference does not change here,
         * we do not reset the lock time carrier_phase_offset.timestamp_ms */
        adjusted = true;
      }
    }
    chMtxUnlock(&pub_data->info_mutex);

    if (adjusted) {
      log_debug_sid(sid, "Adjusting carrier phase offset to %f",
                   carrier_phase_offset);
    }
  }
}

/** Update carrier phase and TOW tag.
 *  Previous reading is saved to ensure one matching pair between
 *  L2CM and L2CL trackers.
 *  This function is called from both L2CM and L2CL trackers.
 *
 * \param[in] sid GNSS signal identifier.
 * \param[in] cp  Carrier phase [cycles].
 * \param[in] TOW Time of Week tag [ms].
 *
 * \return None
 */
void tracking_channel_cp_sync_update(gnss_signal_t sid, double cp, s32 TOW)
{
  for (u8 i = 0; i < nap_track_n_channels; i++) {

    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

    chMtxLock(&pub_data->info_mutex);
    if (sid_is_equal(pub_data->gen_info.sid, sid)) {
      /* Save previous information */
      pub_data->gen_info.tow_ms_prev = pub_data->gen_info.tow_ms;
      pub_data->freq_info.carrier_phase_prev = pub_data->freq_info.carrier_phase;
      /* Save new values */
      pub_data->gen_info.tow_ms = TOW;
      pub_data->freq_info.carrier_phase = cp;
    }
    chMtxUnlock(&pub_data->info_mutex);
  }
}

/** Load carrier phase and TOW tags for comparison.
 *  This function is called from the L2CL tracker.
 *
 * \param[in]     sid          GNSS signal identifier.
 * \param[in,out] own_cp       Current own carrier phase [cycles mod 1.0].
 * \param[in,out] parent_cp    Current parent carrier phase [cycles mod 1.0].
 * \param[in,out] own_cp_p     Previous own carrier phase [cycles mod 1.0].
 * \param[in,out] parent_cp_p  Previous parent carrier phase [cycles mod 1.0].
 * \param[in,out] own_TOW      Current own Time of Week tag [ms].
 * \param[in,out] parent_TOW   Current parent Time of Week tag [ms].
 * \param[in,out] own_TOW_p    Previous own Time of Week tag [ms].
 * \param[in,out] parent_TOW_p Previous parent Time of Week tag [ms].
 * \param[in,out] count        Carrier phase match counter.
 *
 * \return True if parent L2CM data was found
 *  and did not have half-cycle ambiguity resolved.
 *  False, otherwise. Drop L2CL in case of False.
 */
bool tracking_channel_load_data(gnss_signal_t sid,
                                float *own_cp, float *parent_cp,
                                float *own_cp_p, float *parent_cp_p,
                                s32 *own_TOW, s32 *parent_TOW,
                                s32 *own_TOW_p, s32 *parent_TOW_p,
                                u8 *count)
{

  bool parent_synced = false;
  bool parent_found = false;

  for (u8 i = 0; i < nap_track_n_channels; i++) {

    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

    chMtxLock(&pub_data->info_mutex);
    if (sid_is_equal(pub_data->gen_info.sid, sid)) {
      /* Load own information */
      *own_TOW = pub_data->gen_info.tow_ms;
      *own_cp = fmod(pub_data->freq_info.carrier_phase, 1.0f);
      *own_TOW_p = pub_data->gen_info.tow_ms_prev;
      *own_cp_p = fmod(pub_data->freq_info.carrier_phase_prev, 1.0f);
      *count = pub_data->misc_info.cp_sync.counter;
    } else if (pub_data->gen_info.sid.code == CODE_GPS_L2CM &&
               pub_data->gen_info.sid.sat == sid.sat) {
      /* Load L2CM parent information */
      *parent_TOW = pub_data->gen_info.tow_ms;
      *parent_cp = fmod(pub_data->freq_info.carrier_phase, 1.0f);
      *parent_TOW_p = pub_data->gen_info.tow_ms_prev;
      *parent_cp_p = fmod(pub_data->freq_info.carrier_phase_prev, 1.0f);
      parent_synced = pub_data->misc_info.cp_sync.synced;
      parent_found = true;
    }
    chMtxUnlock(&pub_data->info_mutex);
  }

  if (!parent_found || parent_synced) {
    /* If L2CM parent was not found or
     * parent has half-cycle ambiguity resolved, then return false. */
    return false;
  } else {
    /* Otherwise data is ok. */
    return true;
  }
}

/** Compare carrier phase information and find ones with matching TOW.
 *  This function is called from the L2CL tracker.
 *
 * \param[in]     sid            GNSS signal identifier.
 * \param[in]     own_cp         Current own carrier phase [cycles mod 1.0].
 * \param[in]     parent_cp      Current parent carrier phase [cycles mod 1.0].
 * \param[in]     own_cp_p       Previous own carrier phase [cycles mod 1.0].
 * \param[in]     parent_cp_p    Previous parent carrier phase [cycles mod 1.0].
 * \param[in]     own_TOW        Current own Time of Week tag [ms].
 * \param[in]     parent_TOW     Current parent Time of Week tag [ms].
 * \param[in]     own_TOW_p      Previous own Time of Week tag [ms].
 * \param[in]     parent_TOW_p   Previous parent Time of Week tag [ms].
 * \param[in,out] own_cp_test    Matching own carrier phase [cycles mod 1.0].
 * \param[in,out] parent_cp_test Matching parent carrier phase [cycles mod 1.0].
 *
 * \return True if matching data was found, False otherwise.
 */
bool tracking_channel_find_matching_tow(gnss_signal_t sid,
                                        float own_cp, float parent_cp,
                                        float own_cp_p, float parent_cp_p,
                                        s32 own_TOW, s32 parent_TOW,
                                        s32 own_TOW_p, s32 parent_TOW_p,
                                        float *own_cp_test,
                                        float *parent_cp_test)
{
  /* Find matching TOW and save the carrier phase information. */
  bool TOW_match = true;
  if (own_TOW == parent_TOW) {
    *own_cp_test = own_cp;
    *parent_cp_test = parent_cp;
  } else if (own_TOW == parent_TOW_p) {
    *own_cp_test = own_cp;
    *parent_cp_test = parent_cp_p;
  } else if (own_TOW_p == parent_TOW) {
    *own_cp_test = own_cp_p;
    *parent_cp_test = parent_cp;
  } else if (own_TOW_p == parent_TOW_p) {
    *own_cp_test = own_cp_p;
    *parent_cp_test = parent_cp_p;
  } else {
    /* No TOW was matching. */
    TOW_match = false;
  }

  if (!TOW_match) {
    /* If no TOW was matching, reset the counter to zero. */
    for (u8 i = 0; i < nap_track_n_channels; i++) {

      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

      chMtxLock(&pub_data->info_mutex);
      if (sid_is_equal(pub_data->gen_info.sid, sid)) {
        pub_data->misc_info.cp_sync.counter = 0;
        pub_data->misc_info.cp_sync.polarity = BIT_POLARITY_UNKNOWN;
      }
      chMtxUnlock(&pub_data->info_mutex);
    }
  }
  return TOW_match;
}

/** Compare carrier phase information
 *  and find ones with close to zero, or 0.5 cycle difference.
 *  This function is called from the L2CL tracker.
 *
 * \param[in]     sid            GNSS signal identifier.
 * \param[in]     own_cp_test    Own carrier phase [cycles mod 1.0].
 * \param[in]     parent_cp_test Parent carrier phase [cycles mod 1.0].
 * \param[in,out] polarity       Polarity of the carrier phase match.
 *
 * \return True if matching data was found, False otherwise.
 */
bool tracking_channel_compare_cp(gnss_signal_t sid,
                                 float own_cp_test,
                                 float parent_cp_test,
                                 s8 *polarity)
{
  bool match = false;
  bool same_sign = false;
  float test_metric = fabsf(own_cp_test - parent_cp_test);

  /* Depending on the sign of the input carrier phase, there are
   * different tests for the test_metric.
   *
   * If the sign of the input data is same, then the matching test_metric is
   * either close to 0.0, or close to 0.5 [cycles].
   *
   * If signs are different, then matching test_metric can have values
   * close to 0.5, 1.0 or 1.5 [cycles]. */

  /* Check the signs. */
  if ((own_cp_test >= 0.0f && parent_cp_test >= 0.0f) ||
      (own_cp_test < 0.0f && parent_cp_test < 0.0f)) {
    same_sign = true;
  }

  *polarity = BIT_POLARITY_UNKNOWN;
  if (same_sign) {
    if (test_metric < 0.0f + CARRIER_PHASE_TOLERANCE &&
        test_metric > 0.0f - CARRIER_PHASE_TOLERANCE) {
      match = true;
      *polarity = BIT_POLARITY_INVERTED;
    } else if (test_metric < 0.5f + CARRIER_PHASE_TOLERANCE &&
        test_metric > 0.5f - CARRIER_PHASE_TOLERANCE) {
      match = true;
      *polarity = BIT_POLARITY_NORMAL;
    }
  } else {
    if (test_metric < 1.0f + CARRIER_PHASE_TOLERANCE &&
        test_metric > 1.0f - CARRIER_PHASE_TOLERANCE) {
      match = true;
      *polarity = BIT_POLARITY_INVERTED;
    } else if (test_metric < 0.5f + CARRIER_PHASE_TOLERANCE &&
        test_metric > 0.5f - CARRIER_PHASE_TOLERANCE) {
      match = true;
      *polarity = BIT_POLARITY_NORMAL;
    } else if (test_metric < 1.5f + CARRIER_PHASE_TOLERANCE &&
        test_metric > 1.5f - CARRIER_PHASE_TOLERANCE) {
      match = true;
      *polarity = BIT_POLARITY_NORMAL;
    }
  }

  if (!match) {
    /* If carrier phases were not matching, reset the counter to zero. */
    for (u8 i = 0; i < nap_track_n_channels; i++) {

      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

      chMtxLock(&pub_data->info_mutex);
      if (sid_is_equal(pub_data->gen_info.sid, sid)) {
        pub_data->misc_info.cp_sync.counter = 0;
        pub_data->misc_info.cp_sync.polarity = BIT_POLARITY_UNKNOWN;
      }
      chMtxUnlock(&pub_data->info_mutex);
    }
  }
  return match;
}

/** Increment counter when matching carrier phase has been found.
 *  If counter reaches maximum, declare phase sync, save polarity
 *  and drop L2CL tracker.
 *  This function is called from the L2CL tracker.
 *
 * \param[in] sid      GNSS signal identifier.
 * \param[in] count    Carrier phase match counter.
 * \param[in] polarity Polarity of the carrier phase match.
 *
 * \return None
 */
void tracking_channel_increment_cp_counter(gnss_signal_t sid,
                                           u8 count, s8 polarity)

{
  if (count < CARRIER_PHASE_AMBIGUITY_COUNTER) {
    /* If counter is below maximum, only increment it. */
    for (u8 i = 0; i < nap_track_n_channels; i++) {

      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

      chMtxLock(&pub_data->info_mutex);
      if (sid_is_equal(pub_data->gen_info.sid, sid)) {
        pub_data->misc_info.cp_sync.counter += 1;
      }
      chMtxUnlock(&pub_data->info_mutex);
    }
  } else {
    /* If counter reached maximum. */
    for (u8 i = 0; i < nap_track_n_channels; i++) {

      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

      chMtxLock(&pub_data->info_mutex);
      if (sid_is_equal(pub_data->gen_info.sid, sid)) {
        /* Drop L2CL tracker. */
        pub_data->misc_info.cp_sync.drop = true;
      } else if (pub_data->gen_info.sid.code == CODE_GPS_L2CM &&
                 pub_data->gen_info.sid.sat == sid.sat) {
        /* Update L2CM polarity and sync. */
        pub_data->misc_info.cp_sync.polarity = polarity;
        pub_data->misc_info.cp_sync.synced = true;
      }
      chMtxUnlock(&pub_data->info_mutex);
    }
  }
}

/** Drop the L2CL tracker when it is no longer needed.
 *  This function can be called from both L2CM and L2CL trackers.
 *
 * \param[in] sid GNSS signal identifier.
 *
 * \return None
 */
void tracking_channel_drop_l2cl(gnss_signal_t sid)
{
  for (u8 i = 0; i < nap_track_n_channels; i++) {

    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

    chMtxLock(&pub_data->info_mutex);
    if (pub_data->gen_info.sid.code == CODE_GPS_L2CL &&
        pub_data->gen_info.sid.sat == sid.sat) {
      pub_data->misc_info.cp_sync.drop = true;
    }
    chMtxUnlock(&pub_data->info_mutex);
  }
}

/** Read the half-cycle ambiguity status.
 *  This function is called from the L2CM tracker.
 *
 * \param[in] sid GNSS signal identifier.
 *
 * \return Polarity of the half-cycle ambiguity.
 */
s8 tracking_channel_read_ambiguity_status(gnss_signal_t sid)
{
  s8 retval = BIT_POLARITY_UNKNOWN;

  for (u8 i = 0; i < nap_track_n_channels; i++) {

    tracker_channel_t *tracker_channel = tracker_channel_get(i);
    tracker_channel_pub_data_t *pub_data = &tracker_channel->pub_data;

    chMtxLock(&pub_data->info_mutex);
    if (sid_is_equal(pub_data->gen_info.sid, sid)) {
      /* If the half-cycle ambiguity has been resolved,
       * return polarity, and reset polarity and sync status. */
      if (pub_data->misc_info.cp_sync.synced) {
        retval = pub_data->misc_info.cp_sync.polarity;
        pub_data->misc_info.cp_sync.polarity = BIT_POLARITY_UNKNOWN;
        pub_data->misc_info.cp_sync.synced = false;
      }
    }
    chMtxUnlock(&pub_data->info_mutex);
  }
  return retval;
}

/** Main function for comparing phase information between L2CM and L2CL trackers.
 *  This function is called from the L2CL tracker.
 *
 * \param[in] sid      GNSS signal identifier.
 * \param[in] fll_mode Flag indicating if the tracker is in FLL mode.
 *
 * \return None
 */
void tracking_channel_cp_sync_match(gnss_signal_t sid, bool fll_mode)
{
  float own_cp = 0.0f;
  float parent_cp = 0.0f;
  float own_cp_p = 0.0f;
  float parent_cp_p = 0.0f;
  s32 own_TOW = 0;
  s32 parent_TOW = 0;
  s32 own_TOW_p = 0;
  s32 parent_TOW_p = 0;
  u8 count = 0;
  bool data_valid = false;

  /* Drop L2CL tracker if it is FLL mode */
  if (fll_mode) {
    tracking_channel_drop_l2cl(sid);
    return;
  }

  /* Check availability of valid L2CM and L2CL data */
  data_valid = tracking_channel_load_data(sid,
                                          &own_cp, &parent_cp,
                                          &own_cp_p, &parent_cp_p,
                                          &own_TOW, &parent_TOW,
                                          &own_TOW_p, &parent_TOW_p,
                                          &count);

  /* Drop L2CL tracker if no valid data is available */
  if (!data_valid) {
    tracking_channel_drop_l2cl(sid);
    return;
  }

  bool TOW_match = true;
  float own_cp_test = 0.0f;
  float parent_cp_test = 0.0f;

  /* Pick phase measurements with matching TOW tag */
  TOW_match =  tracking_channel_find_matching_tow(sid,
                                                  own_cp, parent_cp,
                                                  own_cp_p, parent_cp_p,
                                                  own_TOW, parent_TOW,
                                                  own_TOW_p,parent_TOW_p,
                                                  &own_cp_test,
                                                  &parent_cp_test);

  /* If no TOW tag matched, skip the round */
  if (!TOW_match) {
    return;
  }

  s8 polarity = BIT_POLARITY_UNKNOWN;
  /* Compare the TOW matched carrier phases */
  bool match = tracking_channel_compare_cp(sid,
                                           own_cp_test,
                                           parent_cp_test,
                                           &polarity);

  /* If carrier phases do not match skip the round */
  if (!match) {
    return;
  }

  /* If carrier phases match, increment counter and
   * declare half-cycle ambiguity resolved when counter
   * reaches maximum value. */
  tracking_channel_increment_cp_counter(sid, count, polarity);
}

/** Set the elevation angle for SV by sid.
 *
 * \param[in] sid       Signal identifier for which the elevation should be set.
 * \param[in] elevation Elevation angle [degrees].
 *
 * \retval true  Elevation has been successfully updated.
 * \retval false Elevation has not been updated because GNSS constellation is
 *               not supported.
 *
 * \sa sv_elevation_degrees_get
 */
bool sv_elevation_degrees_set(gnss_signal_t sid, s8 elevation, u64 timestamp)
{
  tp_elevation_entry_t entry = {.elevation_d = elevation,
                                .timestamp_tk = timestamp};
  return track_sid_db_update_elevation(sid, &entry);
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
s8 sv_elevation_degrees_get(gnss_signal_t sid)
{
  s8 result = TRACKING_ELEVATION_UNKNOWN;
  tp_elevation_entry_t entry = {0};
  if (track_sid_db_load_elevation(sid, &entry)) {
    /* If elevation cache entry is loaded, do the entry age check */
    if (TRACKING_ELEVATION_UNKNOWN != entry.elevation_d &&
        nap_timing_count() - entry.timestamp_tk < MAX_ELEVATION_AGE_TK) {
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
 * \param id               ID of the tracker channel to read from.
 * \param soft_bit         Output soft nav bit value.
 * \param sensitivity_mode Flag indicating tracking channel sensitivity mode.
 *
 * \return true if *soft_bit is valid, false otherwise.
 */
bool tracking_channel_nav_bit_get(tracker_channel_id_t id, s8 *soft_bit,
                                  bool *sensitivity_mode)
{
  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_internal_data_t *internal_data = &tracker_channel->internal_data;

  nav_bit_fifo_element_t element;
  if (nav_bit_fifo_read(&internal_data->nav_bit_fifo, &element)) {
    *soft_bit = element.soft_bit;
    *sensitivity_mode = element.sensitivity_mode;
    return true;
  }
  return false;
}

/** Propagate decoded time of week and bit polarity back to a tracker channel.
 *
 * \note This function should be called from the same thread as
 * tracking_channel_nav_bit_get().
 * \note It is assumed that the specified data is synchronized with the most
 * recent nav bit read from the FIFO using tracking_channel_nav_bit_get().
 *
 * \param id            ID of the tracker channel to synchronize.
 * \param TOW_ms        Time of week in milliseconds.
 * \param bit_polarity  Bit polarity.
 *
 * \return true if data was enqueued successfully, false otherwise.
 */
bool tracking_channel_time_sync(tracker_channel_id_t id, s32 TOW_ms,
                                s8 bit_polarity)
{
  assert(TOW_ms >= 0);
  assert(TOW_ms < WEEK_MS);
  assert((bit_polarity == BIT_POLARITY_NORMAL) ||
         (bit_polarity == BIT_POLARITY_INVERTED));

  tracker_channel_t *tracker_channel = tracker_channel_get(id);
  tracker_internal_data_t *internal_data = &tracker_channel->internal_data;
  nav_bit_fifo_index_t read_index = internal_data->nav_bit_fifo.read_index;
  return nav_time_sync_set(&internal_data->nav_time_sync,
                           TOW_ms, bit_polarity, read_index);
}

/** Retrieve the channel info and internal data associated with a
 * tracker context.
 *
 * \note This function is declared in track_internal.h to avoid polluting
 * the public API in track.h
 *
 * \param tracker_context     Tracker context to be resolved.
 * \param channel_info        Output tracker channel info.
 * \param internal_data       Output tracker internal data.
 */
void tracker_internal_context_resolve(tracker_context_t *tracker_context,
                                      const tracker_channel_info_t **channel_info,
                                      tracker_internal_data_t **internal_data)
{
  tracker_channel_t *tracker_channel = (tracker_channel_t *)tracker_context;
  *channel_info = &tracker_channel->info;
  *internal_data = &tracker_channel->internal_data;
}

/** Check the state of a tracker channel and generate events as required.
 * \param tracker_channel   Tracker channel to use.
 * \param update_required   True when correlations are pending for the
 *                          tracking channel.
 */
static void tracker_channel_process(tracker_channel_t *tracker_channel,
                                    bool update_required)
{
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
      tracking_channel_update_values(&tracker_channel->pub_data,
                                     &info,
                                     &time_info,
                                     &freq_info,
                                     &ctrl_params,
                                     reset_cpo);
    }
  }
  break;

  case STATE_DISABLE_REQUESTED: {
    nap_channel_disable(tracker_channel);
    tracker_channel_lock(tracker_channel);
    {
      interface_function(tracker_channel,
                         tracker_channel->interface->disable);
      tracker_channel->disable_time = chVTGetSystemTimeX();
      event(tracker_channel, EVENT_DISABLE);
    }
    tracker_channel_unlock(tracker_channel);
    /* Clear channel public data to stop usage */
    tracking_channel_cleanup_values(&tracker_channel->pub_data);
  }
  break;

  case STATE_DISABLE_WAIT: {
    nap_channel_disable(tracker_channel);
    if (chVTTimeElapsedSinceX(tracker_channel->disable_time) >=
          MS2ST(CHANNEL_DISABLE_WAIT_TIME_ms)) {
      event(tracker_channel, EVENT_DISABLE_WAIT_COMPLETE);
    }
  }
  break;

  case STATE_DISABLED: {
    if (update_required) {
      /* Tracking channel is not owned by the update thread, but the update
       * register must be written to clear the interrupt flag. Set error
       * flag to indicate that NAP is in an unknown state. */
      nap_channel_disable(tracker_channel);
      error_flags_add(tracker_channel, ERROR_FLAG_INTERRUPT_WHILE_DISABLED);
    }
  }
  break;

  default: {
    assert(!"Invalid tracking channel state");
  }
  break;
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
static update_count_t update_count_diff(const tracker_channel_t *
                                        tracker_channel,
                                        const update_count_t *val)
{
  const tracker_common_data_t *common_data = &tracker_channel->common_data;
  update_count_t result = (update_count_t)(common_data->update_count - *val);
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  /* Allow some margin in case values were not read atomically.
   * Treat a difference of [-10000, 0) as zero. */
  if (result > (update_count_t)(UINT32_MAX - 10000))
    return 0;
  else
    return result;
}

/** Parse the IQ output enable bitfield. */
static bool track_iq_output_notify(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    for (int i = 0; i < NUM_TRACKER_CHANNELS; i++) {
      tracker_channel_t *tracker_channel = tracker_channel_get(i);
      tracker_internal_data_t *internal_data = &tracker_channel->internal_data;
      internal_data->output_iq = (iq_output_mask & (1 << i)) != 0;
    }
    return true;
  }
  return false;
}

/** Disable the NAP tracking channel associated with a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void nap_channel_disable(const tracker_channel_t *tracker_channel)
{
  nap_track_disable(tracker_channel->info.nap_channel);
}

/** Retrieve the tracker channel associated with a tracker channel ID.
 *
 * \param tracker_channel_id    ID of the tracker channel to be retrieved.
 *
 * \return Associated tracker channel.
 */
static tracker_channel_t * tracker_channel_get(tracker_channel_id_t id)
{
  assert(id < NUM_TRACKER_CHANNELS);
  return &tracker_channels[id];
}

/** Look up the tracker interface for the specified sid.
 *
 * \param sid       Signal to be tracked.
 *
 * \return Associated tracker interface. May be the default interface.
 */
static const tracker_interface_t * tracker_interface_lookup(gnss_signal_t sid)
{
  const tracker_interface_list_element_t *e = *tracker_interface_list_ptr_get();
  while (e != 0) {
    const tracker_interface_t *interface = e->interface;
    if (interface->code == sid.code) {
      return interface;
    }
    e = e->next;
  }

  return &tracker_interface_default;
}

/** Determine if a tracker channel can be started to track the specified sid.
 *
 * \param tracker_channel_id    ID of the tracker channel to be checked.
 * \param sid                   Signal to be tracked.
 * \param tracker_interface     Output tracker interface to use.
 * \param tracker               Output tracker instance to use.
 *
 * \return true if the tracker channel is available, false otherwise.
 */
static bool tracker_channel_runnable(const tracker_channel_t *tracker_channel,
                                     gnss_signal_t sid, tracker_t **tracker,
                                     const tracker_interface_t **
                                     tracker_interface)
{
  if (tracker_channel_state_get(tracker_channel) != STATE_DISABLED)
      return false;

  *tracker_interface = tracker_interface_lookup(sid);
  if (!available_tracker_get(*tracker_interface, tracker))
    return false;

  return true;
}

/** Find an inactive tracker instance for the specified tracker interface.
 *
 * \param tracker_interface   Tracker interface to use.
 * \param tracker             Output inactive tracker instance.
 *
 * \return true if *tracker points to an inactive tracker instance,
 * false otherwise.
 */
static bool available_tracker_get(const tracker_interface_t *tracker_interface,
                                  tracker_t **tracker)
{
  /* Search for a free tracker */
  for (u32 i=0; i<tracker_interface->num_trackers; i++) {
    tracker_t *t = &tracker_interface->trackers[i];
    if (!tracker_active(t)) {
      *tracker = t;
      return true;
    }
  }

  return false;
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
static state_t tracker_channel_state_get(const tracker_channel_t *
                                         tracker_channel)
{
  state_t state = tracker_channel->state;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return state;
}

/** Return the state of a tracker instance.
 *
 * \note This function performs an acquire operation, meaning that it ensures
 * the returned state was read before any subsequent memory accesses.
 *
 * \param tracker   Tracker to use.
 *
 * \return true if the tracker is active, false if inactive.
 */
static bool tracker_active(const tracker_t *tracker)
{
  bool active = tracker->active;
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return active;
}

/** Execute an interface function on a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param func              Interface function to execute.
 */
static void interface_function(tracker_channel_t *tracker_channel,
                               tracker_interface_function_t *func)
{
  func(&tracker_channel->info, &tracker_channel->common_data,
       tracker_channel->tracker->data);
}

/** Update the state of a tracker channel and its associated tracker instance.
 *
 * \note This function performs a release operation, meaning that it ensures
 * all prior memory accesses have completed before updating state information.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param event             Event to process.
 */
static void event(tracker_channel_t *tracker_channel, event_t event)
{
  switch (event) {
  case EVENT_ENABLE: {
    assert(tracker_channel->state == STATE_DISABLED);
    assert(tracker_channel->tracker->active == false);
    tracker_channel->tracker->active = true;
    /* Sequence point for enable is setting channel state = STATE_ENABLED */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    tracker_channel->state = STATE_ENABLED;
  }
  break;

  case EVENT_DISABLE_REQUEST: {
    assert(tracker_channel->state == STATE_ENABLED);
    tracker_channel->state = STATE_DISABLE_REQUESTED;
  }
  break;

  case EVENT_DISABLE: {
    assert(tracker_channel->state == STATE_DISABLE_REQUESTED);
    tracker_channel->state = STATE_DISABLE_WAIT;
  }
  break;

  case EVENT_DISABLE_WAIT_COMPLETE: {
    assert(tracker_channel->state == STATE_DISABLE_WAIT);
    assert(tracker_channel->tracker->active == true);
    /* Sequence point for disable is setting channel state = STATE_DISABLED
     * and/or tracker active = false (order of these two is irrelevant here) */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    tracker_channel->tracker->active = false;
    tracker_channel->state = STATE_DISABLED;
  }
  break;

  default: {
    assert(!"Invalid event");
  }
  break;
  }
}

/** Initialize a tracker common data structure.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param sample_count      Sample count.
 * \param carrier_freq      Carrier frequency.
 * \param cn0               C/N0 estimate.
 * \param code              Code identifier.
 */
static void common_data_init(tracker_common_data_t *common_data,
                             u32 sample_count, float carrier_freq,
                             float cn0, code_t code)
{
  /* Initialize all fields to 0 */
  memset(common_data, 0, sizeof(tracker_common_data_t));

  common_data->TOW_ms = TOW_INVALID;

  /* Calculate code phase rate with carrier aiding. */
  common_data->code_phase_rate = (1.0 + carrier_freq / code_to_carr_freq(code)) *
                                 code_to_chip_rate(code);
  common_data->carrier_freq = carrier_freq;

  common_data->sample_count = sample_count;
  common_data->cn0 = cn0;
  u32 now = timing_getms();
  common_data->init_timestamp_ms = now;
  common_data->update_timestamp_ms = now;
  common_data->updated_once = false;
}

/** Initialize a tracker public data structure.
 *
 * \param pub_data Tracker public data structure.
 *
 * \return None
 */
static void public_data_init(tracker_channel_pub_data_t *pub_data)
{
  pub_data->misc_info.cp_sync.counter = 0;
  pub_data->misc_info.cp_sync.drop = false;
  pub_data->misc_info.cp_sync.polarity = BIT_POLARITY_UNKNOWN;
  pub_data->misc_info.cp_sync.synced = false;
}

/** Lock a tracker channel for exclusive access.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void tracker_channel_lock(tracker_channel_t *tracker_channel)
{
  chMtxLock(&tracker_channel->mutex);
}

/** Unlock a locked tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void tracker_channel_unlock(tracker_channel_t *tracker_channel)
{
  chMtxUnlock(&tracker_channel->mutex);
}

/** Clear the error flags for a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 */
static void error_flags_clear(tracker_channel_t *tracker_channel)
{
  tracker_channel->error_flags = ERROR_FLAG_NONE;
}

/** Add an error flag to a tracker channel.
 *
 * \param tracker_channel   Tracker channel to use.
 * \param error_flag        Error flag to add.
 */
static void error_flags_add(tracker_channel_t *tracker_channel,
                            error_flag_t error_flag)
{
  tracker_channel->error_flags |= error_flag;
}

static tracking_channel_flags_t tracking_channel_get_flags(
    const tracker_channel_t *tracker_channel)
{
  tracking_channel_flags_t result = 0;

  const tracker_common_data_t *const common_data = &tracker_channel->common_data;
  const tracker_internal_data_t *const internal_data = &tracker_channel->internal_data;

  if (STATE_ENABLED == tracker_channel_state_get(tracker_channel)) {

    result |= TRACKING_CHANNEL_FLAG_ACTIVE;

    if (ERROR_FLAG_NONE == tracker_channel->error_flags) {
      /* Make sure no errors have occurred. */
      result |= TRACKING_CHANNEL_FLAG_NO_ERROR;
    }
    /* Check if the tracking is in confirmed state. */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_CONFIRMED)) {
      result |= TRACKING_CHANNEL_FLAG_CONFIRMED;
    }
    /* Check C/N0 has been above threshold for a long time (RTK). */
    u32 cn0_threshold_count_ms = (common_data->update_count -
                                  common_data->cn0_below_use_thres_count);
    if (cn0_threshold_count_ms > TRACK_CN0_THRES_COUNT_LONG) {
      result |= TRACKING_CHANNEL_FLAG_CN0_LONG;
    }
    /* Check C/N0 has been above threshold for the minimum time (SPP). */
    if (cn0_threshold_count_ms > TRACK_CN0_THRES_COUNT_SHORT) {
      result |= TRACKING_CHANNEL_FLAG_CN0_SHORT;
    }
    /* Pessimistic phase lock detector = "locked". */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_PLOCK) &&
        (common_data->update_count -
         common_data->ld_pess_change_count) > TRACK_USE_LOCKED_T) {
      result |= TRACKING_CHANNEL_FLAG_CONFIRMED_LOCK;
    }
    /* Some time has elapsed since the last tracking channel mode
     * change, to allow any transients to stabilize.
     * TODO: is this still necessary? */
    if ((common_data->update_count -
         common_data->mode_change_count) > TRACK_STABILIZATION_T) {
      result |= TRACKING_CHANNEL_FLAG_STABLE;
    }

    /* Channel time of week has been decoded. */
    if (TOW_INVALID != common_data->TOW_ms) {
      result |= TRACKING_CHANNEL_FLAG_TOW;
    }
    /* Bit sync has been reached. */
    if (BITSYNC_UNSYNCED != internal_data->bit_sync.bit_phase_ref) {
      result |= TRACKING_CHANNEL_FLAG_BIT_SYNC;
    }
    /* Nav bit polarity is known, i.e. half-cycles have been resolved.
     * bit polarity known flag is set only when phase lock to prevent the
     * situation when channel loses an SV, but decoder just finished TOW decoding
     * which cause bit polarity know flag set */
    if (BIT_POLARITY_UNKNOWN != internal_data->bit_polarity
        && (common_data->flags & TRACK_CMN_FLAG_HAS_PLOCK)) {
      result |= TRACKING_CHANNEL_FLAG_BIT_POLARITY;
    }
    if (BIT_POLARITY_INVERTED == internal_data->bit_polarity) {
      result |= TRACKING_CHANNEL_FLAG_BIT_INVERTED;
    }
    /* Tracking mode */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_PLL_USE)) {
      result |= TRACKING_CHANNEL_FLAG_PLL_USE;
    }
    if (0 != (common_data->flags & TRACK_CMN_FLAG_FLL_USE)) {
      result |= TRACKING_CHANNEL_FLAG_FLL_USE;
    }
    /* Tracking status: pessimistic PLL lock */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_PLOCK)) {
      result |= TRACKING_CHANNEL_FLAG_PLL_PLOCK;
    }
    /* Tracking status: optimistic PLL lock */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_OLOCK)) {
      result |= TRACKING_CHANNEL_FLAG_PLL_OLOCK;
    }
    /* Tracking status: FLL lock */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAS_FLOCK)) {
      result |= TRACKING_CHANNEL_FLAG_FLL_LOCK;
    }
    /* Tracking status: tracking channel has ever been in PLL/FLL pessimistic
     * lock state. */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_HAD_PLOCK) ||
        0 != (common_data->flags & TRACK_CMN_FLAG_HAD_FLOCK)) {
      result |= TRACKING_CHANNEL_FLAG_HAD_LOCKS;
    }
    /* Tracking status: TOW propagation status */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_TOW_PROPAGATED)) {
      result |= TRACKING_CHANNEL_FLAG_TOW_PROPAGATED;
    }
    /* Tracking status: TOW decoding status */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_TOW_DECODED)) {
      result |= TRACKING_CHANNEL_FLAG_TOW_DECODED;
    }
    /* Tracking status: cross-correlation status */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_XCORR_CONFIRMED)) {
      result |= TRACKING_CHANNEL_FLAG_XCORR_CONFIRMED;
    }
    /* Tracking status: cross-correlation suspect */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_XCORR_SUSPECT)) {
      result |= TRACKING_CHANNEL_FLAG_XCORR_SUSPECT;
    }
    /* Tracking status: cross-correlation doppler filter active */
    if (0 != (common_data->flags & TRACK_CMN_FLAG_XCORR_FILTER_ACTIVE)) {
      result |= TRACKING_CHANNEL_FLAG_XCORR_FILTER_ACTIVE;
    }
  }

  return result;
}

/** \} */
