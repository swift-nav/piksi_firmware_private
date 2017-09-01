/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_MANAGE_H
#define SWIFTNAV_MANAGE_H

#include <ch.h>
#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>
#include "soft_macq/soft_macq_main.h"
#include "track_flags.h"

/** \addtogroup manage
 * \{ */

/** How many ms to allow tracking channel to converge after
    initialization before we consider dropping it.
    Applied to all signals initialized from ACQ. */
#define TRACK_INIT_FROM_ACQ_MS 2500

/** How many ms to allow tracking channel to converge after
    initialization before we consider dropping it.
    Applied to all signals initialized from a handover. */
#define TRACK_INIT_FROM_HANDOVER_MS 500

/** If a channel is dropped but was running successfully for at least
    this long, mark it for prioritized reacquisition. */
#define TRACK_REACQ_MS 5000

/** If C/N0 is below track_cn0_threshold for >= TRACK_DROP_CN0_T ms,
    drop the channel. */
#define TRACK_DROP_CN0_MS 500

/** If pessimistic lock detector shows "unlocked" for >=
    TRACK_DROP_UNLOCKED_MS, drop the channel. */
#define TRACK_DROP_UNLOCKED_MS 1500

#define ACQ_FULL_CF_STEP soft_multi_acq_bin_width()

#define MANAGE_NO_CHANNELS_FREE 255

#define MANAGE_ACQ_THREAD_PRIORITY (LOWPRIO)
#define MANAGE_ACQ_THREAD_STACK 16384

typedef struct {
  me_gnss_signal_t mesid; /**< ME signal identifier. */
  u16 glo_slot_id;        /**< GLO orbital slot. */
  u64 sample_count;       /**< Reference NAP sample count. */
  float carrier_freq;     /**< Carrier frequency Doppler (Hz). */
  double code_phase;      /**< Code phase (chips). */
  u32 chips_to_correlate; /**< Chips to integrate over. */
  float cn0_init;         /**< C/N0 estimate (dBHz). */
  s8 elevation;           /**< Elevation (deg). */
} tracking_startup_params_t;

/**
 * Tracking controller parameters.
 */
typedef struct {
  float fll_bw; /**< FLL controller NBW [Hz].
                     Single sided noise bandwidth in case of
                     FLL and FLL-assisted PLL tracking */
  float pll_bw; /**< PLL controller noise bandwidth [Hz].
                     Single sided noise bandwidth in case of
                     PLL and FLL-assisted PLL tracking */
  float dll_bw; /**< DLL controller noise bandwidth [Hz]. */
  u8 int_ms;    /**< PLL/FLL controller integration time [ms] */
} tracking_ctrl_params_t;

/** \} */

void manage_acq_setup(void);

void manage_set_obs_hint(gnss_signal_t sid);

void me_settings_setup(void);

float get_solution_elevation_mask(void);
void acq_result_send(const me_gnss_signal_t mesid,
                     float cn0,
                     float cp,
                     float cf);

u32 get_tracking_channel_meas(u8 i,
                              u64 ref_tc,
                              channel_measurement_t *meas,
                              ephemeris_t *ephe);
void get_tracking_channel_ctrl_params(u8 i, tracking_ctrl_params_t *pparams);
u32 get_tracking_channel_sid_flags(const gnss_signal_t sid,
                                   s32 tow_ms,
                                   const ephemeris_t *pephe);
u8 tracking_channels_ready(u32 required_flags);

bool tracking_startup_ready(const me_gnss_signal_t mesid);
bool tracking_is_running(const me_gnss_signal_t mesid);
u8 tracking_startup_request(const tracking_startup_params_t *startup_params);

bool l1ca_l2cm_handover_reserve(u8 sat);
void l1ca_l2cm_handover_release(u8 sat);
bool mesid_is_tracked(const me_gnss_signal_t mesid);
bool is_glo_enabled(void);
void sanitize_trackers(void);
void check_clear_glo_unhealthy(void);
void check_clear_unhealthy(void);

#endif
