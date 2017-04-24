/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Local headers */
#include "track_glo_l1ca.h"
#include "track_cn0.h"
#include "track_profile_utils.h"
#include "track_profiles.h"
#include "track_sid_db.h"
#include "track_internal.h"

/* Non-local headers */
#include <platform_track.h>
#include <signal.h>
#include <track_api.h>
#include <manage.h>
#include <track.h>
#include <ndb.h>

/* Libraries */
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

/* STD headers */
#include <string.h>
#include <assert.h>

/** GLO L1CA configuration section name */
#define GLO_L1CA_TRACK_SETTING_SECTION "glo_l1ca_track"

static tp_tracker_config_t glo_l1ca_config = TP_TRACKER_DEFAULT_CONFIG;

static tracker_t glo_l1ca_trackers[NUM_GLO_L1CA_TRACKERS];
static gps_l2cm_tracker_data_t glo_l1ca_tracker_data[ARRAY_SIZE(glo_l1ca_trackers)];

/* Forward declarations of interface methods for GLO L1CA */
static tracker_interface_function_t tracker_glo_l1ca_init;
static tracker_interface_function_t tracker_glo_l1ca_disable;
static tracker_interface_function_t tracker_glo_l1ca_update;

/** GLO L1CA tracker interface */
static const tracker_interface_t tracker_interface_glo_l1ca = {
  .code =         CODE_GLO_L1CA,
  .init =         tracker_glo_l1ca_init,
  .disable =      tracker_glo_l1ca_disable,
  .update =       tracker_glo_l1ca_update,
  .trackers =     glo_l1ca_trackers,
  .num_trackers = ARRAY_SIZE(glo_l1ca_trackers)
};

static tracker_interface_list_element_t tracker_interface_list_glo_l1ca = {
  .interface = &tracker_interface_glo_l1ca,
  .next = 0
};

/** Register GLO L1CA tracker into the the tracker interface & settings
 *  framework.
 */
void track_glo_l1ca_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(GLO_L1CA_TRACK_SETTING_SECTION,
                             glo_l1ca_config,
                             settings_default_notify);

  for (u32 i = 0; i < ARRAY_SIZE(glo_l1ca_trackers); i++) {
    glo_l1ca_trackers[i].active = false;
    glo_l1ca_trackers[i].data = &glo_l1ca_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_glo_l1ca);
}

/** Do GLO L1CA to L2CA handover.
 *
 * The condition for the handover is the availability of meander sync on L1CA
 *
 * \param sample_count NAP sample count
 * \param sat GLO L1CA frequency slot FCN
 * \param code_phase_chips L1CA code phase [chips]
 * \param carrier_freq_hz The current Doppler frequency for the L1CA channel
 * \param init_cn0_dbhz CN0 estimate for the L1CA channel [dB-Hz]
 */
void do_glo_l1ca_to_l2ca_handover(u32 sample_count,
                                  u16 sat,
                                  float code_phase_chips,
                                  double carrier_freq_hz,
                                  float init_cn0_dbhz)
{
  (void)sample_count;
  (void)sat;
  (void)code_phase_chips;
  (void)carrier_freq_hz;
  (void)init_cn0_dbhz;
}

static void tracker_glo_l1ca_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  glo_l1ca_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(*data));

  tp_tracker_init(channel_info, common_data, &data->data, &glo_l1ca_config);
}

static void tracker_glo_l1ca_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  glo_l1ca_tracker_data_t *data = tracker_data;

  tp_tracker_disable(channel_info, common_data, &data->data);
}

static void tracker_glo_l1ca_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{

  glo_l1ca_tracker_data_t *glo_l1ca_data = tracker_data;
  tp_tracker_data_t *data = &glo_l1ca_data->data;
  u32 tracker_flags = tp_tracker_update(channel_info, common_data, data,
                                        &glo_l1ca_config);

  if (data->lock_detect.outp &&
      data->confirmed &&
      0 != (tracker_flags & TP_CFLAG_BSYNC_UPDATE) &&
      tracker_bit_aligned(channel_info->context)) {

    /* Start GLO L2CA tracker if not running */
    do_glo_l1ca_to_l2ca_handover(common_data->sample_count,
                                 channel_info->mesid.sat,
                                 common_data->code_phase_prompt,
                                 common_data->carrier_freq,
                                 common_data->cn0);
  }
}
