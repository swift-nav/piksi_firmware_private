/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* skip weak attributes for L2C API implementation */
#define TRACK_GPS_L2CM_INTERNAL
#include "track_gps_l2cm.h"
#include "track_api.h"
#include "track_cn0.h"
#include "track_profile_utils.h"
#include "track_profiles.h"
#include "manage.h"
#include "track.h"
#include "ndb.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>
#include "math.h"

#include "signal.h"
#include "track_profile_utils.h"

#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/**
 * GPS L2C tracker data container type.
 */
typedef struct {
  tp_tracker_data_t data; /**< Common tracker parameters */
} gps_l2cm_tracker_data_t;

/** GPS L2C configuration container */
static tp_tracker_config_t gps_l2cm_config = TP_TRACKER_DEFAULT_CONFIG;
/** GPS L2C tracker table */
static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
/** GPS L2C tracker data */
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

/* Forward declarations of interface methods for GPS L2C */
static tracker_interface_function_t tracker_gps_l2cm_init;
static tracker_interface_function_t tracker_gps_l2cm_disable;
static tracker_interface_function_t tracker_gps_l2cm_update;

/** GPS L2C tracker interface */
static const tracker_interface_t tracker_interface_gps_l2cm = {
  .code =         CODE_GPS_L2CM,
  .init =         tracker_gps_l2cm_init,
  .disable =      tracker_gps_l2cm_disable,
  .update =       tracker_gps_l2cm_update,
  .trackers =     gps_l2cm_trackers,
  .num_trackers = NUM_GPS_L2CM_TRACKERS
};

/** GPS L2C tracker interface list element */
static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cm = {
    .interface = &tracker_interface_gps_l2cm,
    .next = 0
  };

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  TP_TRACKER_REGISTER_CONFIG(L2CM_TRACK_SETTING_SECTION, gps_l2cm_config);

  for (u32 i = 0; i < NUM_GPS_L2CM_TRACKERS; i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sample_count NAP sample count
 * \param sat L1C/A Satellite ID
 * \param code_phase L1CA code phase [chips]
 * \param carrier_freq The current Doppler frequency for the L1 C/A channel
 * \param cn0 CN0 estimate for the L1 C/A channel
 */
void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              float code_phase,
                              double carrier_freq,
                              float cn0_init)
{
  /* compose SID: same SV, but code is L2 CM */
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(sid)) {
    return; /* L2C signal from the SV is already in track */
  }

  u32 capb;
  ndb_gps_l2cm_l2c_cap_read(&capb);
  if (0 == (capb & ((u32)1 << (sat - 1)))) {
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn_sid(sid, "Unexpected L1C/A to L2C handover code phase: %f",
                 code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = GPS_L2C_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .sid                = sid,
    .sample_count       = sample_count,
    /* recalculate doppler freq for L2 from L1*/
    .carrier_freq       = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
    .code_phase         = code_phase,
    .chips_to_correlate = 1023,
    /* get initial cn0 from parent L1 channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  switch (tracking_startup_request(&startup_params)) {
  case 0:
    log_debug_sid(sid, "L2 CM handover done");
    break;

  case 1:
    /* sat is already in fifo, no need to inform */
    break;

  case 2:
    log_warn_sid(sid, "Failed to start L2C tracking");
    break;

  default:
    assert(!"Unknown code returned");
    break;
  }
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));

  tp_tracker_init(channel_info, common_data, &data->data, &gps_l2cm_config);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(channel_info->context, 0);
}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  /* Unused parameters */
  (void)common_data;
  (void)tracker_data;
  tp_tracker_disable(channel_info);
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *l2c_data = tracker_data;
  tp_tracker_data_t *data = &l2c_data->data;

  tp_tracker_update(channel_info, common_data, data);
}
