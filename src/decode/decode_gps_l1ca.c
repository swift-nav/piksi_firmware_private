/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/logging.h>

#include "decode.h"
#include "decode_common.h"
#include "decode_gps_l1ca.h"
#include "ephemeris/ephemeris.h"
#include "me_constants.h"
#include "nav_msg/nav_msg.h"
#include "ndb/ndb.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "shm/shm.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track/track_decode.h"
#include "track/track_flags.h"
#include "track/track_sid_db.h"
#include "track/track_state.h"

#include <assert.h>
#include <string.h>

#define IONO_HYSTERESIS_MS 1000

/** GPS L1 C/A decoder data */
typedef struct { nav_msg_t nav_msg; } gps_l1ca_decoder_data_t;

static decoder_t gps_l1ca_decoders[NUM_GPS_L1CA_DECODERS];
static gps_l1ca_decoder_data_t
    gps_l1ca_decoder_data[ARRAY_SIZE(gps_l1ca_decoders)];

static void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_gps_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l1ca = {
    .code = CODE_GPS_L1CA,
    .init = decoder_gps_l1ca_init,
    .disable = decoder_gps_l1ca_disable,
    .process = decoder_gps_l1ca_process,
    .decoders = gps_l1ca_decoders,
    .num_decoders = ARRAY_SIZE(gps_l1ca_decoders)};

static decoder_interface_list_element_t list_element_gps_l1ca = {
    .interface = &decoder_interface_gps_l1ca, .next = NULL};

/**
 * Check that an almanac matches with the ephemeris of that satellite but not
 * with any other.
 *
 * An almanac and ephemeris are considered to match if the satellite positions
 * computed at three points (TOE and both end points of ephemeris fit interval)
 * match within 40 kilometers.
 *
 * If the new almanac does not match with the corresponding ephemeris, the
 * ephemeris is deleted. If the new almanac matches the ephemeris of some other
 * satellite, the ephemeris is deleted and that channel flagged for
 * cross-correlation.
 *
 * \param[in] sid GNSS signal identifier for which almanac has been updated
 *
 * \return None
 */
static void check_almanac_xcorr(gnss_signal_t sid) {
  xcorr_positions_t alm_pos; /* Almanac's positions for sid */
  alm_pos.time_s = 0;

  for (u8 sv_idx = 0; sv_idx < NUM_SATS_GPS; ++sv_idx) {
    gnss_signal_t sid1 = construct_sid(CODE_GPS_L1CA, sv_idx + GPS_FIRST_PRN);
    ephemeris_t e;
    if (NDB_ERR_NONE != ndb_ephemeris_read(sid1, &e) || e.toe.wn <= 0) {
      continue;
    }

    u32 time_s = (u32)e.toe.wn * WEEK_SECS + (s32)e.toe.tow;
    u32 interval_s = e.fit_interval / 2;
    if (alm_pos.time_s != time_s || alm_pos.interval_s != interval_s) {
      /* If ephemeris time differs from last computed almanac time, or the
       * first one, compute the almanac positions for the new time */
      if (!xcorr_calc_alm_positions(sid, time_s, interval_s, &alm_pos)) {
        /* this happens when the almanac does not have WN yet */
        log_debug_sid(sid1,
                      "Failed to compute almanac positions (wn:%u, toe:%f)",
                      e.toe.wn,
                      e.toe.tow);
        continue;
      }
    }

    xcorr_positions_t eph_pos;
    if (!xcorr_calc_eph_positions(&e, time_s, &eph_pos)) {
      /* this should not happen with a valid ephemeris */
      log_warn_sid(sid1,
                   "Failed to compute ephemeris positions (wn:%u, toe:%f)",
                   e.toe.wn,
                   e.toe.tow);
      continue;
    }

    bool match = xcorr_match_positions(sid, sid1, &eph_pos, &alm_pos);

    if (sid1.sat == sid.sat) {
      if (match) {
        /* OK */
      } else {
        log_warn_sid(sid1, "Ephemeris does not match with almanac, dropping");
        ndb_ephemeris_erase(sid1);
      }
    } else if (match) {
      /* Cross-correlation */
      char sid_str_[SID_STR_LEN_MAX];
      sid_to_string(sid_str_, sizeof(sid_str_), sid);

      log_warn_sid(
          sid1, "Almanac-ephemeris cross-correlation with %s", sid_str_);
      ndb_ephemeris_erase(sid1);
      tracker_set_xcorr_flag(construct_mesid(sid1.code, sid1.sat));
    }
  }
}

/**
 * Checks cross-correlations for all almanacs that have given WN/TOA
 *
 * The method is called after new WN/TOA pair has been decoded and accepted for
 * NDB storage. As this implies update of NDB almanacs with matching TOA, the
 * system executes a cross-correlation checks for all of those.
 *
 * \param[in] wn  Almanac's week number
 * \param[in] toa Almanac's TOA
 *
 * \return None
 *
 * \sa check_almanac_xcorr
 */
static void check_almanac_wn_xcorr(s16 wn, s32 toa) {
  for (u8 sv_idx = 0; sv_idx < NUM_SATS_GPS; ++sv_idx) {
    gnss_signal_t sid = construct_sid(CODE_GPS_L1CA, sv_idx + GPS_FIRST_PRN);
    almanac_t a;
    ndb_op_code_t oc = ndb_almanac_read(sid, &a);
    /* Here we do not care if GPS time is unknown
     * since almanac toa is compared against ephemeris toe. */
    bool alma_valid = (NDB_ERR_NONE == oc || NDB_ERR_GPS_TIME_MISSING == oc);
    if (alma_valid && (a.toa.wn == wn) && ((s32)a.toa.tow == toa)) {
      check_almanac_xcorr(sid);
    }
  }
}

/**
 * For decoded ionosphere data, check if the time of almanac (toa) is newer
 * than the one currently stored in NDB.
 *
 * \param sid   GNSS signal identifier for which to check almanac toa
 * \param iono  Decoded ionosphere data
 *
 * return True  if toa is valid and not older than the one in NDB.
 *        False otherwise
 */
static bool check_iono_timestamp(gnss_signal_t sid, ionosphere_t *iono) {
  bool alma_valid = false; /* Valid toa is available for given sid */
  bool iono_valid = false; /* Valid iono is available in NDB */
  almanac_t existing_a;    /* Existing almanac data */
  ionosphere_t existing_i; /* Existing ionosphere data */

  /* Check if valid almanac toa is present for given sid. */
  ndb_op_code_t oc = ndb_almanac_read(sid, &existing_a);
  /* Here we do not care if GPS time is unknown
   * since alma toa is compared against iono toa. */
  alma_valid = (NDB_ERR_NONE == oc || NDB_ERR_GPS_TIME_MISSING == oc) &&
               gps_time_valid(&existing_a.toa);

  if (!alma_valid) {
    return false;
  }

  /* If almanac toa was available, copy it for iono parameters. */
  iono->toa = existing_a.toa;

  /* Check if previously stored ionosphere data is available in NDB. */
  oc = ndb_iono_corr_read(&existing_i);
  /* Here we do not care if GPS time is unknown
   * since iono toa is compared against almanac toa. */
  iono_valid = (NDB_ERR_NONE == oc || NDB_ERR_GPS_TIME_MISSING == oc);

  if (iono_valid) {
    /* Check if decoded data is not older that the one stored in NDB. */
    double age = gpsdifftime(&existing_a.toa, &existing_i.toa);
    return age >= 0.0f;
  } else {
    /* If NDB has no previously saved data, or contains aged data */
    return true;
  }
}

/**
 * Stores new almanac data to NDB.
 *
 * \param sid   Almanac source
 * \param alma  Almanac data
 *
 * return None
 */
static void decode_almanac_new(gnss_signal_t sid, const almanac_t *alma) {
  ndb_op_code_t oc =
      ndb_almanac_store(&sid, alma, NDB_DS_RECEIVER, NDB_EVENT_SENDER_ID_VOID);
  char src_sid_str[SID_STR_LEN_MAX];
  sid_to_string(src_sid_str, sizeof(src_sid_str), sid);
  switch (oc) {
    case NDB_ERR_NONE:
      log_debug_sid(alma->sid, "almanac from %s saved", src_sid_str);
      check_almanac_xcorr(alma->sid);
      break;
    case NDB_ERR_NO_CHANGE:
      log_debug_sid(
          alma->sid, "almanac from %s is already present", src_sid_str);
      break;
    case NDB_ERR_UNCONFIRMED_DATA:
      log_debug_sid(
          alma->sid, "almanac from %s is unconfirmed, not saved", src_sid_str);
      break;
    case NDB_ERR_OLDER_DATA:
      log_debug_sid(alma->sid,
                    "almanac from %s is older than one in DB, not saved",
                    src_sid_str);
      break;
    case NDB_ERR_MISSING_IE:
    case NDB_ERR_UNSUPPORTED:
    case NDB_ERR_FILE_IO:
    case NDB_ERR_INIT_DONE:
    case NDB_ERR_BAD_PARAM:
    case NDB_ERR_ALGORITHM_ERROR:
    case NDB_ERR_NO_DATA:
    case NDB_ERR_AGED_DATA:
    case NDB_ERR_GPS_TIME_MISSING:
    default:
      log_warn_sid(
          alma->sid, "error %d storing almanac from %s", (int)oc, src_sid_str);
      break;
  }
}

/**
 * Updates NDB with new TOA/WN pair.
 *
 * \param sid       Time source
 * \param alma_time New almanac time
 *
 * \return None
 */
static void decode_almanac_time_new(gnss_signal_t sid,
                                    const gps_time_t *alma_time) {
  ndb_op_code_t r = ndb_almanac_wn_store(sid,
                                         alma_time->tow,
                                         alma_time->wn,
                                         NDB_DS_RECEIVER,
                                         NDB_EVENT_SENDER_ID_VOID);

  switch (r) {
    case NDB_ERR_NONE:
      log_debug_sid(sid,
                    "almanac time info saved (%" PRId16 ", %" PRId32 ")",
                    alma_time->wn,
                    (s32)alma_time->tow);
      check_almanac_wn_xcorr(alma_time->wn, (s32)alma_time->tow);
      break;
    case NDB_ERR_NO_CHANGE:
      log_debug_sid(sid,
                    "almanac time info is already present (%" PRId16
                    ", %" PRId32 ")",
                    alma_time->wn,
                    (s32)alma_time->tow);
      break;
    case NDB_ERR_UNCONFIRMED_DATA:
      log_debug_sid(sid,
                    "almanac time info is unconfirmed (%" PRId16 ", %" PRId32
                    ")",
                    alma_time->wn,
                    (s32)alma_time->tow);
      break;
    case NDB_ERR_OLDER_DATA:
    case NDB_ERR_MISSING_IE:
    case NDB_ERR_UNSUPPORTED:
    case NDB_ERR_FILE_IO:
    case NDB_ERR_INIT_DONE:
    case NDB_ERR_BAD_PARAM:
    case NDB_ERR_ALGORITHM_ERROR:
    case NDB_ERR_NO_DATA:
    case NDB_ERR_AGED_DATA:
    case NDB_ERR_GPS_TIME_MISSING:
    default:
      log_error_sid(sid,
                    "error %d updating almanac time (%" PRId16 ", %" PRId32 ")",
                    (int)r,
                    alma_time->wn,
                    (s32)alma_time->tow);
      break;
  }
}

/**
 * Deletes almanacs/ephemeris for SVs with error bit set and updates almanacs
 * otherwise. Data source SV is assumed valid and healthy.
 *
 * \param[in] src_sid    Health bits source SV
 * \param[in] hlags_mask Mask where new bits are set for valid entries in \a
 *                       hflags.
 * \param[in] hflags     Mask array for GPS satellites, where index is `PRN - 1`
 *
 * \return None
 */
static void decode_almanac_health_new(gnss_signal_t src_sid,
                                      u32 hlags_mask,
                                      const u8 hflags[32]) {
  /* Copy updated flags into the cache, and update all entries in NDB */
  for (u16 sv_idx = 0; sv_idx < 32; ++sv_idx) {
    if (0 == (hlags_mask & 1u << sv_idx)) {
      /* No flag information for this SV */
      continue;
    }

    gnss_signal_t target_sid = construct_sid(CODE_GPS_L1CA, sv_idx + 1);

    u8 health_bits = hflags[sv_idx];

    ndb_op_code_t r = ndb_almanac_hb_update(target_sid,
                                            health_bits,
                                            NDB_DS_RECEIVER,
                                            &src_sid,
                                            NDB_EVENT_SENDER_ID_VOID);

    switch (r) {
      case NDB_ERR_NONE:
        log_debug_sid(target_sid,
                      "almanac health bits updated (0x%02" PRIX8 ")",
                      health_bits);
        break;
      case NDB_ERR_NO_CHANGE:
        log_debug_sid(target_sid,
                      "almanac health bits up to date (0x%02" PRIX8 ")",
                      health_bits);
        break;
      case NDB_ERR_UNCONFIRMED_DATA:
        log_debug_sid(target_sid,
                      "almanac health bits are unconfirmed (0x%02" PRIX8 ")",
                      health_bits);
        break;
      case NDB_ERR_NO_DATA:
        log_debug_sid(target_sid,
                      "almanac health bits are ignored (0x%02" PRIX8 ")",
                      health_bits);
        break;
      case NDB_ERR_OLDER_DATA:
      case NDB_ERR_MISSING_IE:
      case NDB_ERR_UNSUPPORTED:
      case NDB_ERR_FILE_IO:
      case NDB_ERR_INIT_DONE:
      case NDB_ERR_BAD_PARAM:
      case NDB_ERR_ALGORITHM_ERROR:
      case NDB_ERR_AGED_DATA:
      case NDB_ERR_GPS_TIME_MISSING:
      default:
        log_error_sid(target_sid,
                      "error %d updating almanac health bits (0x%02" PRIX8 ")",
                      (int)r,
                      health_bits);
        break;
    }

    /* Health indicates CODE_NAV_STATE_INVALID */
    if (shm_signal_unhealthy(target_sid)) {
      /* Clear NDB and TOW cache */
      erase_nav_data(target_sid, src_sid);
    }

    gnss_signal_t l2cm =
        (gnss_signal_t){.sat = target_sid.sat, .code = CODE_GPS_L2CM};
    if (shm_signal_unhealthy(l2cm)) {
      /* Clear CNAV data and TOW cache */
      erase_cnav_data(l2cm, src_sid);
    }
  }
}

void decode_gps_l1ca_register(void) {
  for (u32 i = 0; i < ARRAY_SIZE(gps_l1ca_decoders); i++) {
    gps_l1ca_decoders[i].active = false;
    gps_l1ca_decoders[i].data = &gps_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l1ca);
}

static void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data) {
  (void)channel_info;
  gps_l1ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init(&data->nav_msg);
}

static void decoder_gps_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data) {
  gps_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  nav_bit_t nav_bit;
  s8 prev_polarity = BIT_POLARITY_UNKNOWN;
  while (tracker_nav_bit_get(channel_info->tracking_channel, &nav_bit)) {
    /* Don't decode data while in sensitivity mode. */
    if (0 == nav_bit) {
      nav_msg_init(&data->nav_msg);
      continue;
    }
    /* Update TOW */
    bool bit_val = nav_bit > 0;
    nav_data_sync_t from_decoder;
    tracker_data_sync_init(&from_decoder);
    prev_polarity = data->nav_msg.bit_polarity;
    from_decoder.TOW_ms = nav_msg_update(&data->nav_msg, bit_val);
    from_decoder.bit_polarity = data->nav_msg.bit_polarity;
    /* Let's not update TOW together with fast HCA resolution. */
    if (BIT_POLARITY_UNKNOWN == prev_polarity &&
        BIT_POLARITY_UNKNOWN != from_decoder.bit_polarity) {
      /* Only update polarity. */
      from_decoder.sync_flags = SYNC_POL;
    }
    tracker_data_sync(channel_info->tracking_channel, &from_decoder);
  }

  /* Check if there is a new nav msg subframe to process. */
  if (!subframe_ready(&data->nav_msg)) {
    return;
  }

  /* Decode nav data to temporary structure */
  gps_l1ca_decoded_data_t dd;
  s8 ret = process_subframe(&data->nav_msg, channel_info->mesid, &dd);

  if (ret <= 0) {
    return;
  }

  gnss_signal_t l1ca_sid =
      construct_sid(channel_info->mesid.code, channel_info->mesid.sat);

  if (dd.almanac_upd_flag && (0 != dd.almanac.health_bits)) {
    /* If almanac health bits indicate problems,
     * clear NDB and TOW cache of the SV the almanac is for. */
    erase_nav_data(dd.almanac.sid, l1ca_sid);
    return;
  }

  if (dd.invalid_control_or_data) {
    log_info_mesid(channel_info->mesid, "Invalid control or data element");

    ndb_op_code_t c = ndb_ephemeris_erase(l1ca_sid);

    if (NDB_ERR_NONE == c) {
      log_info_mesid(channel_info->mesid, "ephemeris deleted (1/0)");
    } else if (NDB_ERR_NO_CHANGE != c) {
      log_warn_mesid(
          channel_info->mesid, "error %d deleting ephemeris (1/0)", (int)c);
    }
    return;
  }

  shm_gps_set_shi4(l1ca_sid.sat, !data->nav_msg.alert);

  if (dd.shi1_upd_flag) {
    log_debug_mesid(channel_info->mesid, "SHI1: 0x%" PRIx8, dd.shi1);
    shm_gps_set_shi1(l1ca_sid.sat, dd.shi1);
  }

  /* Health indicates CODE_NAV_STATE_INVALID for L2CM */
  gnss_signal_t l2cm_sid = construct_sid(CODE_GPS_L2CM, l1ca_sid.sat);
  if (shm_signal_unhealthy(l2cm_sid)) {
    /* Clear CNAV data and TOW cache */
    erase_cnav_data(l2cm_sid, l1ca_sid);
  }

  /* Health indicates CODE_NAV_STATE_INVALID */
  if (shm_signal_unhealthy(l1ca_sid)) {
    /* Clear NDB and TOW cache */
    erase_nav_data(l1ca_sid, l1ca_sid);
    /* Clear decoded subframe data */
    nav_msg_clear_decoded(&data->nav_msg);
    return;
  }

  /* Do not use data from sv that is not declared as CODE_NAV_STATE_VALID. */
  if (!shm_navigation_suitable(l1ca_sid)) {
    return;
  }

  if (dd.gps_l2c_sv_capability_upd_flag) {
    /* store new L2C value into NDB */
    if (ndb_gps_l2cm_l2c_cap_store(&l1ca_sid,
                                   &dd.gps_l2c_sv_capability,
                                   NDB_DS_RECEIVER,
                                   NDB_EVENT_SENDER_ID_VOID) == NDB_ERR_NONE) {
      sbp_send_l2c_capabilities(&dd.gps_l2c_sv_capability);
    }
  }

  if (dd.iono_corr_upd_flag) {
    /* check if IONO info isn't older than the TOA */
    if (check_iono_timestamp(l1ca_sid, &dd.iono)) {
      /* send IONO SBP unless another satellite sent it recently  */
      DO_EACH_MS(IONO_HYSTERESIS_MS, sbp_send_iono(&dd.iono););
      /* store the new IONO (will return NDB_ERR_NO_CHANGE if no diff) */
      ndb_op_code_t oc = ndb_iono_corr_store(
          &l1ca_sid, &dd.iono, NDB_DS_RECEIVER, NDB_EVENT_SENDER_ID_VOID);
      if ((NDB_ERR_NO_CHANGE != oc) && (NDB_ERR_NONE != oc)) {
        log_error_mesid(channel_info->mesid, "error storing IONO params");
      }
    }
  }

  if (dd.utc_params_upd_flag) {
    /* store new utc parameters */
    if (ndb_utc_params_store(
            &l1ca_sid, &dd.utc, NDB_DS_RECEIVER, NDB_EVENT_SENDER_ID_VOID) ==
        NDB_ERR_NONE) {
      /*TODO: sbp_send_utc_params(&dd.utc); */
    }
  }

  if (dd.ephemeris_upd_flag) {
    /* Store new ephemeris to NDB */
    log_debug_mesid(channel_info->mesid,
                    "New ephemeris received [%" PRId16 ", %lf]",
                    dd.ephemeris.toe.wn,
                    dd.ephemeris.toe.tow);
    eph_new_status_t r = ephemeris_new(&dd.ephemeris);

    switch (r) {
      case EPH_NEW_OK:
        /* if time is not already known, construct coarse time from ephemeris'
         * time of week and decoded TOW_ms */
        if (TIME_UNKNOWN == get_time_quality()) {
          gps_time_t t = GPS_TIME_UNKNOWN;
          tracker_t *tracker_channel =
              tracker_get(channel_info->tracking_channel);
          chMtxLock(&tracker_channel->mutex);
          t.tow = (double)tracker_channel->nav_data_sync.TOW_ms / SECS_MS +
                  GPS_NOMINAL_RANGE / GPS_C;
          chMtxUnlock(&tracker_channel->mutex);
          gps_time_match_weeks(&t, &dd.ephemeris.toe);
          /* Initialize clock with accuracy of 1 ms */
          set_time(nap_timing_count(), &t, 1e-3);
        }
        break;
      case EPH_NEW_ERR:
        break;
      case EPH_NEW_XCORR:
        log_info_mesid(channel_info->mesid,
                       "Channel cross-correlation detected "
                       "(ephe/ephe or ephe/alm check)");
        /* Ephemeris cross-correlates with almanac of another SV */
        tracker_set_xcorr_flag(channel_info->mesid);
        break;
      default:
        break;
    }
  }

  if (dd.almanac_upd_flag) {
    /* Store new almanac to NDB*/
    log_debug_mesid(channel_info->mesid,
                    "New almanac received [%" PRId16 ", %lf]",
                    dd.almanac.toa.wn,
                    dd.almanac.toa.tow);

    decode_almanac_new(l1ca_sid, &dd.almanac);
  }
  if (dd.almanac_time_upd_flag) {
    /* Store new almanac time to NDB*/
    log_debug_mesid(channel_info->mesid,
                    "New almanac time received [%" PRId16 ", %" PRId32 "]",
                    dd.almanac_time.wn,
                    (s32)dd.almanac_time.tow);

    decode_almanac_time_new(l1ca_sid, &dd.almanac_time);
  }
  if (0 != dd.almanac_health_upd_flags) {
    log_debug_mesid(channel_info->mesid,
                    "New almanac health update received [0x08%" PRIX32 "]",
                    dd.almanac_health_upd_flags);
    /* Erase bad almanacs/ephemeris and update health flags for others */
    decode_almanac_health_new(
        l1ca_sid, dd.almanac_health_upd_flags, dd.almanac_health);
  }
}
