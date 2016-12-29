/*
 * Copyright (C) 2011 - 2017 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "decode_gps_l1ca.h"
#include "decode.h"

#include <libswiftnav/logging.h>
#include <libswiftnav/nav_msg.h>

#include "ephemeris.h"
#include "track.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "signal.h"
#include "ndb.h"
#include "shm.h"

#include <assert.h>
#include <string.h>

/** GPS L1 C/A decoder data */
typedef struct {
  nav_msg_t nav_msg;
} gps_l1ca_decoder_data_t;

static decoder_t gps_l1ca_decoders[NUM_GPS_L1CA_DECODERS];
static gps_l1ca_decoder_data_t gps_l1ca_decoder_data[ARRAY_SIZE(gps_l1ca_decoders)];

static void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data);
static void decoder_gps_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);
static void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data);

static const decoder_interface_t decoder_interface_gps_l1ca = {
  .code =         CODE_GPS_L1CA,
  .init =         decoder_gps_l1ca_init,
  .disable =      decoder_gps_l1ca_disable,
  .process =      decoder_gps_l1ca_process,
  .decoders =     gps_l1ca_decoders,
  .num_decoders = ARRAY_SIZE(gps_l1ca_decoders)
};

static decoder_interface_list_element_t list_element_gps_l1ca = {
  .interface = &decoder_interface_gps_l1ca,
  .next = NULL
};

/**
 * For a newly saved almanac check if it correlates with any of the tracked
 * satellite ephemeris.
 *
 * \param[in] sid GNSS signal identifier for which almanac has been updated
 *
 * \return None
 */
static void check_almanac_xcorr(gnss_signal_t sid)
{
  xcorr_positions_t alm_pos; /* Almanac's positions for sid */
  alm_pos.time_s = 0;

  for (u8 sv_idx = 0; sv_idx < NUM_SATS_GPS; ++sv_idx) {
    gnss_signal_t sid1 = construct_sid(CODE_GPS_L1CA, sv_idx + GPS_FIRST_PRN);

    ephemeris_t e;
    if (NDB_ERR_NONE == ndb_ephemeris_read(sid1, &e) && e.toe.wn > 0) {
      u32 time_s = (u32)e.toe.wn * WEEK_SECS + (s32)e.toe.tow;
      u32 interval_s = e.fit_interval / 2;
      if (alm_pos.time_s != time_s || alm_pos.interval_s != interval_s) {
        /* If ephemeris time differs from last computed almanac time, or the
         * first one, compute the almanac positions for the new time */
        if (!xcorr_calc_alm_positions(sid, time_s, interval_s, &alm_pos)) {
          log_debug_sid(sid1, "Failed to compute almanac's positions");
          continue;
        }
      }

      xcorr_positions_t eph_pos;
      if (!xcorr_calc_eph_positions(&e, time_s, &eph_pos)) {
        log_debug_sid(sid1, "Failed to compute ephemeris positions");
      } else {
        bool match = xcorr_match_positions(sid, sid1, &eph_pos, &alm_pos);

        if (sid1.sat == sid.sat) {
          if (match) {
            /* OK */
          } else {
            log_warn_sid(sid1, "Position mismatch with own almanac");
            ndb_ephemeris_erase(sid1);
          }
        } else if (match) {
          /* Cross-correlation */
          char sid_str_[SID_STR_LEN_MAX];
          sid_to_string(sid_str_, sizeof(sid_str_), sid);

          log_warn_sid(sid1, "Cross-correlation detected with %s", sid_str_);
          ndb_ephemeris_erase(sid1);
          tracking_channel_set_xcorr_flag(sid1);
        }
      }
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
static void check_almanac_wn_xcorr(s16 wn, s32 toa)
{
  for (u8 sv_idx = 0; sv_idx < NUM_SATS_GPS; ++sv_idx) {
    gnss_signal_t sid = construct_sid(CODE_GPS_L1CA, sv_idx + GPS_FIRST_PRN);
    almanac_t a;
    if (NDB_ERR_NONE == ndb_almanac_read(sid, &a) &&
        a.toa.wn == wn && (s32)a.toa.tow == toa) {
      check_almanac_xcorr(sid);
    }
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
static void decode_almanac_new(gnss_signal_t sid, const almanac_t *alma)
{
  ndb_op_code_t oc = ndb_almanac_store(&sid,
                                       alma,
                                       NDB_DS_RECEIVER,
                                       NDB_EVENT_SENDER_ID_VOID);
  char src_sid_str[SID_STR_LEN_MAX];
  sid_to_string(src_sid_str, sizeof(src_sid_str), sid);
  switch (oc) {
  case NDB_ERR_NONE:
    log_debug_sid(alma->sid, "almanac from %s saved", src_sid_str);
    check_almanac_xcorr(alma->sid);
    break;
  case NDB_ERR_NO_CHANGE:
    log_debug_sid(alma->sid, "almanac from %s is already present",
                 src_sid_str);
    break;
  case NDB_ERR_UNRELIABLE_DATA:
    log_debug_sid(alma->sid, "almanac from %s is unconfirmed, not saved",
                 src_sid_str);
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
  default:
    log_warn_sid(alma->sid, "error %d storing almanac from %s",
                  (int)oc,
                  src_sid_str);
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
                                    const gps_time_t *alma_time)
{
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
                 "almanac time info is already present (%" PRId16 ", %" PRId32 ")",
                 alma_time->wn,
                 (s32)alma_time->tow);
    break;
  case NDB_ERR_UNRELIABLE_DATA:
    log_debug_sid(sid,
                 "almanac time info is unconfirmed (%" PRId16 ", %" PRId32 ")",
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
 * otherwise.
 *
 * \param[in] src_sid    Health bits source SV
 * \param[in] hlags_mask Mask where new bits are set for valid entries in \a
 *                       hflags.
 * \param[in] hflags     Mask array for GPS satellites, where index is `PRN - 1`
 *
 * \return None
 */
void decode_almanac_health_new(gnss_signal_t src_sid,
                               u32 hlags_mask,
                               const u8 hflags[32])
{
  /* Copy updated flags into the cache, and update all entries in NDB */
  for (u16 sv_idx = 0; sv_idx < 32; ++sv_idx) {
    if (0 != (hlags_mask & 1u << sv_idx)) {

      gnss_signal_t target_sid = construct_sid(CODE_GPS_L1CA, sv_idx + 1);
      char hf_sid_str[SID_STR_LEN_MAX];
      sid_to_string(hf_sid_str, sizeof(hf_sid_str), src_sid);

      u8 health_bits = hflags[sv_idx];

      if (0 != (health_bits & 1 << 5)) {
        /* Error in almanac */
        if (NDB_ERR_NONE == ndb_almanac_erase(target_sid)) {
          log_info_sid(target_sid,
                       "almanac deleted (health flags from %s)",
                       hf_sid_str);
        }
        if (NDB_ERR_NONE == ndb_ephemeris_erase(target_sid)) {
          log_info_sid(target_sid,
                       "ephemeris deleted (health flags from %s)",
                       hf_sid_str);
        }
      } else {
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
        case NDB_ERR_UNRELIABLE_DATA:
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
        default:
          log_error_sid(target_sid,
                        "error %d updating almanac health bits (0x%02" PRIX8 ")",
                        (int)r,
                        health_bits);
          break;
        }
      }
    }
  }
}

void decode_gps_l1ca_register(void)
{
  for (u32 i = 0; i < ARRAY_SIZE(gps_l1ca_decoders); i++) {
    gps_l1ca_decoders[i].active = false;
    gps_l1ca_decoders[i].data = &gps_l1ca_decoder_data[i];
  }

  decoder_interface_register(&list_element_gps_l1ca);
}

static void decoder_gps_l1ca_init(const decoder_channel_info_t *channel_info,
                                  decoder_data_t *decoder_data)
{
  (void)channel_info;
  gps_l1ca_decoder_data_t *data = decoder_data;

  memset(data, 0, sizeof(*data));
  nav_msg_init(&data->nav_msg);
}

static void decoder_gps_l1ca_disable(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  (void)channel_info;
  (void)decoder_data;
}

static void decoder_gps_l1ca_process(const decoder_channel_info_t *channel_info,
                                     decoder_data_t *decoder_data)
{
  gps_l1ca_decoder_data_t *data = decoder_data;

  /* Process incoming nav bits */
  s8 soft_bit;
  bool sensitivity_mode = true;
  while (tracking_channel_nav_bit_get(channel_info->tracking_channel,
                                      &soft_bit, &sensitivity_mode)) {
    /* Don't trust polarity information while in sensitivity mode. */
    if (sensitivity_mode) {
      data->nav_msg.bit_polarity = BIT_POLARITY_UNKNOWN;
    }
    /* Update TOW */
    bool bit_val = soft_bit >= 0;
    s32 TOW_ms = nav_msg_update(&data->nav_msg, bit_val);
    s8 bit_polarity = data->nav_msg.bit_polarity;
    if ((TOW_ms >= 0) && (bit_polarity != BIT_POLARITY_UNKNOWN)) {
      if (!tracking_channel_time_sync(channel_info->tracking_channel, TOW_ms,
                                      bit_polarity)) {
        log_warn_sid(channel_info->sid, "TOW set failed");
      }
    }
  }

  /* Check if there is a new nav msg subframe to process. */
  if (!subframe_ready(&data->nav_msg))
    return;

  /* Decode nav data to temporary structure */
  gps_l1ca_decoded_data_t dd;
  s8 ret = process_subframe(&data->nav_msg, channel_info->sid, &dd);

  if (ret <= 0) {
    return;
  }

  if (dd.invalid_control_or_data) {
    log_info_sid(channel_info->sid, "Invalid control or data element");

    ndb_op_code_t c = ndb_ephemeris_erase(channel_info->sid);

    if (NDB_ERR_NONE == c) {
      log_info_sid(channel_info->sid, "ephemeris deleted (1/0)");
    } else if (NDB_ERR_NO_CHANGE != c){
      log_warn_sid(channel_info->sid, "error %d deleting ephemeris (1/0)",
                   (int)c);
    }
    return;
  }

  shm_gps_set_shi4(channel_info->sid.sat, false == data->nav_msg.alert);

  if (dd.shi1_upd_flag) {
    log_debug_sid(channel_info->sid, "SHI1: 0x%" PRIx8, dd.shi1);
    shm_gps_set_shi1(channel_info->sid.sat, dd.shi1);
  }

  if (dd.gps_l2c_sv_capability_upd_flag) {
    /* store new L2C value into NDB */
    log_debug_sid(channel_info->sid, "L2C capabilities received: 0x%08"PRIx32,
                  dd.gps_l2c_sv_capability);
    if (ndb_gps_l2cm_l2c_cap_store(&channel_info->sid,
                                   &dd.gps_l2c_sv_capability,
                                   NDB_DS_RECEIVER,
                                   NDB_EVENT_SENDER_ID_VOID) ==
        NDB_ERR_NONE) {
      sbp_send_l2c_capabilities(&dd.gps_l2c_sv_capability);
    }
  }

  if (dd.iono_corr_upd_flag) {
    /* store new iono parameters */
    log_debug_sid(channel_info->sid, "Iono parameters received");

    if (ndb_iono_corr_store(&channel_info->sid,
                            &dd.iono,
                            NDB_DS_RECEIVER,
                            NDB_EVENT_SENDER_ID_VOID) ==
        NDB_ERR_NONE) {
      sbp_send_iono(&dd.iono);
    }
  }

  if (dd.ephemeris_upd_flag) {
    /* Store new ephemeris to NDB*/
    log_debug_sid(channel_info->sid, "New ephemeris received [%" PRId16 ", %lf]",
                  dd.ephemeris.toe.wn, dd.ephemeris.toe.tow);
    eph_new_status_t r = ephemeris_new(&dd.ephemeris);
    switch (r) {
    case EPH_NEW_OK:
    case EPH_NEW_ERR:
      break;
    case EPH_NEW_XCORR:
      log_info_sid(channel_info->sid,
                   "Channel cross-correlation detected (ephe/alm check)");
      /* Ephemeris cross-correlates with almanac of another SV */
      tracking_channel_set_xcorr_flag(channel_info->sid);
      break;
    default:
      break;
    }
  }

  if (dd.almanac_upd_flag) {
    /* Store new almanac to NDB*/
    log_debug_sid(channel_info->sid, "New almanac received [%"  PRId16 ", %lf]",
                  dd.almanac.toa.wn, dd.almanac.toa.tow);

    decode_almanac_new(channel_info->sid, &dd.almanac);
  }
  if (dd.almanac_time_upd_flag) {
    /* Store new almanac time to NDB*/
    log_debug_sid(channel_info->sid,
                  "New almanac time received [%" PRId16 ", %" PRId32 "]",
                  dd.almanac_time.wn, (s32)dd.almanac_time.tow);

    decode_almanac_time_new(channel_info->sid, &dd.almanac_time);

  }
  if (0 != dd.almanac_health_upd_flags) {
    log_debug_sid(channel_info->sid,
                  "New almanac health update received [0x08%" PRIX32 "]",
                  dd.almanac_health_upd_flags);
    /* Erase bad almanacs/ephemeris and update health flags for others */
    decode_almanac_health_new(channel_info->sid,
                              dd.almanac_health_upd_flags,
                              dd.almanac_health);
  }
}
