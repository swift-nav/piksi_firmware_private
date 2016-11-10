/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#define NDB_WEAK

#include <string.h>
#include <assert.h>
#include "ndb.h"
#include "ndb_internal.h"
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <timing.h>
#include <signal.h>
#include <sbp.h>
#include <sbp_utils.h>
#include "settings.h"
#include "ndb_fs_access.h"

#define NDB_EPHE_FILE_NAME   "persistent/ephemeris"

static ephemeris_t ndb_ephemeris[PLATFORM_SIGNAL_COUNT] _CCM;
static ndb_element_metadata_t ndb_ephemeris_md[PLATFORM_SIGNAL_COUNT];
static ndb_file_t ndb_ephe_file = {
    .name = NDB_EPHE_FILE_NAME,
    .expected_size =
          sizeof(ephemeris_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_file_end_mark),
    .data_size = sizeof(ephemeris_t),
    .n_elements = PLATFORM_SIGNAL_COUNT,
};

typedef struct {
  ephemeris_t ephe;
  bool used;
  ndb_timestamp_t received_at;
} ephemeris_candidate_t;

typedef enum {
  EPHE_IDENTICAL,
  EPHE_NEW_CANDIDATE,
  EPHE_NEW_TRUSTED,
  EPHE_CAND_MISMATCH,
} ndb_ephemeris_status_t;

#define EPHE_CAND_LIST_LEN (MAX_CHANNELS)
#define MAX_EPHE_CANDIDATE_AGE 92 /* seconds */
static ephemeris_candidate_t ephe_candidates[EPHE_CAND_LIST_LEN] _CCM;
static MUTEX_DECL(cand_list_access);

#define EPHEMERIS_MESSAGE_SPACING_cycle        (200 / NV_WRITE_REQ_TIMEOUT)
#define EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle (15000 / NV_WRITE_REQ_TIMEOUT)

void ndb_ephemeris_init(void)
{
  static bool erase_ephemeris = true;
  SETTING("ndb", "erase_ephemeris", erase_ephemeris, TYPE_BOOL);
  if (erase_ephemeris) {
    ndb_fs_remove(NDB_EPHE_FILE_NAME);
  }

  memset(ephe_candidates, 0, sizeof(ephe_candidates));

  ndb_load_data(&ndb_ephe_file, "ephemeris", (u8 *)ndb_ephemeris, ndb_ephemeris_md,
                sizeof(ephemeris_t), PLATFORM_SIGNAL_COUNT);
  u32 loaded = 0;
  for (size_t i = 0; i < PLATFORM_SIGNAL_COUNT; ++i) {
    if (0 != (ndb_ephemeris_md[i].nv_data.state & NDB_IE_VALID)) {
      loaded++;
    }
  }
  if (0 != loaded) {
    if (erase_ephemeris) {
      log_error("NDB ephemeris erase is not working");
    }

    log_info("Loaded %" PRIu32 " ephemeris", loaded);
  }
}

static bool ndb_ephemeris_validate(const ephemeris_t *e) {
  /* Check SID is valid */
  if (!sid_valid(e->sid)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: sid invalid");
    return false;
  }

  /* Check ToE is valid */
  if (!isfinite(e->toe.tow) ||
      !gps_current_time_valid(&e->toe)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: toe invalid");
    return false;
  }

  /* Currently we only support validating GPS ephemeris */
  /* TODO(Leith): Expand to GLONASS and SBAS */
  if (sid_to_constellation(e->sid) != CONSTELLATION_GPS) {
    return true;
  }

  /* Check ToE is valid */
  if (e->toe.tow > GPS_LNAV_EPH_TOE_MAX) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: toe.tow invalid");
    return false;
  }

  /* Check ToC is valid */
  if (!isfinite(e->kepler.toc.tow) ||
      e->kepler.toc.tow > GPS_LNAV_EPH_TOC_MAX ||
      !gps_current_time_valid(&e->kepler.toc)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: toc invalid");
    return false;
  }

  /* Check fit interval, 4 hours is min and 98 hours is max possible value */
  if ((e->fit_interval < 4 * HOUR_SECS) ||
      (e->fit_interval > 98 * HOUR_SECS)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: fit_interval invalid");
    return false;
  }

  /* Ensure that we are currently in the valid time range.
   *
   * NOTE: we specifically don't check the health status as we need to be able
   * to save the unhealthy status to ensure we stop using it.
   *
   * We also ensure we have at least COARSE time to prevent issues during
   * receiver start up when we don't know current time yet so just need to
   * use ephemeris as is. After first SPP fix we will know time so we will
   * throw out any out of date ephemeris before the second SPP fix. */
  if (time_quality >= TIME_COARSE) {
    gps_time_t t = get_current_time();
    if (ephemeris_valid(e, &t) == 0) {
      log_info_sid(e->sid, "ndb_ephemeris_validate: not ephemeris_valid()");
      return false;
    }
  }

  /* Check URA, min 2 m is max is 6144 m */
  if (!isfinite(e->ura) ||
      (e->ura < 2.0) ||
      (e->ura > 6144.0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: ura invalid");
    return false;
  }

  /* Check health bits, it is 6 bit field, so anything larger is invalid */
  if (e->health_bits > 0x3F) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: health_bits invalid");
    return false;
  }

  /* Check TGD */
  /* Range is signed 8bit * 2^-31 */
  if (!isfinite(e->kepler.tgd) ||
      (e->kepler.tgd < -128 * GPS_LNAV_EPH_SF_TGD) ||
      (e->kepler.tgd > 127 * GPS_LNAV_EPH_SF_TGD)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: tgd invalid");
    return false;
  }

  /* Check C_rc */
  /* Range is signed 16bit * 2^-5 */
  if (!isfinite(e->kepler.crc) ||
      (e->kepler.crc < -32768 * GPS_LNAV_EPH_SF_CRC) ||
      (e->kepler.crc > 32767 * GPS_LNAV_EPH_SF_CRC)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: crc invalid");
    return false;
  }

  /* Check C_rs */
  /* Range is signed 16bit * 2^-5 */
  if (!isfinite(e->kepler.crs) ||
      (e->kepler.crs < -32768 * GPS_LNAV_EPH_SF_CRS) ||
      (e->kepler.crs > 32767 * GPS_LNAV_EPH_SF_CRS)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: crs invalid");
    return false;
  }

  /* Check C_uc */
  /* Range is signed 16bit * 2^-29 */
  if (!isfinite(e->kepler.cuc) ||
      (e->kepler.cuc < -32768 * GPS_LNAV_EPH_SF_CUC) ||
      (e->kepler.cuc > 32767 * GPS_LNAV_EPH_SF_CUC)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: cuc invalid");
    return false;
  }

  /* Check C_us */
  /* Range is signed 16bit * 2^-29 */
  if (!isfinite(e->kepler.cus) ||
      (e->kepler.cus < -32768 * GPS_LNAV_EPH_SF_CUS) ||
      (e->kepler.cus > 32767 * GPS_LNAV_EPH_SF_CUS)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: cus invalid");
    return false;
  }

  /* Check C_ic */
  /* Range is signed 16bit * 2^-29 */
  if (!isfinite(e->kepler.cic) ||
      (e->kepler.cic < -32768 * GPS_LNAV_EPH_SF_CIC) ||
      (e->kepler.cic > 32767 * GPS_LNAV_EPH_SF_CIC)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: cic invalid");
    return false;
  }

  /* Check C_is */
  /* Range is signed 16bit * 2^-29 */
  if (!isfinite(e->kepler.cis) ||
      (e->kepler.cis < -32768 * GPS_LNAV_EPH_SF_CIS) ||
      (e->kepler.cis > 32767 * GPS_LNAV_EPH_SF_CIS)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: cis invalid");
    return false;
  }

  /* Check delta-n */
  /* Range is signed 16bit * 2^-43 */
  if (!isfinite(e->kepler.dn) ||
      (e->kepler.dn < -32768 * GPS_LNAV_EPH_SF_DN) ||
      (e->kepler.dn > 32767 * GPS_LNAV_EPH_SF_DN)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: dn invalid = %.12e", e->kepler.dn);
    return false;
  }

  /* Check M_0 */
  /* Range is signed 32bit * 2^-31 */
  if (!isfinite(e->kepler.m0) ||
      (e->kepler.m0 < -2147483648 * GPS_LNAV_EPH_SF_M0) ||
      (e->kepler.m0 > 2147483647 * GPS_LNAV_EPH_SF_M0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: m0 invalid");
    return false;
  }

  /* Check e */
  /* Range is 0 to 0.03 */
  if (!isfinite(e->kepler.ecc) ||
      (e->kepler.ecc < 0.0) ||
      (e->kepler.ecc > 0.03)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: ecc invalid");
    return false;
  }

  /* Check sqrt(A) */
  /* Range is 4906 to 5390 */
  if (!isfinite(e->kepler.sqrta) ||
      (e->kepler.sqrta < 4906.0) ||
      (e->kepler.sqrta > 5390.0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: sqrta invalid");
    return false;
  }

  /* Check omega_0 */
  /* Range is 32bit * 2^-31 */
  if (!isfinite(e->kepler.omega0) ||
      (e->kepler.omega0 < -2147483648 * GPS_LNAV_EPH_SF_OMEGA0) ||
      (e->kepler.omega0 > 2147483647 * GPS_LNAV_EPH_SF_OMEGA0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: omega0 invalid");
    return false;
  }

  /* Check omega-dot */
  /* Range is -5.20E-09 to 0.0 */
  if (!isfinite(e->kepler.omegadot) ||
      (e->kepler.omegadot < -5.20e-9) ||
      (e->kepler.omegadot > 0.0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: omegadot invalid");
    return false;
  }

  /* Check w */
  /* Range is 32bit * 2^-31 */
  if (!isfinite(e->kepler.w) ||
      (e->kepler.w < -2147483648 * GPS_LNAV_EPH_SF_W) ||
      (e->kepler.w > 2147483647 * GPS_LNAV_EPH_SF_W)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: w invalid");
    return false;
  }

  /* Check i_0 */
  /* Range is 0.237 to 0.363 */
  if (!isfinite(e->kepler.inc) ||
      (e->kepler.inc < 0.237 * GPS_LNAV_EPH_SF_I0) ||
      (e->kepler.inc > 0.363 * GPS_LNAV_EPH_SF_I0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: inc invalid");
    return false;
  }

  /* Check IDOT */
  /* Range is 14bit * 2^-43 */
  if (!isfinite(e->kepler.inc_dot) ||
      (e->kepler.inc_dot < -8192 * GPS_LNAV_EPH_SF_IDOT) ||
      (e->kepler.inc_dot > 8191 * GPS_LNAV_EPH_SF_IDOT)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: inc_dot invalid");
    return false;
  }

  /* Check a_f0 */
  /* Range is 22bit * 2^-31 */
  if (!isfinite(e->kepler.af0) ||
      (e->kepler.af0 < -2097152 * GPS_LNAV_EPH_SF_AF0) ||
      (e->kepler.af0 > 2097151 * GPS_LNAV_EPH_SF_AF0)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: af0 invalid");
    return false;
  }

  /* Check a_f1 */
  /* Range is 16bit * 2^-43 */
  if (!isfinite(e->kepler.af1) ||
      (e->kepler.af1 < -32768 * GPS_LNAV_EPH_SF_AF1) ||
      (e->kepler.af1 > 32767 * GPS_LNAV_EPH_SF_AF1)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: af1 invalid");
    return false;
  }

  /* Check a_f2 */
  /* Range is 8bit * 2^-55 */
  if (!isfinite(e->kepler.af2) ||
      (e->kepler.af2 < -128 * GPS_LNAV_EPH_SF_AF2) ||
      (e->kepler.af2 > 127 * GPS_LNAV_EPH_SF_AF2)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: af2 invalid");
    return false;
  }

  /* Check IODC */
  /* Range is 0 to 1023 */
  if (e->kepler.iodc > 1023) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: iodc invalid");
    return false;
  }

  /* Check IODE */
  /* Must be equal to 8 LSBs of IODC */
  if (e->kepler.iode == (e->kepler.iodc & 0xFF)) {
    log_info_sid(e->sid, "ndb_ephemeris_validate: iode invalid");
    return false;
  }

  /* TODO(Leith): Perform alamanac cross validation */

  return true;
}

ndb_op_code_t ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e)
{
  u16 idx;
  /*
   * Current architecture uses GPS L1 C/A ephemeris for GPS satellites.
   */
  if (sid_to_constellation(sid) == CONSTELLATION_GPS) {
    idx = sid_to_global_index(construct_sid(CODE_GPS_L1CA, sid.sat));
  } else {
    idx = sid_to_global_index(sid);
  }

  if (PLATFORM_SIGNAL_COUNT <= idx) {
    return NDB_ERR_BAD_PARAM;
  }

  ndb_op_code_t res = ndb_retrieve(e, &ndb_ephemeris_md[idx]);

  if ((res == NDB_ERR_NONE) &&
      !ndb_ephemeris_validate(e)) {
    log_warn_sid(sid, "NDB: Invalid ephemeris data retreived. Erasing.");
    //ndb_erase(&ndb_ephemeris_md[idx]);
    return NDB_ERR_UNRELIABLE_DATA;
  }

  /* Patch SID to be accurate for GPS L1/L2 */
  e->sid = sid;
  return res;
}

s16 ndb_ephe_find_candidate(const ephemeris_t *new)
{
  int i;
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    if (ephe_candidates[i].used &&
        sid_is_equal(ephe_candidates[i].ephe.sid, new->sid))
      return i;
  }
  return -1;
}

void ndb_ephe_try_adding_candidate(const ephemeris_t *new)
{
  int i;
  u32 candidate_age;
  ndb_timestamp_t now  = ndb_get_timestamp();
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    bool empty = true;
    if(ephe_candidates[i].used) {
      candidate_age = ST2S(now - ephe_candidates[i].received_at);
      empty = candidate_age >  MAX_EPHE_CANDIDATE_AGE;
    }

    if (empty) {
      memcpy(&ephe_candidates[i].ephe, new, sizeof(ephemeris_t));
      ephe_candidates[i].received_at = ndb_get_timestamp();
      ephe_candidates[i].used = true;
      return;
    }
  }
}

void ndb_ephe_release_candidate(s16 cand_index)
{
  if((cand_index < 0) || (cand_index >= EPHE_CAND_LIST_LEN))
    return;
  ephe_candidates[cand_index].used = false;
}

ndb_ephemeris_status_t ndb_get_ephemeris_status(const ephemeris_t *new)
{
  ndb_ephemeris_status_t r = EPHE_CAND_MISMATCH;
  ephemeris_t existing;
  ndb_ephemeris_read(new->sid, &existing);

  chMtxLock(&cand_list_access);

  if (!existing.valid) {
    chMtxUnlock(&cand_list_access);
    return EPHE_NEW_TRUSTED;
  }

  s16 cand_idx = ndb_ephe_find_candidate(new);

  /* Ephemeris for this SV was stored to the database already */
  if (memcmp(&existing, new, sizeof(ephemeris_t)) == 0) {
    /* New one is identical to the one in DB, no need to do anything */
    if (cand_idx != -1)
      ndb_ephe_release_candidate(cand_idx);
    chMtxUnlock(&cand_list_access);
    return EPHE_IDENTICAL;
  }

  if (cand_idx != -1) {
    /* Candidate was added already */
    r = memcmp(&ephe_candidates[cand_idx].ephe, new, sizeof(ephemeris_t)) == 0 ?
        EPHE_NEW_TRUSTED : EPHE_CAND_MISMATCH;
    ndb_ephe_release_candidate(cand_idx);
  } else {
    /* New one is not in candidate list yet, try to put it
     * to an empty slot */
    ndb_ephe_try_adding_candidate(new);
    r = EPHE_NEW_CANDIDATE;
  }

  chMtxUnlock(&cand_list_access);
  return r;
}

ndb_op_code_t ndb_ephemeris_store(const ephemeris_t *e, ndb_data_source_t src)
{
  if (!ndb_ephemeris_validate(e)) {
    log_warn("NDB: Invalid ephemeris was attempted to be stored.");
    return NDB_ERR_BAD_PARAM;
  }

  if (NDB_DS_RECEIVER == src) {
    /* TODO(Leith): do we want to store ephemeris if elevation < 5 degrees? */
    switch (ndb_get_ephemeris_status(e)) {
      case EPHE_IDENTICAL:
        return NDB_ERR_NONE;
      case EPHE_NEW_TRUSTED:
      {
        u16 idx = sid_to_global_index(e->sid);
        return ndb_update(e, src, &ndb_ephemeris_md[idx]);
      }
      case EPHE_NEW_CANDIDATE:
      case EPHE_CAND_MISMATCH:
        return NDB_ERR_UNRELIABLE_DATA;
      default:
        assert(!"Invalid status");
    }
  } else if (NDB_DS_SBP == src) {
    u8 valid, health_bits;
    gps_time_t toe;
    u32 fit_interval;
    float ura;
    ndb_ephemeris_info(e->sid, &valid, &health_bits, &toe, &fit_interval, &ura);
    if (!valid || gpsdifftime(&e->toe, &toe) > 0) {
    /* If local ephemeris is not valid or received one is newer then
     * save the received one. */
      log_debug_sid(e->sid,
                    "Saving ephemeris received over SBP v:%d [%d,%d] vs [%d,%d]",
                    (int)valid, toe.wn, toe.tow, e->toe.wn, e->toe.tow);
      u16 idx = sid_to_global_index(e->sid);
      return ndb_update(e, src, &ndb_ephemeris_md[idx]);
    }
    return NDB_ERR_NONE;
  }
  assert(!"ndb_ephemeris_store()");
  return NDB_ERR_ALGORITHM_ERROR;
}

ndb_op_code_t ndb_ephemeris_info(gnss_signal_t sid, u8* valid,
                                 u8* health_bits, gps_time_t* toe,
                                 u32* fit_interval, float* ura)
{
  assert(valid != NULL);
  assert(health_bits != NULL);
  assert(toe != NULL);
  assert(fit_interval != NULL);
  assert(ura != NULL);
  u16 idx = sid_to_global_index(sid);
  ndb_lock();
  *valid = ndb_ephemeris[idx].valid;
  *health_bits = ndb_ephemeris[idx].health_bits;
  *toe = ndb_ephemeris[idx].toe;
  *fit_interval = ndb_ephemeris[idx].fit_interval;
  *ura = ndb_ephemeris[idx].ura;
  ndb_unlock();
  return NDB_ERR_NONE;
}

/** Determine next index of the ephemeris to be sent over SBP.
 *  This function takes previous index (can be set to PLATFORM_SIGNAL_COUNT to
 *  indicate no previous value available) and increments it by one making sure
 *  that index is within codes that contain 'original' ephemerides. This is
 *  necessary to prevent outputting the same ephemeris for different codes
 *  of the same satellite.
 *  */
static u32 get_next_idx_to_send(gnss_signal_t *sid, u32 prev_idx)
{
  u32 i = prev_idx != PLATFORM_SIGNAL_COUNT ? prev_idx + 1 : 0;

  while (i < PLATFORM_SIGNAL_COUNT) {
    *sid = sid_from_global_index(i);
    if (sid->code != CODE_GPS_L1CA &&
        sid->code != CODE_SBAS_L1CA &&
        sid->code != CODE_GLO_L1CA) {
      i++;
    } else {
      break;
    }
  }

  return i;
}

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_ephemeris_sbp_update(void)
{
  static u32 count = 0;
  static u32 i = PLATFORM_SIGNAL_COUNT;
  static bool tx_en = true; /* initially enable SBP TX */

  if (tx_en) {
    if (!(count % EPHEMERIS_MESSAGE_SPACING_cycle)) {
      /* every 200 ms send eph of a SV */
      ephemeris_t e;
      gnss_signal_t sid;

      do {
        i = get_next_idx_to_send(&sid, i);
        tx_en = (i != PLATFORM_SIGNAL_COUNT) ? true : false;

        if (tx_en) {
          enum ndb_op_code oc = ndb_ephemeris_read(sid, &e);
          if (NDB_ERR_NONE == oc) {
            gps_time_t t = get_current_time();
            if (ephemeris_valid(&e, &t)) {
              msg_ephemeris_t msg;
              msg_ephemeris_info_t info = pack_ephemeris(&e, &msg);
              sbp_send_msg(info.msg_id, info.size, (u8 *)&msg);
              break;
            }
          }
        }
      } while(tx_en);
    }
  } else {
    if (!(count % EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle)) {
      /* every 15 sec enable tx again */
      count = 0;
      i = PLATFORM_SIGNAL_COUNT;
      tx_en = true;
      return;
    }
  }

  count++;
}
