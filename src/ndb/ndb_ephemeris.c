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
#define NDB_EPHE_FILE_TYPE   "ephemeris"

static ephemeris_t ndb_ephemeris[PLATFORM_SIGNAL_COUNT];
static ndb_element_metadata_t ndb_ephemeris_md[PLATFORM_SIGNAL_COUNT];
static ndb_file_t ndb_ephe_file = {
    .name = NDB_EPHE_FILE_NAME,
    .type = NDB_EPHE_FILE_TYPE,
    .block_data = (u8*)&ndb_ephemeris[0],
    .block_md = &ndb_ephemeris_md[0],
    .block_size = sizeof(ndb_ephemeris[0]),
    .block_count = sizeof(ndb_ephemeris) / sizeof(ndb_ephemeris[0]),
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

  ndb_load_data(&ndb_ephe_file);
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

  ndb_op_code_t res = ndb_retrieve(&ndb_ephemeris_md[idx], e, sizeof(*e),
                                   NULL, NULL);
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
  if (!e->valid) {
    return NDB_ERR_BAD_PARAM;
  }

  if (NDB_DS_RECEIVER == src) {
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
