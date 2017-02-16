/*
 * Copyright (C) 2016 - 2017 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
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

#define EPHE_CAND_LIST_LEN (MAX_CHANNELS)
#define MAX_EPHE_CANDIDATE_AGE 92 /* seconds */
static ephemeris_candidate_t ephe_candidates[EPHE_CAND_LIST_LEN];
static MUTEX_DECL(cand_list_access);

/** Minimum interval between two ephemeris transmission inside one transmission
    epoch [cycles] */
#define NDB_EPHE_MESSAGE_SPACING        (150 / NV_WRITE_REQ_TIMEOUT)
/** Minimum interval between ephemeris transmit epoch starts, can be longer
    if the amount of sent messages makes epoch longer [cycles] */
#define NDB_EPHE_TRANSMIT_EPOCH_SPACING (30000 / NV_WRITE_REQ_TIMEOUT)

static u16 map_sid_to_index(gnss_signal_t sid)
{
  u16 idx = PLATFORM_SIGNAL_COUNT;
  /*
   * Current architecture uses GPS L1 C/A ephemeris for GPS satellites.
   */
  if (sid_to_constellation(sid) == CONSTELLATION_GPS) {
    idx = sid_to_global_index(construct_sid(CODE_GPS_L1CA, sid.sat));
  } else {
    idx = sid_to_global_index(sid);
  }
  return idx;
}

void ndb_ephemeris_init(void)
{
  static bool erase_ephemeris = true;
  SETTING("ndb", "erase_ephemeris", erase_ephemeris, TYPE_BOOL);

  ndb_load_data(&ndb_ephe_file, erase_ephemeris);
}

ndb_op_code_t ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e)
{
  u16 idx = map_sid_to_index(sid);

  if (PLATFORM_SIGNAL_COUNT <= idx) {
    return NDB_ERR_BAD_PARAM;
  }

  ndb_op_code_t res = ndb_retrieve(&ndb_ephemeris_md[idx], e, sizeof(*e),
                                   NULL, NULL);
  /* Patch SID to be accurate for GPS L1/L2 */
  e->sid = sid;
  return res;
}

static s16 ndb_ephe_find_candidate(gnss_signal_t sid)
{
  int i;
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    if (ephe_candidates[i].used &&
        sid_is_equal(ephe_candidates[i].ephe.sid, sid))
      return i;
  }
  return -1;
}

static void ndb_ephe_try_adding_candidate(const ephemeris_t *new)
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

static void ndb_ephe_release_candidate(s16 cand_index)
{
  if((cand_index < 0) || (cand_index >= EPHE_CAND_LIST_LEN))
    return;
  ephe_candidates[cand_index].used = false;
}

static ndb_cand_status_t ndb_get_ephemeris_status(const ephemeris_t *new)
{
  ndb_cand_status_t r = NDB_CAND_MISMATCH;
  ephemeris_t existing;
  ndb_ephemeris_read(new->sid, &existing);

  chMtxLock(&cand_list_access);

  if (!existing.valid) {
    chMtxUnlock(&cand_list_access);
    return NDB_CAND_NEW_TRUSTED;
  }

  s16 cand_idx = ndb_ephe_find_candidate(new->sid);

  /* Ephemeris for this SV was stored to the database already */
  if (memcmp(&existing, new, sizeof(ephemeris_t)) == 0) {
    /* New one is identical to the one in DB, no need to do anything */
    if (cand_idx != -1)
      ndb_ephe_release_candidate(cand_idx);
    chMtxUnlock(&cand_list_access);
    return NDB_CAND_IDENTICAL;
  }

  if (cand_idx != -1) {
    /* Candidate was added already */
    r = memcmp(&ephe_candidates[cand_idx].ephe, new, sizeof(ephemeris_t)) == 0 ?
        NDB_CAND_NEW_TRUSTED : NDB_CAND_MISMATCH;
    ndb_ephe_release_candidate(cand_idx);
  } else {
    /* New one is not in candidate list yet, try to put it
     * to an empty slot */
    ndb_ephe_try_adding_candidate(new);
    r = NDB_CAND_NEW_CANDIDATE;
  }

  chMtxUnlock(&cand_list_access);
  return r;
}

static ndb_op_code_t ndb_ephemeris_store_do(const ephemeris_t *e,
                                            ndb_data_source_t src)
{
  if (!e->valid) {
    return NDB_ERR_BAD_PARAM;
  }

  if (NDB_DS_RECEIVER == src) {
    switch (ndb_get_ephemeris_status(e)) {
    case NDB_CAND_IDENTICAL:
      return NDB_ERR_NO_CHANGE;
    case NDB_CAND_OLDER:
      return NDB_ERR_OLDER_DATA;
    case NDB_CAND_NEW_TRUSTED:
    {
      u16 idx = sid_to_global_index(e->sid);
      return ndb_update(e, src, &ndb_ephemeris_md[idx]);
    }
    case NDB_CAND_NEW_CANDIDATE:
    case NDB_CAND_MISMATCH:
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

/**
 * Store ephemeris
 *
 * \param[in] e           Ephemeris
 * \param[in] src         Data source
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP. In other cases
 *                        set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE            On success. Ephemeris is persisted.
 * \retval NDB_ERR_NO_CHANGE       On success. The entry is already persisted.
 * \retval NDB_ERR_BAD_PARAM       Parameter errors.
 * \retval NDB_ERR_UNRELIABLE_DATA New entry, but confirmation is required.
 */
ndb_op_code_t ndb_ephemeris_store(const ephemeris_t *e,
                                  ndb_data_source_t src,
                                  u16 sender_id)
{
  ndb_op_code_t res = ndb_ephemeris_store_do(e, src);

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_EPHEMERIS,
                     res,
                     src,
                     &e->sid,
                     NULL,
                     sender_id);

  return res;
}

/**
 * Erase ephemeris data for a given satellite
 *
 * \param[in] sid SV signal identifier
 *
 * \retval NDB_ERR_NONE      Successful operation.
 * \retval NDB_ERR_NO_CHANGE No data to erase.
 * \retval NDB_ERR_BAD_PARAM Bad parameter.
 */
ndb_op_code_t ndb_ephemeris_erase(gnss_signal_t sid)
{
  u16 idx = map_sid_to_index(sid);

  if (PLATFORM_SIGNAL_COUNT <= idx) {
    return NDB_ERR_BAD_PARAM;
  }

  ndb_op_code_t res = ndb_erase(&ndb_ephemeris_md[idx]);

  chMtxLock(&cand_list_access);
  s16 cand_idx = ndb_ephe_find_candidate(sid);
  if (cand_idx >= 0) {
    ndb_ephe_release_candidate(cand_idx);
  }
  chMtxUnlock(&cand_list_access);

  sbp_send_ndb_event(NDB_EVENT_ERASE,
                     NDB_EVENT_OTYPE_EPHEMERIS,
                     res,
                     NDB_DS_UNDEFINED,
                     &sid,
                     NULL,
                     NDB_EVENT_SENDER_ID_VOID);

  return res;
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

/**
 * Sends out MsgEphemeris
 *
 * \param[in] sid  GNSS signal identifier to indicate which ephe to send
 *
 * \retval TRUE    Ephe found, valid and sent
 * \retval FALSE   Ephe not sent
 */
bool ndb_ephemeris_sbp_update_tx(gnss_signal_t sid)
{
  ephemeris_t e;
  gps_time_t t = get_current_time();
  enum ndb_op_code oc = ndb_ephemeris_read(sid, &e);
  if (NDB_ERR_NONE == oc && ephemeris_valid(&e, &t)) {
    msg_ephemeris_t msg;
    msg_info_t info = pack_ephemeris(&e, &msg);
    sbp_send_msg(info.msg_id, info.size, (u8 *)&msg);
    return TRUE;
  }

  return FALSE;
}

static ndb_sbp_update_info_t ephe_update_info = {
  NDB_SBP_UPDATE_CYCLE_COUNT_INIT,
  NDB_SBP_UPDATE_SIG_IDX_INIT,
  NDB_EPHE_TRANSMIT_EPOCH_SPACING,
  NDB_EPHE_MESSAGE_SPACING,
  &ndb_ephemeris_sbp_update_tx
};

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_ephemeris_sbp_update(void)
{
  ndb_sbp_update(&ephe_update_info);
}
