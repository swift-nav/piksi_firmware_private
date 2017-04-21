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

#include <ndb.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/linear_algebra.h>
#include <timing.h>
#include <signal.h>
#include <sbp.h>
#include <sbp_utils.h>
#include <settings.h>

#include "ndb_internal.h"
#include "ndb_fs_access.h"

#define NDB_EPHE_FILE_NAME   "persistent/ephemeris"
#define NDB_EPHE_FILE_TYPE   "ephemeris"

static ephemeris_t ndb_ephemeris[PLATFORM_SIGNAL_COUNT];
static ndb_element_metadata_t ndb_ephemeris_md[ARRAY_SIZE(ndb_ephemeris)];
static ndb_file_t ndb_ephe_file = {
    .name = NDB_EPHE_FILE_NAME,
    .type = NDB_EPHE_FILE_TYPE,
    .block_data = (u8*)&ndb_ephemeris[0],
    .block_md = &ndb_ephemeris_md[0],
    .block_size = sizeof(ndb_ephemeris[0]),
    .block_count = ARRAY_SIZE(ndb_ephemeris),
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

typedef struct {
  bool erase_ephemeris;    /**< Erase ephemeris data on boot */
  s16  valid_alm_accuracy; /**< Cross-checking accuracy with valid almanac [m] */
  s16  valid_eph_accuracy; /**< Cross-checking accuracy with valid ephemeris [m] */
  s16  alm_fit_interval;   /**< Almanac fit interval (days) */
} ndb_ephe_config_t;

static ndb_ephe_config_t ndb_ephe_config = {
  .erase_ephemeris = true,
  .valid_alm_accuracy = 5000,
  .valid_eph_accuracy = 100,
  .alm_fit_interval = 6,
};

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
  SETTING("ndb", "erase_ephemeris", ndb_ephe_config.erase_ephemeris, TYPE_BOOL);
  SETTING("ndb", "valid_alm_acc", ndb_ephe_config.valid_alm_accuracy, TYPE_INT);
  SETTING("ndb", "valid_eph_acc", ndb_ephe_config.valid_eph_accuracy, TYPE_INT);
  SETTING("ndb", "valid_alm_days", ndb_ephe_config.alm_fit_interval, TYPE_INT);

  ndb_load_data(&ndb_ephe_file, ndb_ephe_config.erase_ephemeris);
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

/* Find an empty slot (unused or outdated ephemeris) in the candidate list
 * and add the given candidate. Log a warning if no empty slot found.
 */
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
  log_warn_sid(new->sid, "Could not add ephemeris candidate (%d slots full)",
      EPHE_CAND_LIST_LEN);
}

static void ndb_ephe_release_candidate(s16 cand_index)
{
  if ((cand_index < 0) || (cand_index >= EPHE_CAND_LIST_LEN)) {
    return;
  }
  ephe_candidates[cand_index].used = false;
}

/**
 * Check if the new ephemeris seems to be correct
 *
 * \param[in] new        New ephemeris data
 * \param[in] existing_e Optional ephemeris from NDB
 * \param[in] existing_a Optional almanac from NDB
 * \param[in] candidate  Optional ephemeris candidate
 *
 * \retval true The ephemeris can be directly stored to NDB
 * \retval false The ephemeris data can't be verified and shall be stored as
 *               a new candidate.
 */
static bool ndb_can_confirm_ephemeris(const ephemeris_t *new,
                                      const ephemeris_t *existing_e,
                                      const almanac_t   *existing_a,
                                      const ephemeris_t *candidate)
{

  if (NULL != candidate && 0 == memcmp(new, candidate, sizeof(*new))) {
    /* Exact match */
    log_debug_sid(new->sid, "[EPH] candidate match");
    return true;
  } else if (NULL != existing_e && 0 == memcmp(new, existing_e, sizeof(*new))) {
    /* Exact match with stored */
    log_debug_sid(new->sid, "[EPH] NDB match");
    return true;
  }

  bool res = false;

  /* First check point: start of the position test interval */
  gps_time_t t_start = new->toe;
  t_start.tow += -MINUTE_SECS * 30;
  normalize_gps_time(&t_start);

  /* Last check point: end of the position test interval */
  gps_time_t t_end = t_start;
  t_end.tow += MINUTE_SECS * 30 * 2;
  normalize_gps_time(&t_end);

  if (NULL != existing_a &&
      almanac_valid(existing_a, &t_start) &&
      almanac_valid(existing_a, &t_end) &&
      ephemeris_valid(new, &t_start) &&
      ephemeris_valid(new, &t_end)) {

    /* Almanac position verification */

    bool ok = true;
    gps_time_t t = t_start;

    for (u8 i = 0; i < 3 && ok;
        ++i, t.tow += MINUTE_SECS * 30, normalize_gps_time(&t)) {

      double _[3];
      double alm_sat_pos[3];
      double eph_sat_pos[3];

      ok = false;

      u8 iode;
      u16 iodc;
      if (0 == calc_sat_state_almanac(existing_a, &t, alm_sat_pos, _, _, _) &&
          0 == calc_sat_state_n(new, &t, eph_sat_pos, _, _, _, &iode, &iodc)) {

        /* Compute distance [m] */
        double d = vector_distance(3, alm_sat_pos, eph_sat_pos);

        ok = (d <= ndb_ephe_config.valid_alm_accuracy);
        log_debug_sid(new->sid,
                     "[EPH] almanac position error %lf T=%" PRId16 ",%" PRId32,
                     d, t.wn, (s32)t.tow);
      }
    }

    res = ok;
    if (res) {
      log_debug_sid(new->sid, "[EPH] verified against almanac");
    }
  }

  if (!res &&
      NULL != existing_e &&
      ephemeris_valid(existing_e, &t_start) &&
      ephemeris_valid(existing_e, &t_end)) {
    /* Previous ephemeris, but still valid */

    bool ok = true;
    gps_time_t t = t_start;

    for (u8 i = 0; i < 3 && ok;
        ++i, t.tow += MINUTE_SECS * 30, normalize_gps_time(&t)) {

      double _[3];
      double old_sat_pos[3];
      double new_sat_pos[3];

      ok = false;

      u8 iode;
      u16 iodc;
      if (0 == calc_sat_state_n(existing_e, &t, old_sat_pos, _, _, _, &iode, &iodc) &&
          0 == calc_sat_state_n(new, &t, new_sat_pos, _, _, _, &iode, &iodc)) {

        /* Compute distance [m] */
        double d = vector_distance(3, old_sat_pos, new_sat_pos);

        ok = (d <= ndb_ephe_config.valid_eph_accuracy);
        log_debug_sid(new->sid, "[EPH] ephemeris position error %lf", d);
      }
    }

    res = ok;
    if (res) {
      log_debug_sid(new->sid, "[EPH] verified against NDB ephemeris");
    }
  } else if (!res) {
    log_debug_sid(new->sid, "[EPH] can't verify");
  }

  return res;
}

/**
 * Check the status of a new ephemeris candidate against NDB
 *
 * \param[in] new        New ephemeris data
 *
 * \retval NDB_CAND_NEW_CANDIDATE New candidate, cannot be confirmed yet
 * \retval NDB_CAND_NEW_TRUSTED   Candidate is confirmed, either by matching
 *                                to previous candidate, previous ephemeris or
 *                                almanac
 * \retval NDB_CAND_IDENTICAL     Candidate is identical to current ephemeris
 * \retval NDB_CAND_MISMATCH      Candidate differs from current ephemeris and
 *                                cannot be confirmed yet
 * \retval NDB_ERR_BAD_PARAM      Bad SID
 */
static ndb_cand_status_t ndb_get_ephemeris_status(const ephemeris_t *new)
{
  ndb_cand_status_t r = NDB_CAND_MISMATCH;

  ephemeris_t existing_e; /* Existing ephemeris data */
  ephemeris_t *pe = NULL; /* Existing ephemeris data pointer if valid */
  ephemeris_t *ce = NULL; /* Candidate ephemeris data pointer if valid */
  almanac_t existing_a;   /* Existing almanac data */
  almanac_t *pa = NULL;   /* Existing almanac data pointer if valid */

  u16 idx = map_sid_to_index(new->sid);
  if (ARRAY_SIZE(ndb_ephemeris) <= idx) {
    return NDB_ERR_BAD_PARAM;
  }

  if (NDB_ERR_NONE == ndb_retrieve(&ndb_ephemeris_md[idx], &existing_e,
                                   sizeof(existing_e), NULL, NULL)) {
    pe = &existing_e;
  }
  if (NDB_ERR_NONE == ndb_almanac_read(new->sid, &existing_a)) {
    pa = &existing_a;
    existing_a.fit_interval = ndb_ephe_config.alm_fit_interval * DAY_SECS;
  }

  chMtxLock(&cand_list_access);

  s16 cand_idx = ndb_ephe_find_candidate(new->sid);
  if (-1 != cand_idx) {
    ce = &ephe_candidates[cand_idx].ephe;
  }

  if (NULL != pe && 0 == memcmp(&existing_e, new, sizeof(ephemeris_t))) {
    /* New one is identical to the one in DB, no need to do anything */
    ndb_ephe_release_candidate(cand_idx);
    r = NDB_CAND_IDENTICAL;

    log_debug_sid(new->sid, "[EPH] identical");
  } else if (NULL != ce) {
    /* Candidate was added already */
    if (ndb_can_confirm_ephemeris(new, pe, pa, ce)) {
      /* New ephemeris matches candidate - confirm it */
      ndb_ephe_release_candidate(cand_idx);
      r = NDB_CAND_NEW_TRUSTED;
      log_debug_sid(new->sid, "[EPH] confirmed");
    } else {
      /* New ephemeris doesn't match new candidate - check for validity */
      r = NDB_CAND_MISMATCH;
      ndb_ephe_release_candidate(cand_idx);
      ndb_ephe_try_adding_candidate(new);
      log_debug_sid(new->sid, "[EPH] mismatch");
    }
  } else if (ndb_can_confirm_ephemeris(new, pe, pa, NULL)) {
    /* first candidate, but can be verified from an older ephemeris
     * or almanac */
    log_debug_sid(new->sid, "[EPH] new trusted");
    r = NDB_CAND_NEW_TRUSTED;
  } else {
    /* New one is not in candidate list yet, try to put it
     * to an empty slot */
    ndb_ephe_try_adding_candidate(new);
    r = NDB_CAND_NEW_CANDIDATE;
    log_debug_sid(new->sid, "[EPH] untrusted");
  }

  chMtxUnlock(&cand_list_access);
  return r;
}

/**
 * Load ephemeris from NDB
 *
 * The method looks for an appropriate ephemeris in persistent database, if the
 * entry is found, it is returned with #NDB_ERR_NONE. If there is no entry,
 * an attempt to look in a candidate table is made, and, if candidate entry is
 * found, it is returned with #NDB_ERR_UNCONFIRMED_DATA.
 *
 * \param[in]  sid Signal identifier
 * \param[out] e   Destination for ephemeris data
 *
 * \return Operation result
 * \retval NDB_ERR_NONE             On success
 * \retval NDB_ERR_BAD_PARAM        On parameter error
 * \retval NDB_ERR_MISSING_IE       No cached data block
 * \retval NDB_ERR_UNCONFIRMED_DATA Unconfirmed data block
 */
ndb_op_code_t ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e)
{
  u16 idx = map_sid_to_index(sid);

  if (ARRAY_SIZE(ndb_ephemeris) <= idx || NULL == e) {
    return NDB_ERR_BAD_PARAM;
  }

  ndb_op_code_t res = ndb_retrieve(&ndb_ephemeris_md[idx], e, sizeof(*e),
                                   NULL, NULL);
  if (NDB_ERR_NONE != res) {
    /* If there is a data loading error, check for unconfirmed candidate */
    chMtxLock(&cand_list_access);
    s16 cand_idx = ndb_ephe_find_candidate(sid);
    if (cand_idx >= 0) {
      /* Return unconfirmed candidate data with an appropriate error code */
      *e = ephe_candidates[cand_idx].ephe;
      res = NDB_ERR_UNCONFIRMED_DATA;
    }
    chMtxUnlock(&cand_list_access);
  }

  /* Patch SID to be accurate for GPS L1/L2 */
  e->sid = sid;
  return res;
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
      return NDB_ERR_UNCONFIRMED_DATA;
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
 * \retval NDB_ERR_NONE             On success. Ephemeris is persisted.
 * \retval NDB_ERR_NO_CHANGE        On success. The entry is already persisted.
 * \retval NDB_ERR_BAD_PARAM        Parameter errors.
 * \retval NDB_ERR_UNCONFIRMED_DATA New entry, but confirmation is required.
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

  if (ARRAY_SIZE(ndb_ephemeris_md) <= idx) {
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
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  assert(valid != NULL);
  assert(health_bits != NULL);
  assert(toe != NULL);
  assert(fit_interval != NULL);
  assert(ura != NULL);
  u16 idx = sid_to_global_index(sid);
  ndb_lock();
  if (0 != (ndb_ephemeris_md[idx].nv_data.state & NDB_IE_VALID)) {
    *valid = ndb_ephemeris[idx].valid;
    *health_bits = ndb_ephemeris[idx].health_bits;
    *toe = ndb_ephemeris[idx].toe;
    *fit_interval = ndb_ephemeris[idx].fit_interval;
    *ura = ndb_ephemeris[idx].ura;
    res = NDB_ERR_NONE;
  } else {
    *valid = false;
    res = NDB_ERR_MISSING_IE;
  }
  ndb_unlock();
  return res;
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
