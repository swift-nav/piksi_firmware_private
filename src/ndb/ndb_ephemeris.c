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

#include <assert.h>
#include <string.h>
#include <swiftnav/constants.h>
#include <swiftnav/linear_algebra.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>

#include "hal/piksi_systime.h"
#include "ndb.h"
#include "ndb_fs_access.h"
#include "ndb_internal.h"
#include "sbp/sbp.h"
#include "sbp/sbp_utils.h"
#include "settings/settings_client.h"
#include "signal_db/signal_db.h"
#include "timing/timing.h"

#define NDB_EPHE_FILE_NAME "persistent/ndb/ephemeris"
#define NDB_EPHE_FILE_TYPE "ephemeris"
static ephemeris_t ndb_ephemeris[NUM_SATS];
static ndb_element_metadata_t ndb_ephemeris_md[ARRAY_SIZE(ndb_ephemeris)];
static ndb_file_t ndb_ephe_file = {
    .name = NDB_EPHE_FILE_NAME,
    .type = NDB_EPHE_FILE_TYPE,
    .block_data = (u8 *)&ndb_ephemeris[0],
    .block_md = &ndb_ephemeris_md[0],
    .block_size = sizeof(ndb_ephemeris[0]),
    .block_count = ARRAY_SIZE(ndb_ephemeris),
};

typedef struct {
  ephemeris_t ephe;
  bool used;
  piksi_systime_t received_at;
} ephemeris_candidate_t;

#define EPHE_CAND_LIST_LEN (NUM_SATS)
#define MAX_EPHE_CANDIDATE_AGE 92 /* seconds */
static ephemeris_candidate_t ephe_candidates[EPHE_CAND_LIST_LEN];
static MUTEX_DECL(cand_list_access);

/* identically received ephemeris are resent every N seconds */
#define SBP_EPHEMERIS_RESEND_PERIOD_S 30
static MUTEX_DECL(ephemeris_send_counter_lock);

typedef struct {
  bool erase_ephemeris;   /**< Erase ephemeris data on boot */
  s16 valid_alm_accuracy; /**< Cross-checking accuracy with valid almanac [m] */
  s16 valid_eph_accuracy; /**< Cross-checking accuracy with valid ephemeris [m]
                           */
  s16 alm_fit_interval;   /**< Almanac fit interval (days) */
} ndb_ephe_config_t;

static ndb_ephe_config_t ndb_ephe_config = {
    .erase_ephemeris = true,
    .valid_alm_accuracy = 5000,
    .valid_eph_accuracy = 100,
    .alm_fit_interval = 6,
};

static piksi_systime_t ephemeris_send_time[NUM_SATS];

/** Flag if almanacs can be used in ephemeris candidate validation */
static bool almanacs_enabled = false;

void ndb_ephemeris_init(void) {
#if defined NDB_USE_NV_EPHEMERIS && NDB_USE_NV_EPHEMERIS > 0
  SETTING("ndb",
          "erase_ephemeris",
          ndb_ephe_config.erase_ephemeris,
          SETTINGS_TYPE_BOOL);
#endif
  SETTING("ndb",
          "valid_alm_acc",
          ndb_ephe_config.valid_alm_accuracy,
          SETTINGS_TYPE_INT);
  SETTING("ndb",
          "valid_eph_acc",
          ndb_ephe_config.valid_eph_accuracy,
          SETTINGS_TYPE_INT);
  SETTING("ndb",
          "valid_alm_days",
          ndb_ephe_config.alm_fit_interval,
          SETTINGS_TYPE_INT);

  ndb_load_data(&ndb_ephe_file,
                ndb_ephe_config.erase_ephemeris || !NDB_USE_NV_EPHEMERIS);
}

static bool sid_sibling(const gnss_signal_t sid1, const gnss_signal_t sid2) {
  if (sid_to_constellation(sid1) == sid_to_constellation(sid2)) {
    return (sid1.sat == sid2.sat);
  }
  return false;
}

static s16 ndb_ephe_find_candidate(gnss_signal_t sid) {
  for (u16 i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    if (ephe_candidates[i].used &&
        sid_sibling(ephe_candidates[i].ephe.sid, sid)) {
      return i;
    }
  }
  return -1;
}

/* Find an empty slot (unused or outdated ephemeris) in the candidate list
 * and add the given candidate. Log a warning if no empty slot found.
 */
static void ndb_ephe_try_adding_candidate(const ephemeris_t *new) {
  assert(new);
  int i;
  u32 candidate_age;
  piksi_systime_t now;
  piksi_systime_get(&now);
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    bool empty = true;
    if (ephe_candidates[i].used) {
      candidate_age =
          piksi_systime_sub_s(&now, &ephe_candidates[i].received_at);
      empty = candidate_age > MAX_EPHE_CANDIDATE_AGE;
    }

    if (empty) {
      MEMCPY_S(&ephe_candidates[i].ephe,
               sizeof(ephemeris_t),
               new,
               sizeof(ephemeris_t));
      piksi_systime_get(&ephe_candidates[i].received_at);
      ephe_candidates[i].used = true;
      return;
    }
  }
  log_warn_sid(new->sid,
               "Could not add ephemeris candidate (%d slots full)",
               EPHE_CAND_LIST_LEN);
}

static void ndb_ephe_release_candidate(s16 cand_index) {
  if ((cand_index < 0) || (cand_index >= EPHE_CAND_LIST_LEN)) {
    return;
  }
  ephe_candidates[cand_index].used = false;
}

bool ephemeris_equal_except_fit_interval(const ephemeris_t *a,
                                         const ephemeris_t *b) {
  assert(a);
  assert(b);

  ephemeris_t cpy;
  MEMCPY_S(&cpy, sizeof(cpy), a, sizeof(*a));

  if (IS_GLO(a->sid)) {
    /* GLO fit interval might change during ephemeris validity time.
       So exclude it from the comparison. */
    cpy.fit_interval = b->fit_interval;
  }
  return ephemeris_equal(&cpy, b);
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
                                      const almanac_t *existing_a,
                                      const ephemeris_t *candidate) {
  assert(new);
  if (NULL != candidate) {
    if (ephemeris_equal_except_fit_interval(candidate, new)) {
      /* Exact match */
      log_debug_sid(new->sid, "[EPH] candidate match");
      return true;
    }
  } else if (NULL != existing_e) {
    if (ephemeris_equal_except_fit_interval(existing_e, new)) {
      /* Exact match with stored */
      log_debug_sid(new->sid, "[EPH] NDB match");
      return true;
    }
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

  if (NULL != existing_a && almanac_valid(existing_a, &t_start) &&
      almanac_valid(existing_a, &t_end) && ephemeris_valid(new, &t_start) &&
      ephemeris_valid(new, &t_end)) {
    /* Almanac position verification */

    bool ok = true;
    gps_time_t t = t_start;

    for (u8 i = 0; i < 3 && ok;
         ++i, t.tow += MINUTE_SECS * 30, normalize_gps_time(&t)) {
      double _[3];
      double alm_sat_pos[3];
      double eph_sat_pos[3];
      u8 iode;
      u16 iodc;

      ok = false;

      if (0 ==
              calc_sat_state_almanac(existing_a, &t, alm_sat_pos, _, _, _, _) &&
          0 == calc_sat_state_n(
                   new, &t, eph_sat_pos, _, _, _, _, &iodc, &iode)) {
        /* Compute distance [m] */
        double d = vector_distance(3, alm_sat_pos, eph_sat_pos);

        ok = (d <= ndb_ephe_config.valid_alm_accuracy);
        log_debug_sid(new->sid,
                      "[EPH] almanac position error %lf T=%" PRId16 ",%" PRId32,
                      d,
                      t.wn,
                      (s32)t.tow);
      }
    }

    res = ok;
    if (res) {
      log_debug_sid(new->sid, "[EPH] verified against almanac");
    }
  }

  if (!res && NULL != existing_e && ephemeris_valid(existing_e, &t_start) &&
      ephemeris_valid(existing_e, &t_end)) {
    /* Previous ephemeris, but still valid */

    bool ok = true;
    gps_time_t t = t_start;

    for (u8 i = 0; i < 3 && ok;
         ++i, t.tow += MINUTE_SECS * 30, normalize_gps_time(&t)) {
      double _[3];
      double old_sat_pos[3];
      double new_sat_pos[3];
      u8 iode;
      u16 iodc;

      ok = false;

      if (0 == calc_sat_state_n(
                   existing_e, &t, old_sat_pos, _, _, _, _, &iodc, &iode) &&
          0 == calc_sat_state_n(
                   new, &t, new_sat_pos, _, _, _, _, &iodc, &iode)) {
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
 * \retval NDB_CAND_BAD_PARAM     Bad SID
 */
static ndb_cand_status_t ndb_get_ephemeris_status(const ephemeris_t *new) {
  assert(new);
  ndb_cand_status_t r = NDB_CAND_MISMATCH;

  ephemeris_t existing_e; /* Existing ephemeris data */
  ephemeris_t *pe = NULL; /* Existing ephemeris data pointer if valid */
  ephemeris_t *ce = NULL; /* Candidate ephemeris data pointer if valid */
  almanac_t existing_a;   /* Existing almanac data */
  almanac_t *pa = NULL;   /* Existing almanac data pointer if valid */

  u16 idx = sid_to_sv_index(new->sid);
  if (ARRAY_SIZE(ndb_ephemeris) <= idx) {
    return NDB_CAND_BAD_PARAM;
  }

  assert(idx < ARRAY_SIZE(ndb_ephemeris_md));
  if (NDB_ERR_NONE ==
      ndb_retrieve(
          &ndb_ephemeris_md[idx], &existing_e, sizeof(existing_e), NULL)) {
    pe = &existing_e;
  }
  if (NDB_ERR_NONE == ndb_almanac_read(new->sid, &existing_a) &&
      almanacs_enabled) {
    pa = &existing_a;
    existing_a.fit_interval = ndb_ephe_config.alm_fit_interval * DAY_SECS;
  }

  chMtxLock(&cand_list_access);

  s16 cand_idx = ndb_ephe_find_candidate(new->sid);
  if (-1 != cand_idx) {
    ce = &ephe_candidates[cand_idx].ephe;
  }

  /* Check that GLO L1CA and GLO L2CA always send same set of ephemeris. */
  ephemeris_t *ephep = pe;
  if (!ephep) {
    ephep = ce;
  }

  if (ephep) {
    bool ep_eq = ephemeris_equal_except_fit_interval(ephep, new);
    if (!ep_eq && (ephep->toe.wn == new->toe.wn) &&
        (ephep->toe.tow == new->toe.tow)) {
      log_warn_sid(new->sid,
                   "Ephemeris discrepancy detected: "
                   "%" PRIi16 " %" PRIi16 " %lf %lf %p %p",
                   ephep->toe.wn,
                   new->toe.wn,
                   ephep->toe.tow,
                   new->toe.tow,
                   ce,
                   pe);
    }
  }

  if (TIME_UNKNOWN == get_time_quality()) {
    ndb_ephe_release_candidate(cand_idx);
    ndb_ephe_try_adding_candidate(new);
    r = NDB_CAND_GPS_TIME_MISSING;
  } else if (NULL != pe && ephemeris_equal_except_fit_interval(pe, new) &&
             0 == (ndb_ephemeris_md[idx].vflags & NDB_VFLAG_DATA_FROM_NV)) {
    /* If new ephemeris is identical to the one in NDB,
     * and the NDB data is not initially loaded from NV,
     * then no need to do anything */
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
 * \retval NDB_ERR_AGED_DATA        Data in NDB has aged out
 * \retval NDB_ERR_MISSING_GPS_TIME GPS time is unknown
 */
ndb_op_code_t ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e) {
  u16 idx = sid_to_sv_index(sid);

  if (ARRAY_SIZE(ndb_ephemeris) <= idx || NULL == e) {
    return NDB_ERR_BAD_PARAM;
  }

  assert(idx < ARRAY_SIZE(ndb_ephemeris_md));
  ndb_op_code_t res = ndb_retrieve(&ndb_ephemeris_md[idx], e, sizeof(*e), NULL);

  double ndb_eph_age = NDB_NV_WARM_START_LIMIT_SECS;

  if (NDB_ERR_NONE == res) {
    /* If NDB read was successful, check that data has not aged out */
    res = ndb_check_age(&e->toe, ndb_eph_age);
  } else if (NDB_ERR_BAD_PARAM == res) {
    /* Handle the situation when ndb_retrieve returns NDB_ERR_BAD_PARAM.
     * This may happen when we've already read ephemerides during startup from
     * NV RAM, so check that locally stored ephemeris not aged out */
    res = ndb_check_age(&ndb_ephemeris[idx].toe, ndb_eph_age);
  }

  if (NDB_ERR_NONE != res) {
    /* If there is a data loading error, check for unconfirmed candidate */
    chMtxLock(&cand_list_access);
    s16 cand_idx = ndb_ephe_find_candidate(sid);
    if (cand_idx >= 0) {
      *e = ephe_candidates[cand_idx].ephe;
      res = ndb_check_age(&e->toe, ndb_eph_age);
      if (NDB_ERR_AGED_DATA != res) {
        /* Found unconfirmed candidate that is recent enough, return it
         * with an appropriate error code */
        res = NDB_ERR_UNCONFIRMED_DATA;
      }
    }
    chMtxUnlock(&cand_list_access);
  }

  /* Patch SID to be accurate for GPS L1/L2 */
  e->sid = sid;
  return res;
}

static ndb_op_code_t ndb_ephemeris_store_do(const ephemeris_t *e,
                                            ndb_data_source_t src) {
  assert(e);
  if (!e->valid) {
    return NDB_ERR_BAD_PARAM;
  }

  if (NDB_DS_RECEIVER == src) {
    switch (ndb_get_ephemeris_status(e)) {
      case NDB_CAND_IDENTICAL:
        return NDB_ERR_NO_CHANGE;
      case NDB_CAND_OLDER:
        return NDB_ERR_OLDER_DATA;
      case NDB_CAND_NEW_TRUSTED: {
        u16 idx = sid_to_sv_index(e->sid);
        return ndb_update(e, src, &ndb_ephemeris_md[idx]);
      }
      case NDB_CAND_NEW_CANDIDATE:
      case NDB_CAND_MISMATCH:
        return NDB_ERR_UNCONFIRMED_DATA;
      case NDB_CAND_GPS_TIME_MISSING:
        return NDB_ERR_GPS_TIME_MISSING;
      case NDB_CAND_BAD_PARAM:
        return NDB_ERR_BAD_PARAM;
      default:
        assert(!"Invalid status");
    }
  } else if (NDB_DS_SBP == src) {
    u8 valid, health_bits;
    gps_time_t toe = GPS_TIME_UNKNOWN;
    u32 fit_interval;
    float ura;
    u16 idx = sid_to_sv_index(e->sid);
    ndb_ephemeris_info(e->sid, &valid, &health_bits, &toe, &fit_interval, &ura);
    if (!valid || gpsdifftime(&e->toe, &toe) ||
        0 == (ndb_ephemeris_md[idx].vflags & NDB_VFLAG_DATA_FROM_NV)) {
      /* If local ephemeris is not valid or received one is newer or
       * existing data is initially loaded from NDB,
       * then save the received one. */
      log_debug_sid(
          e->sid,
          "Saving ephemeris received over SBP v:%d [%d,%f] vs [%d,%f]",
          (int)valid,
          toe.wn,
          toe.tow,
          e->toe.wn,
          e->toe.tow);
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
                                  u16 sender_id) {
  assert(e);
  ndb_op_code_t res = ndb_ephemeris_store_do(e, src);

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_EPHEMERIS,
                     res,
                     src,
                     &e->sid,
                     NULL,
                     sender_id);

  bool send_sbp = false;
  piksi_systime_t now;
  piksi_systime_get(&now);
  u16 idx = sid_to_sv_index(e->sid);

  /* ephemeris is sent over SBP always on the first reception, and after that
   * periodically */

  chMtxLock(&ephemeris_send_counter_lock);
  s64 timediff = piksi_systime_sub_s(&now, &ephemeris_send_time[idx]);
  if (NDB_ERR_NO_CHANGE != res || (timediff >= SBP_EPHEMERIS_RESEND_PERIOD_S)) {
    send_sbp = true;
    ephemeris_send_time[idx] = now;
  }
  chMtxUnlock(&ephemeris_send_counter_lock);

  if (send_sbp) {
    msg_ephemeris_t msg;
    msg_info_t info = pack_ephemeris(e, &msg);
    sbp_send_msg(info.msg_id, info.size, (u8 *)&msg);
  }

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
ndb_op_code_t ndb_ephemeris_erase(gnss_signal_t sid) {
  u16 idx = sid_to_sv_index(sid);

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

ndb_op_code_t ndb_ephemeris_info(gnss_signal_t sid,
                                 u8 *valid,
                                 u8 *health_bits,
                                 gps_time_t *toe,
                                 u32 *fit_interval,
                                 float *ura) {
  assert(valid);
  assert(health_bits);
  assert(toe);
  assert(fit_interval);
  assert(ura);

  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;
  u16 idx = sid_to_sv_index(sid);
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
