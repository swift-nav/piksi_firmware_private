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

/** Almanac file name */
#define NDB_ALMA_FILE_NAME   "persistent/almanac"
/** Almanac file type */
#define NDB_ALMA_FILE_TYPE   "almanac"

/** Almanac week number file name */
#define NDB_ALMA_WN_FILE_NAME   "persistent/almanac_wn"
/** Almanac week number file type */
#define NDB_ALMA_WN_FILE_TYPE   "almanac WN"

/** Maximum number of almanac candidates to maintain.
 * In the worst scenario, each of satellites broadcasts own version of almanacs,
 * which would require at least 31 * NUM_SATS_GPS entries in the table to
 * operate. However normal scenario assumes that there are at least 2 satellites
 * that broadcast the same data. */
#define NDB_ALMA_CAND_LIST_LEN (NUM_SATS_GPS * 2)
/** Maximum number of almanac week number candidates to maintain. */
#define NDB_ALMA_WN_CAND_LIST_LEN (NUM_SATS_GPS)
/** Maximum almanac candidate age before expiration [s] */
#define NDB_MAX_ALMA_CANDIDATE_AGE (MINUTE_SECS * 14)
/** Maximum almanac week number candidate age before expiration [s] */
#define NDB_MAX_ALMA_WN_CANDIDATE_AGE (MINUTE_SECS * 14)
/** Total number of almanacs entries to store */
#define NDB_ALMA_IE_COUNT (NUM_SATS_GPS)
/** Total number of almanac week numbers to store */
#define NDB_ALMA_IE_WN_COUNT (NUM_SATS_GPS * 2)
/** Interval between two almanac transmission [cycles] */
#define NDB_ALMA_MESSAGE_SPACING        (200 / NV_WRITE_REQ_TIMEOUT)
/** Minimum interval between almanac transmit epoch starts, can be longer
    if the amount of sent messages makes epoch longer [cycles] */
#define NDB_ALMA_TRANSMIT_EPOCH_SPACING (60000 / NV_WRITE_REQ_TIMEOUT)

/** Almanac candidate entry */
typedef struct {
  almanac_t       alma;        /**< Almanac data */
  bool            used;        /**< Entry flag */
  ndb_timestamp_t received_at; /**< Time of entry creation */
} ndb_alma_candidate_t;

/** GPS almanac time data type */
typedef struct {
  u32             gps_toa;    /**< GPS almanac TOA field [s] */
  u16             gps_wn;     /**< GPA almanac WN field [week] */
} ndb_alma_wn_t;

/** Almanac time/week number candidate entry */
typedef struct {
  ndb_alma_wn_t   alma_wn;     /**< Almanac's week number */
  bool            used;        /**< Entry flag */
  ndb_timestamp_t received_at; /**< Time of entry creation */
} ndb_alma_wn_candidate_t;

/** Persistent almanac data */
static almanac_t ndb_almanac[NDB_ALMA_IE_COUNT];
/** Persistent and non-persistent almanac metadata */
static ndb_element_metadata_t ndb_almanac_md[ARRAY_SIZE(ndb_almanac)];

/** Persistent almanac week number data */
static ndb_alma_wn_t ndb_almanac_wn[NDB_ALMA_IE_WN_COUNT];
/** Persistent and non-persistent almanac week number metadata */
static ndb_element_metadata_t ndb_almanac_wn_md[ARRAY_SIZE(ndb_almanac_wn)];

/** Non-persistent almanac candidates */
static ndb_alma_candidate_t alma_candidates[NDB_ALMA_CAND_LIST_LEN];
/** Non-persistent almanac week number candidates */
static ndb_alma_wn_candidate_t alma_wn_candidates[NDB_ALMA_WN_CAND_LIST_LEN];
/** Mutex for synchronizing access to candidate lists */
static MUTEX_DECL(cand_list_access);

/** NDB almanac file object */
static ndb_file_t ndb_alma_file = {
    .name = NDB_ALMA_FILE_NAME,
    .type = NDB_ALMA_FILE_TYPE,
    .block_data = (u8*)&ndb_almanac[0],
    .block_md = &ndb_almanac_md[0],
    .block_size = sizeof(ndb_almanac[0]),
    .block_count = ARRAY_SIZE(ndb_almanac),
};

/** NDB almanac's week numbers file object */
static ndb_file_t ndb_alma_wn_file = {
    .name = NDB_ALMA_WN_FILE_NAME,
    .type = NDB_ALMA_WN_FILE_TYPE,
    .block_data = (u8*)&ndb_almanac_wn[0],
    .block_md = &ndb_almanac_wn_md[0],
    .block_size = sizeof(ndb_almanac_wn[0]),
    .block_count = ARRAY_SIZE(ndb_almanac_wn),
};

static u16 map_sid_to_index(gnss_signal_t sid)
{
  u16 idx = PLATFORM_SIGNAL_COUNT;
  /*
   * Current architecture uses GPS L1 C/A almanac for GPS satellites.
   */
  if (sid_to_constellation(sid) == CONSTELLATION_GPS) {
    idx = sid_to_global_index(construct_sid(CODE_GPS_L1CA, sid.sat));
  } else {
    idx = sid_to_global_index(sid);
  }
  return idx;
}

/**
 * Looks for almanac candidate with given signal ID.
 *
 * \param sid      Signal identifier for which almanac is searched.
 * \param prev_idx Previous index to continue search or `-1` for starting the
 *                 search from the beginning.
 *
 * \return Next candidate index.
 * \retval -1 If the candidate is not found
 *
 * \internal
 */
static s16 ndb_alma_candidate_find(gnss_signal_t sid, s16 prev_idx)
{
  assert(prev_idx >= -1);

  for (u16 i = (u16)(prev_idx + 1); i < ARRAY_SIZE(alma_candidates); i++) {
    if (alma_candidates[i].used &&
        sid_is_equal(alma_candidates[i].alma.sid, sid)) {
      return i;
    }
  }
  return -1;
}

/**
 * Adds new almanac candidate
 *
 * The almanac candidate replaces an existing candidate with the same TOA field.
 * If the matching old candidate is not present, an empty slot is used. And if
 * there is no empty slot, an oldest candidate entry is replaced.
 *
 * \param[in] alma New almanac data.
 *
 * \return None
 *
 * \internal
 */
static void ndb_alma_candidate_add(const almanac_t *alma)
{
  ndb_timestamp_t now = ndb_get_NAP_timestamp();
  ndb_timestamp_t max_age = 0;
  ndb_ie_index_t  max_age_idx = ARRAY_SIZE(alma_candidates);
  ndb_ie_index_t  idx;
  ndb_ie_index_t  empty_idx = ARRAY_SIZE(alma_candidates);
  for (idx = 0; idx < ARRAY_SIZE(alma_candidates); idx++) {
    if (alma_candidates[idx].used) {
      if (alma_candidates[idx].alma.toa.tow == alma->toa.tow) {
        /* Replace existing entry with the same TOA */
        break;
      }

      u32 candidate_age = now - alma_candidates[idx].received_at;
      if (candidate_age > max_age) {
        max_age = candidate_age;
        max_age_idx = idx;
      }
    } else if (ARRAY_SIZE(alma_candidates) == empty_idx){
      empty_idx = idx;
    }
  }

  if (ARRAY_SIZE(alma_candidates) == idx) {
    if (ARRAY_SIZE(alma_candidates) == empty_idx) {
      /* Replacing oldest entry */
      idx = max_age_idx;
    } else {
      /* Replacing empty entry */
      idx = empty_idx;
    }
  } else {
    /* Replacing non-empty entry with the same TOA value */
  }

  assert(idx < ARRAY_SIZE(alma_candidates));

  alma_candidates[idx].alma = *alma;
  alma_candidates[idx].received_at = now;
  alma_candidates[idx].used = true;
}

/**
 * Releases almanac candidate entry
 *
 * \param[in] cand_index Candidate entry index
 *
 * \internal
 */
static void ndb_alma_candidate_release(s16 cand_index)
{
  assert(cand_index >= 0 && (u16)cand_index < ARRAY_SIZE(alma_candidates));
  memset(&alma_candidates[cand_index], 0, sizeof(alma_candidates[cand_index]));
}

/**
 * Drops expired entries from the almanac candidate table
 *
 * \return None
 */
static void ndb_alma_candidate_cleanup(void)
{
  ndb_timestamp_t time = ndb_get_NAP_timestamp();
  if (time < NDB_MAX_ALMA_CANDIDATE_AGE) {
    time = 0;
  } else {
    time -= NDB_MAX_ALMA_CANDIDATE_AGE;
  }
  for (u16 i = 0; i < ARRAY_SIZE(alma_candidates); i++) {
    if (alma_candidates[i].used && alma_candidates[i].received_at < time) {
      ndb_alma_candidate_release(i);
    }
  }
}

/**
 * Checks the new almanac data status and updates candidate table
 *
 * The new almanac's data is checked against NDB persistent storage and
 * almanac candidate list.
 *
 * If the entry matches the contents of persistent storage, nothing is done and
 * #NDB_CAND_IDENTICAL is returned.
 *
 * If the candidate has older time than the the one stored in persistence
 * storage, the candidate is discarded and #NDB_CAND_OLDER is returned.
 *
 * If the entry matches one of the existing candidates, the candidate entry
 * is deleted and #NDB_CAND_NEW_TRUSTED is returned.
 *
 * If the entry doesn't match any of the candidates, it is added to the
 * candidate list by occupying available slot (#NDB_CAND_NEW_CANDIDATE), or
 * replacing one of the existing entries (NDB_CAND_MISMATCH).
 *
 * \param[in] alma New almanac's data
 *
 * \retval NDB_CAND_IDENTICAL     Almanac is already stored in persistent file.
 * \retval NDB_CAND_OLDER         Almanac is older that the one we already store
 *                                and shall be discarded.
 * \retval NDB_CAND_NEW_TRUSTED   Almanac is a trusted candidate and shall be
 *                                stored in persistent file.
 * \retval NDB_CAND_NEW_CANDIDATE Almanac is a new version and shall be
 *                                confirmed before use.
 * \retval NDB_CAND_MISMATCH      Almanac is a new version and replaces existing
 *                                candidate with the same keys.
 *
 * \internal
 */
static ndb_cand_status_t ndb_alma_candidate_update(const almanac_t *alma)
{
  ndb_cand_status_t r = NDB_CAND_MISMATCH;
  almanac_t existing;

  if (NDB_ERR_NONE == ndb_almanac_read(alma->sid, &existing)) {
    bool existing_is_newer = false;
    bool existing_is_same = false;

    if (memcmp(&existing, alma, sizeof(*alma)) != 0) {
      /* We already have different almanac in DB, it might be newer than given */
      if (WN_UNKNOWN != existing.toa.wn && WN_UNKNOWN != alma->toa.wn) {
        u32 tai_existing = (u32)existing.toa.wn * WEEK_SECS + (s32)existing.toa.tow;
        u32 tai_alma = (u32)alma->toa.wn * WEEK_SECS + (s32)alma->toa.tow;

        /* Compute, which almanac is newer */
        existing_is_newer = tai_existing > tai_alma;
      } else if (WN_UNKNOWN == existing.toa.wn && WN_UNKNOWN == alma->toa.wn) {
        /* Both candidate and persisted ones don't have WN. Assume the same
         * or possible week number wrap. Make a wild guess the time difference
         * is less than half a week */
        s32 delta_time = (s32)existing.toa.tow - (s32)alma->toa.tow;
        if (delta_time > 0) {
          /* The difference is OK, if the delta is less, than half a week */
          existing_is_newer = delta_time < WEEK_SECS / 2;
        } else {
          /* The difference is OK, if the delta is less, than half a week */
          existing_is_newer = delta_time < -WEEK_SECS / 2;
        }
      }
    } else {
      /* New one is identical to the one in DB */
      existing_is_same = true;
    }
    if (existing_is_same || existing_is_newer) {
      /* New one is identical to the one in DB or older than the one in DB.
       * A candidate with a wrong data may exist, so delete it. */
      chMtxLock(&cand_list_access);
      /* Do cleanup */
      ndb_alma_candidate_cleanup();

      /* Unlike ephemeris, there could be more than one almanac candidate per SV */
      for (s16 cand_idx = ndb_alma_candidate_find(alma->sid, -1);
           cand_idx >= 0;
           cand_idx = ndb_alma_candidate_find(alma->sid, cand_idx)) {
        /* Almanac for this SV is candidate list */
        if (alma_candidates[cand_idx].alma.toa.tow == alma->toa.tow ||
            alma_candidates[cand_idx].alma.toa.tow == existing.toa.tow) {
          /* Erase only candidate with matching TOA. */
          ndb_alma_candidate_release(cand_idx);
          break;
        }
      }
      chMtxUnlock(&cand_list_access);

      return existing_is_same ? NDB_CAND_IDENTICAL : NDB_CAND_OLDER;
    }
  }

  chMtxLock(&cand_list_access);
  /* Do cleanup */
  ndb_alma_candidate_cleanup();

  s16 cand_idx = -1;

  /* Unlike ephemeris, there could be more than one almanac candidate per SV */
  for (cand_idx = ndb_alma_candidate_find(alma->sid, -1);
       cand_idx >= 0;
       cand_idx = ndb_alma_candidate_find(alma->sid, cand_idx)) {
    /* Almanac for this SV is candidate list */
    if (memcmp(&alma_candidates[cand_idx].alma, alma, sizeof(*alma)) == 0) {
      break;
    }
  }

  if (-1 != cand_idx) {
    /* New one is identical to the one candidate list, save it to DB */
    ndb_alma_candidate_release(cand_idx);
    r = NDB_CAND_NEW_TRUSTED;
  } else {
    /* New one is not in candidate list yet, try to put it to an empty slot or
     * replace an existing entry */
    ndb_alma_candidate_add(alma);
    r = NDB_CAND_NEW_CANDIDATE;
  }

  chMtxUnlock(&cand_list_access);

  return r;
}

/* Forward declaration to ensure the callback matches NDB prototype */
static ndb_entry_match_fn ndb_alma_wn_match;

/**
 * Internal helper to find almanac week number entry by ToW
 *
 * This method is a callback used with #ndb_find_retrieve and shall conform
 * to #ndb_entry_match_fn interface type.
 *
 * \param[in] data   Pointer to NDB week number entry
 * \param[in] md     Pointer to NDB week number entry metadata
 * \param[in] cookie Pointer to ToW [s], must be u32.
 *
 * \retval true  Entry has the same ToW
 * \retval false Entry has different ToW
 *
 * \sa ndb_entry_match_fn
 * \sa ndb_find_retrieve
 *
 * \internal
 */
static bool ndb_alma_wn_match(const void *data,
                              const ndb_element_metadata_t *md,
                              void *cookie)
{
  (void)md; /* Unused */

  u32 toa = *(const u32 *)cookie;
  const ndb_alma_wn_t *alma_wn = data;

  return alma_wn->gps_toa == toa;
}

/**
 * Helper to update existing almanac candidates with known week number.
 *
 * This method is called whenever a new pair of TOA/WN is accepted by NDB. It
 * goes through a list of existing candidate almanac entries and performs
 * in-place update of WN fields.
 *
 * \param[in] toa Almanac's time
 * \param[in] wn  Almanac's week number
 *
 * \sa ndb_alma_wn_update_alma_file
 *
 * \internal
 */
static void ndb_alma_wn_update_alma_candidates(u32 toa, u16 wn)
{
  for (ndb_ie_index_t idx = 0; idx < ARRAY_SIZE(alma_candidates); ++idx) {
    if (alma_candidates[idx].used &&
        WN_UNKNOWN == alma_candidates[idx].alma.toa.wn &&
        toa == alma_candidates[idx].alma.toa.tow) {
      alma_candidates[idx].alma.toa.wn = wn;
    }
  }
}

/**
 * Drops expired entries from the almanac TOA/WN candidate table
 *
 * \return None
 *
 * \internal
 */
static void ndb_alma_wn_candidate_cleanup(void)
{
  ndb_timestamp_t time = ndb_get_NAP_timestamp();
  if (time < NDB_MAX_ALMA_WN_CANDIDATE_AGE) {
    time = 0;
  } else {
    time -= NDB_MAX_ALMA_WN_CANDIDATE_AGE;
  }
  for (u16 i = 0; i < ARRAY_SIZE(alma_wn_candidates); i++) {
    if (alma_wn_candidates[i].used && alma_wn_candidates[i].received_at < time) {
      memset(&alma_wn_candidates[i], 0, sizeof(alma_wn_candidates[i]));
    }
  }
}

/**
 * Checks what is the candidate status of the new almanac's TOA/WN pair.
 *
 * The method performs the following actions:
 * - If the same data exists in NDB file, no further actions are required
 * - If the same data exists in candidate table, it is removed and should
 *   be stored into NDB file as reliable data.
 * - If the data with the same key (TOA) exists with in the candidate table,
 *   but contents differs (WN), then the new value replaces old one.
 * - New values are added to candidate table either by occupying empty entry,
 *   or by replacing oldest one.
 *
 * \param[in] toa Almanac's time
 * \param[in] wn  Almanac's week number
 *
 * \retval NDB_CAND_IDENTICAL     Data is present in NDB, no further actions
 *                                required.
 * \retval NDB_CAND_NEW_CANDIDATE Data is added as a new candidate.
 * \retval NDB_CAND_NEW_TRUSTED   The candidate data is confirmed and removed
 *                                from a candidate list. This data shall be
 *                                persisted
 * \retval NDB_CAND_MISMATCH      The candidate data matches by key existing
 *                                entry, so new data replaces an old candidate
 *                                entry.
 *
 * \internal
 */
static ndb_cand_status_t ndb_alma_wn_candidate_update(u32 toa, u16 wn)
{
  ndb_cand_status_t res = NDB_CAND_MISMATCH;

  u16 wn_existing = 0;
  if (NDB_ERR_NONE == ndb_almanac_wn_read(toa, &wn_existing)) {
    if (wn_existing == wn) {
      /* Same pair of WN and ToW is already present in persistent storage;
       * A candidate with a wrong data may exist, so delete it. */
      chMtxLock(&cand_list_access);
      /* Do cleanup */
      ndb_alma_wn_candidate_cleanup();

      for (ndb_ie_index_t idx = 0; idx < ARRAY_SIZE(alma_wn_candidates); ++idx) {
        if (alma_wn_candidates[idx].used &&
            alma_wn_candidates[idx].alma_wn.gps_toa == toa) {
          memset(&alma_wn_candidates[idx], 0, sizeof(alma_wn_candidates[idx]));
          break;
        }
      }
      chMtxUnlock(&cand_list_access);

      return NDB_CAND_IDENTICAL;
    }
    /* We have already a week number, but different for the same ToW */
  }

  ndb_ie_index_t idx         = 0;
  ndb_ie_index_t empty_idx   = ARRAY_SIZE(alma_wn_candidates);
  ndb_ie_index_t oldest_idx  = ARRAY_SIZE(alma_wn_candidates);
  u32            oldest_tai  = 0;
  ndb_ie_index_t oldest_idx2 = ARRAY_SIZE(alma_wn_candidates);

  chMtxLock(&cand_list_access);
  /* Do cleanup */
  ndb_alma_wn_candidate_cleanup();

  res = NDB_CAND_NEW_CANDIDATE;
  /* Locate matching index, free index and old index */
  for (idx = 0; idx < ARRAY_SIZE(alma_wn_candidates); ++idx) {
    if (alma_wn_candidates[idx].used) {
      if (alma_wn_candidates[idx].alma_wn.gps_toa == toa) {
        /* Entry with matching ToW is found */
        if (alma_wn_candidates[idx].alma_wn.gps_wn == wn) {
          /* Full candidate match: both ToW and WN are OK. */
          memset(&alma_wn_candidates[idx], 0, sizeof(alma_wn_candidates[idx]));
          res = NDB_CAND_NEW_TRUSTED;
        } else {
          /* WN mismatch */
          res = NDB_CAND_MISMATCH;
        }
        /* In both cases - break the loop */
        break;
      }
      /* ToW doesn't match */
      u32 new_tai = (u32)alma_wn_candidates[idx].alma_wn.gps_wn * WEEK_SECS +
                    alma_wn_candidates[idx].alma_wn.gps_toa;
      if (ARRAY_SIZE(alma_wn_candidates) == oldest_idx ||
          oldest_tai > new_tai) {
        /* Either first non-matching TAI, or have older TAI */
        oldest_idx = idx;
        oldest_tai = new_tai;
      }
      if (ARRAY_SIZE(alma_wn_candidates) == oldest_idx2 ||
          alma_wn_candidates[oldest_idx2].received_at <
          alma_wn_candidates[idx].received_at) {
        /* Entry with the oldest update timestamp */
        oldest_idx2 = idx;
      }
    } else if (ARRAY_SIZE(alma_wn_candidates) == empty_idx) {
      /* Found first empty index */
      empty_idx = idx;
    }
  }
  if (NDB_CAND_NEW_TRUSTED != res) {
    if (ARRAY_SIZE(alma_wn_candidates) == idx) {
      /* Matching entry is not found */
      if (ARRAY_SIZE(alma_wn_candidates) > empty_idx) {
        /* Free entry is found */
        idx = empty_idx;
      } else if (ARRAY_SIZE(alma_wn_candidates) > oldest_idx) {
        idx = oldest_idx;
      } else {
        /* Drop entry with the oldest NDB update time, whatever it is */
        idx = oldest_idx2;
      }
    }
    assert(idx < ARRAY_SIZE(alma_wn_candidates));

    alma_wn_candidates[idx].received_at = ndb_get_NAP_timestamp();
    alma_wn_candidates[idx].used = true;
    alma_wn_candidates[idx].alma_wn.gps_wn = wn;
    alma_wn_candidates[idx].alma_wn.gps_toa = toa;
    res = NDB_CAND_MISMATCH;
  } else {
    /* Update all candidate almanacs with matching TOA, while still keeping the
     * candidate lock */
    ndb_alma_wn_update_alma_candidates(toa, wn);
  }

  chMtxUnlock(&cand_list_access);

  return res;
}

/**
 * Creates a persistent entry for almanac's TOA/WN pair in NDB
 *
 * The method tries to locate an empty entry in the file and update it. If
 * entry entry is not available, an entry with oldest TAI time is discarded.
 *
 * \param[in] toa Almanac's time
 * \param[in] wn  Almanac's week number
 * \param[in] ds  NDB data source
 *
 * \return None
 *
 * \internal
 */
static void ndb_alma_wn_update_wn_file(u32 toa, u16 wn, ndb_data_source_t ds)
{
  ndb_ie_index_t idx = 0;
  ndb_ie_index_t empty_idx  = ndb_alma_wn_file.block_count;
  ndb_ie_index_t oldest_idx = ndb_alma_wn_file.block_count;
  u32            oldest_tai = 0;

  ndb_alma_wn_t * const wn_data = (ndb_alma_wn_t *)ndb_alma_wn_file.block_data;
  ndb_element_metadata_t * const wn_md = ndb_alma_wn_file.block_md;

  /* Step 1 and 2: find entry with matching ToW or an empty one */
  for (idx = 0; idx < ndb_alma_wn_file.block_count; ++idx) {
    if (0 != (wn_md[idx].nv_data.state & NDB_IE_VALID)) {
      if (wn_data[idx].gps_toa == toa) {
        /* Found a matching entry with the same ToW */
        break;
      }
      /* Non-matching entry, compute TAI */
      u32 tai_new = (u32)wn_data[idx].gps_wn * WEEK_SECS + wn_data[idx].gps_toa;
      if (oldest_idx == ndb_alma_wn_file.block_count ||
          tai_new < oldest_tai) {
        /* Block is the first used, or TAI is older*/
        oldest_idx = idx;
        oldest_tai = tai_new;
      }
    } else if (empty_idx == ndb_alma_wn_file.block_count) {
      /* First empty entry */
      empty_idx = idx;
    }
  }
  if (idx == ndb_alma_wn_file.block_count) {
    /* Matching entry is not found, try empty */
    idx = empty_idx;
    if (idx == ndb_alma_wn_file.block_count) {
      /* Matching and empty entry are not found, replace oldest */
      idx = oldest_idx;
    }
  }

  assert(idx < ndb_alma_wn_file.block_count);

  /* Update persistent WN entry */
  wn_data[idx].gps_toa = toa;
  wn_data[idx].gps_wn = wn;
  wn_md[idx].nv_data.source = ds;
  wn_md[idx].nv_data.received_at_NAP = ndb_get_NAP_timestamp();
  wn_md[idx].nv_data.received_at_TAI = ndb_get_TAI_timestamp();
  wn_md[idx].nv_data.state |= NDB_IE_VALID;
  wn_md[idx].vflags |= NDB_VFLAG_MD_DIRTY | NDB_VFLAG_IE_DIRTY;

  ndb_wq_put(&wn_md[idx]);
}

/**
 * Helper to update existing almanac entries with known week number.
 *
 * This method is called whenever a new pair of TOA/WN is accepted by NDB. It
 * goes through a list of existing persisting NDB almanac entries and performs
 * in-place update of WN fields. Updated entries are scheduled for writing.
 *
 * \param[in] toa Almanac's time
 * \param[in] wn  Almanac's week number
 *
 * \sa ndb_alma_wn_update_alma_candidates
 * \internal
 */
static void ndb_alma_wn_update_alma_file(u32 toa, u16 wn)
{
  /* Update all almanac entries with the same ToW and empty WN */
  almanac_t * const data = (almanac_t *)ndb_alma_file.block_data;
  ndb_element_metadata_t * const md = ndb_alma_file.block_md;

  for (ndb_ie_index_t idx = 0; idx < ndb_alma_file.block_count; ++idx) {
    if (0 != (md[idx].nv_data.state & NDB_IE_VALID) &&
        toa == (u32) data[idx].toa.tow &&
        WN_UNKNOWN == data[idx].toa.wn) {
      log_debug_sid(ndb_almanac[idx].sid,
                   "NDB: updating almanac time (%" PRIu16 ", % " PRIu32 ")",
                   wn, toa);

      /* Week number has not been known before - set it */
      data[idx].toa.wn = (s16) wn;
      md[idx].vflags |= NDB_VFLAG_IE_DIRTY; /* Metadata is not updated */
      ndb_wq_put(&md[idx]);
    }
  }
}

/**
 * Initializes NDB support for almanacs.
 *
 * NDB support for almanacs include separate storages for almanac data and
 * matching week number entries. Potentially there could be separate week
 * number entries and almanacs for every satellite from each of channels.
 * However under normal conditions, such number of invariants should not exceed
 * two (when SV almanac data upload is in progress).
 *
 * Because of that, almanac week numbers are stored separately and used to
 * patch new almanacs.
 *
 * \note GPS only!
 */
void ndb_almanac_init(void)
{
  static bool erase_almanac = true;
  static bool erase_almanac_wn = true;
  SETTING("ndb", "erase_almanac", erase_almanac, TYPE_BOOL);
  SETTING("ndb", "erase_almanac_wn", erase_almanac_wn, TYPE_BOOL);

  ndb_load_data(&ndb_alma_file, erase_almanac);
  ndb_load_data(&ndb_alma_wn_file, erase_almanac_wn);

  /* After startup check if there are any matching WN entries not yet updated
   * in almanac file. Then do cleanup for duplicate entries */
  ndb_lock();
  for (ndb_ie_index_t wn_idx = 0;
       wn_idx < ARRAY_SIZE(ndb_almanac_wn);
       ++wn_idx) {

    if (0 != (ndb_almanac_wn_md[wn_idx].nv_data.state & NDB_IE_VALID)) {
      u16 wn  = ndb_almanac_wn[wn_idx].gps_wn;
      u32 toa = ndb_almanac_wn[wn_idx].gps_toa;

      ndb_alma_wn_update_alma_file(toa, wn);
    }
  }
  ndb_unlock();
}

/**
 * Reads almanac's data from NDB
 *
 * The client may submit almanac's data with or without WN field populated. If
 * WN field is not set, a lookup for a matching TOA/WN pair from NDB is
 * performed. In case WN can't be resolved, almanac is returned with WN set to
 * \a WN_UNKNOWN until matching WN/TOA pair is proved with #ndb_almanac_wn_store
 *
 * \param[in]  sid GNSS signal identifier
 * \param[out] a   Almanac data destination. May have WN field set to
 *                 #WN_UNKNOWN if NDB doesn't have matching TOA/WN pair.
 *
 * \retval NDB_ERR_NONE       On success
 * \retval NDB_ERR_BAD_PARAM  On parameter error
 * \retval NDB_ERR_MISSING_IE No cached data block
 *
 * \sa ndb_almanac_store
 * \sa ndb_almanac_wn_store
 */
ndb_op_code_t ndb_almanac_read(gnss_signal_t sid, almanac_t *a)
{
  u16 idx = map_sid_to_index(sid);

  return ndb_retrieve(&ndb_almanac_md[idx], a, sizeof(*a), NULL, NULL);
}

/**
 * Updates NDB with the new almanac's data.
 *
 * \param[in] src_sid Almanac source in case of data source is NDB_DS_RECEIVER
 * \param[in] a  Almanac's data. Can be with or without WN.
 * \param[in] ds Data source
 * \param[in] sender_id Sender ID if data source is NDB_DS_SBP. In other cases
 *                      set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE             On success. Almanac is already persisted.
 * \retval NDB_ERR_NO_CHANGE        On success. The entry is already persisted.
 * \retval NDB_ERR_BAD_PARAM        Parameter errors.
 * \retval NDB_ERR_UNCONFIRMED_DATA New entry, but confirmation is required.
 */
ndb_op_code_t ndb_almanac_store(const gnss_signal_t *src_sid,
                                const almanac_t *a,
                                ndb_data_source_t ds,
                                u16 sender_id)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != a && a->valid) {
    almanac_t tmp; /* Temporary for almanac's WN field patching */
    u16       wn;  /* Almanac's WN value for matching TOA */

    if (WN_UNKNOWN == a->toa.wn &&
        NDB_ERR_NONE == ndb_almanac_wn_read(a->toa.tow, &wn)) {
      /* If WN is not specified in the almanac, but is found in NDB, update
       * it */
      tmp = *a;
      tmp.toa.wn = wn;
      a = &tmp;
    }

    switch (ndb_alma_candidate_update(a)) {
    case NDB_CAND_IDENTICAL:
      res = NDB_ERR_NO_CHANGE;
      break;
    case NDB_CAND_OLDER:
      res = NDB_ERR_OLDER_DATA;
      break;
    case NDB_CAND_NEW_TRUSTED:
      if (TIME_FINE == time_quality) {
        /* If GPS time is known, save almanac to NDB */
        res = ndb_update(a, ds, &ndb_almanac_md[map_sid_to_index(a->sid)]);
      } else {
        /* If GPS time is unknown, no updates to NDB */
        res = NDB_ERR_TIME_UNKNOWN;
      }
      break;
    case NDB_CAND_NEW_CANDIDATE:
    case NDB_CAND_MISMATCH:
      res = NDB_ERR_UNCONFIRMED_DATA;
      break;
    default:
      assert(!"Invalid status");
    }
  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_ALMANAC,
                     res,
                     ds,
                     &a->sid,
                     src_sid,
                     sender_id);

  return res;
}

/**
 * Read almanac week number for the given ToW
 *
 * \param[in]  toa Almanac's week time [s]
 * \param[out] wn  Almanac's week number if available
 *
 * \retval NDB_ERR_NONE       On success.
 * \retval NDB_ERR_BAD_PARAM  Parameter errors.
 * \retval NDB_ERR_MISSING_IE Entry is not found.
 */
ndb_op_code_t ndb_almanac_wn_read(u32 toa, u16 *wn)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  if (NULL != wn) {
    ndb_alma_wn_t alma_wn;

    res = ndb_find_retrieve(&ndb_alma_wn_file,
                            ndb_alma_wn_match,
                            &toa,
                            &alma_wn,
                            sizeof(alma_wn),
                            NULL,
                            NULL);

    *wn = alma_wn.gps_wn;
  } else {
    res = NDB_ERR_BAD_PARAM;
  }

  return res;
}

/**
 * Add almanac's time and week number pair to NDB database
 *
 * The method takes Almanac's TOA and WN parameters for almanac's TOA/WN
 * matching.
 *
 * If the WN/TOA pair is already in NDB database, nothing is done and
 * \a NDB_ERR_NONE is returned.
 *
 * If the cache contains same TOA with different WN, the cache entry is replaced
 * and is subject for subsequent verification.
 *
 * If the cache doesn't contain same TOA, an attempt to add a new cache entry
 * is made. If the cache is full, the entry with earliest time is dropped. The
 * time is TAI computed by formula: `WN * WEEK_SECS + TOA`.
 *
 * Whenever the WN entry is confirmed, it is removed from the candidate cache,
 * and all almanacs in NDB database with matching TOA and without WN are
 * updated.
 *
 * \param[in] sid  Time source
 * \param[in] tow  Almanac's week time [s]
 * \param[in] wn   Almanac's week number
 * \param[in] ds   Data source
 * \param[in] sender_id Sender ID if data source is NDB_DS_SBP. In other cases
 *                      set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE             On success. Entry has been persisted.
 * \retval NDB_ERR_NO_CHANGE        On success. The entry is already persisted.
 * \retval NDB_ERR_UNCONFIRMED_DATA New entry, but confirmation is required.
 */
ndb_op_code_t ndb_almanac_wn_store(gnss_signal_t sid, u32 toa, u16 wn,
                                   ndb_data_source_t ds, u16 sender_id)
{
  ndb_op_code_t res = NDB_ERR_ALGORITHM_ERROR;

  switch (ndb_alma_wn_candidate_update(toa, wn))
  {
  case NDB_CAND_IDENTICAL:
    /* Nothing to be done: same data is already present in NDB file */
    res = NDB_ERR_NO_CHANGE;
    break;
  case NDB_CAND_NEW_CANDIDATE:
  case NDB_CAND_MISMATCH:
    res = NDB_ERR_UNCONFIRMED_DATA;
    break;
  case NDB_CAND_NEW_TRUSTED:
    if (TIME_FINE == time_quality) {
      /* If GPS time is known, save almanac wn to NDB. */
      /* Perform NDB database update inside NDB lock section */
      ndb_lock();
      /* Create persistent TAI/WN pair entry */
      ndb_alma_wn_update_wn_file(toa, wn, ds);
      /* Update persistent almanac entries with matching TAI */
      ndb_alma_wn_update_alma_file(toa, wn);
      ndb_unlock();
      res = NDB_ERR_NONE;
    } else {
      /* If GPS time is unknown, no updates to NDB */
      return NDB_ERR_TIME_UNKNOWN;
    }
    break;
  case NDB_CAND_OLDER:
  default:
    assert(!"Unexpected almanac's TOA/WN candidate status");
  }

  sbp_send_ndb_event(NDB_EVENT_STORE,
                     NDB_EVENT_OTYPE_ALMANAC_WN,
                     res,
                     ds,
                     NULL,
                     &sid,
                     sender_id);

  return res;
}

/**
 * Erase almanac data for a given satellite
 *
 * \param[in] sid SV signal identifier
 *
 * \retval NDB_ERR_NONE      Successful operation.
 * \retval NDB_ERR_NO_CHANGE No data to erase.
 * \retval NDB_ERR_BAD_PARAM Bad parameter.
 */
ndb_op_code_t ndb_almanac_erase(gnss_signal_t sid)
{
  u16 idx = map_sid_to_index(sid);

  ndb_op_code_t res = ndb_erase(&ndb_almanac_md[idx]);

  chMtxLock(&cand_list_access);
  for (s16 cand_idx = ndb_alma_candidate_find(sid, -1);
       cand_idx != -1;
       cand_idx = ndb_alma_candidate_find(sid, cand_idx)) {
    ndb_alma_candidate_release(cand_idx);
  }
  chMtxUnlock(&cand_list_access);

  sbp_send_ndb_event(NDB_EVENT_ERASE,
                     NDB_EVENT_OTYPE_ALMANAC,
                     res,
                     NDB_DS_UNDEFINED,
                     &sid,
                     NULL,
                     NDB_EVENT_SENDER_ID_VOID);

  return res;
}

/**
 * Updates health flags of existing almanac.
 *
 * \param[in] target_sid  GNSS signal identifier to update
 * \param[in] health_bits Health bits (5 LSB)
 * \param[in] ds          Data source
 * \param[in] src_sid     hb source in case of data source being NDB_DS_RECEIVER
 * \param[in] sender_id   Sender ID if data source is NDB_DS_SBP. In other cases
 *                        set to NDB_EVENT_SENDER_ID_VOID.
 *
 * \retval NDB_ERR_NONE             On success. Health data is updated.
 * \retval NDB_ERR_NO_CHANGE        On success. Health data is unchanged.
 * \retval NDB_ERR_BAD_PARAM        Parameter errors.
 * \retval NDB_ERR_UNCONFIRMED_DATA New entry, but confirmation is required.
 * \retval NDB_ERR_NO_DATA          No data entry to update.
 */
ndb_op_code_t ndb_almanac_hb_update(gnss_signal_t target_sid,
                                    u8 health_bits,
                                    ndb_data_source_t ds,
                                    const gnss_signal_t *src_sid,
                                    u16 sender_id)
{
  health_bits &= 0x1F;

  almanac_t tmp;
  if (NDB_ERR_NONE == ndb_almanac_read(target_sid, &tmp) &&
      (tmp.health_bits & 0x1F) != health_bits) {
    tmp.health_bits &= 0xE0;
    tmp.health_bits |= health_bits;
    return ndb_almanac_store(src_sid, &tmp, ds, sender_id);
  }

  return NDB_ERR_NO_DATA;
}

/**
 * Sends out MsgAlmanac
 *
 * \param[in] sid  GNSS signal identifier to indicate which alma to send
 *
 * \retval TRUE    Alma found, valid and sent
 * \retval FALSE   Alma not sent
 */
bool ndb_almanac_sbp_update_tx(gnss_signal_t sid)
{
  almanac_t a;
  gps_time_t t = get_current_time();
  enum ndb_op_code oc = ndb_almanac_read(sid, &a);
  if (NDB_ERR_NONE == oc && almanac_valid(&a, &t)) {
    msg_almanac_t msg;
    msg_info_t info = pack_almanac(&a, &msg);
    sbp_send_msg(info.msg_id, info.size, (u8 *)&msg);
    return TRUE;
  }

  return FALSE;
}

static ndb_sbp_update_info_t alma_update_info = {
  NDB_SBP_UPDATE_CYCLE_COUNT_INIT,
  NDB_SBP_UPDATE_SIG_IDX_INIT,
  NDB_ALMA_TRANSMIT_EPOCH_SPACING,
  NDB_ALMA_MESSAGE_SPACING,
  &ndb_almanac_sbp_update_tx
};

/**
 * The function sends almanac if valid.
 *
 * Function is supposed to be called every NV_WRITE_REQ_TIMEOUT ms from
 * NDB thread.
 */
void ndb_almanac_sbp_update(void)
{
  ndb_sbp_update(&alma_update_info);
}

