/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Michele Bavaro <michele@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "track_sid_db.h"

#include <assert.h>
#include <inttypes.h>
#include <swiftnav/constants.h>
#include <swiftnav/gnss_time.h>

#include "signal_db/signal_db.h"
#include "timing/timing.h"
#include "track_api.h"
#include "track_flags.h"

/** Maximum interval for reusing cached ToW values [ms] */
#define MAXIMUM_DB_CACHE_USE_INTERVAL_MS (3 * MINUTE_SECS * 1000)

/**
 * Cache entry.
 *
 * Container for information, that is accessed by SID, not by tracker number.
 * Note that data is saved per SV, and each SID originating from that SV
 * accesses the same data.
 * The type is \a volatile to eliminate reordering.
 */
typedef struct {
  tp_tow_entry_t tow;   /**< ToW cache entry */
  tp_azel_entry_t azel; /**< SV azimuth & elevation cache entry */
  xcorr_positions_t
      positions; /**< SV cross-correlation positions cache entry */
} volatile sid_db_cache_entry_t;

/**
 * SID data cache.
 *
 * This structure encapsulates ToW and elevation cache entries for GPS
 * satellites.
 *
 * L1 C/A and L2 C ToW are almost the same with difference due to group and iono
 * delays. The difference is ignored.
 */
typedef struct {
  mutex_t mutex; /**< DB mutex */
  sid_db_cache_entry_t
      entries[NUM_SATS]; /**< Cache entries for all satellites */
} sid_db_cache_t;

static sid_db_cache_t sid_db_cache;

/**
 * Initializes ToW computation subsystem.
 */
void track_sid_db_init(void) {
  chMtxObjectInit(&sid_db_cache.mutex);
  for (size_t i = 0; i < NUM_SATS; ++i) {
    volatile sid_db_cache_entry_t *entry = &sid_db_cache.entries[i];
    entry->tow.TOW_ms = TOW_UNKNOWN;
    entry->tow.TOW_residual_ns = 0;
    entry->azel.timestamp_tk = 0;
  }
}

/**
 * Clears GLO ToW cache prior to leap second event.
 */
void track_sid_db_clear_glo_tow(void) {
  tp_tow_entry_t tow_entry = {
      .TOW_ms = TOW_UNKNOWN, .TOW_residual_ns = 0, .sample_time_tk = 0};
  for (u8 i = GLO_FIRST_PRN; i <= NUM_SATS_GLO; ++i) {
    gnss_signal_t sid = construct_sid(CODE_GLO_L1OF, i);
    track_sid_db_update_tow(sid, &tow_entry);
  }
}

/**
 * Provides access to ToW cache entry.
 *
 * \param[in]  sid       GNSS signal identifier.
 * \param[out] tow_entry ToW entry for the given signal.
 */
void track_sid_db_load_tow(const gnss_signal_t sid, tp_tow_entry_t *tow_entry) {
  assert(tow_entry);

  u16 sv_index = sid_to_sv_index(sid);
  chMtxLock(&sid_db_cache.mutex);
  *tow_entry = sid_db_cache.entries[sv_index].tow;
  chMtxUnlock(&sid_db_cache.mutex);
}

/**
 * Provides update of ToW cache entry.
 *
 * \param[in] sid       GNSS signal identifier.
 * \param[in] tow_entry ToW data.
 */
void track_sid_db_update_tow(const gnss_signal_t sid,
                             const tp_tow_entry_t *tow_entry) {
  assert(tow_entry);

  u16 sv_index = sid_to_sv_index(sid);
  chMtxLock(&sid_db_cache.mutex);
  sid_db_cache.entries[sv_index].tow = *tow_entry;
  chMtxUnlock(&sid_db_cache.mutex);
}

/**
 * Loads SV elevation data from the cache.
 *
 * \param[in]  sid             GNSS signal identifier.
 * \param[out] azel_entry      Container for loaded data.
 *
 * \retval true  If elevation entry has been loaded.
 * \retval false If elevation entry is not present.
 */
static bool track_sid_db_load_azel(const gnss_signal_t sid,
                                   tp_azel_entry_t *azel_entry) {
  if (NULL == azel_entry) {
    return false;
  }

  u16 sv_index = sid_to_sv_index(sid);
  chMtxLock(&sid_db_cache.mutex);
  *azel_entry = sid_db_cache.entries[sv_index].azel;
  chMtxUnlock(&sid_db_cache.mutex);

  /* if timestamp is not set, then the entry has not been set yet */
  return (azel_entry->timestamp_tk > 0);
}

/**
 * Stores SV elevation data into the cache.
 *
 * \param[in] sid             GNSS signal identifier.
 * \param[in] elevation_entry Data to store.
 *
 */
static void track_sid_db_update_azel(const gnss_signal_t sid,
                                     const tp_azel_entry_t *azel_entry) {
  assert(NULL != azel_entry);
  u16 sv_index = sid_to_sv_index(sid);
  chMtxLock(&sid_db_cache.mutex);
  sid_db_cache.entries[sv_index].azel = *azel_entry;
  chMtxUnlock(&sid_db_cache.mutex);
}

/** Set the azimuth and elevation angles for SV by sid.
 *
 * \param[in] sid       Signal identifier for which the elevation should be set.
 * \param[in] azimuth   Azimuth angle [degrees].
 * \param[in] elevation Elevation angle [degrees].
 * \param[in] timestamp Azimuth and elevation evaluation time [ticks].
 *
 * \sa sv_elevation_degrees_get
 */
void track_sid_db_azel_degrees_set(const gnss_signal_t sid,
                                   double azimuth,
                                   double elevation,
                                   u64 timestamp) {
  tp_azel_entry_t entry = {.azimuth_d = azimuth,
                           .elevation_d = elevation,
                           .timestamp_tk = timestamp};
  track_sid_db_update_azel(sid, &entry);
}

/** Return the azimuth angle for a satellite.
 *
 * \param[in] sid Signal identifier for which the elevation should be returned.
 * \param[out] result Pointer for storing the SV azimuth in degrees
 *
 * \return true if success, false if azimuth is not present in the cache,
 *                                   cache entry is too old, or GNSS
 *                                   constellation is not supported.
 *
 * \sa sv_azimuth_degrees_set
 */
bool track_sid_db_azimuth_degrees_get(const gnss_signal_t sid, double *result) {
  tp_azel_entry_t entry = {0};
  /* If azimuth cache entry is loaded, do the entry age check */
  if (track_sid_db_load_azel(sid, &entry) &&
      nap_timing_count() - entry.timestamp_tk < SEC2TICK(MAX_AZ_EL_AGE_SEC)) {
    *result = entry.azimuth_d;
    return true;
  }
  return false;
}

/** Return the elevation angle for a satellite.
 *
 * \param[in] sid Signal identifier for which the elevation should be returned.
 * \param[out] result Pointer for storing the SV elevation in degrees
 *
 * \return true if success, false if elevation is not present in the cache,
 *              or signal is not supported.
 */
bool sid_db_elevation_degrees_get(const gnss_signal_t sid, float *elev) {
  tp_azel_entry_t entry = {0};
  if (sid_valid(sid) && elev && track_sid_db_load_azel(sid, &entry)) {
    *elev = (float)entry.elevation_d;
    return true;
  }
  return false;
}

/** Return the elevation angle for a satellite.
 *
 * \param[in] sid Signal identifier for which the elevation should be returned.
 * \param[out] result Pointer for storing the SV elevation in degrees
 *
 * \return true if success, false if elevation is not present in the cache,
 *                                   cache entry is too old, or GNSS
 *                                   constellation is not supported.
 *
 * \sa sv_elevation_degrees_set
 */
bool track_sid_db_elevation_degrees_get(const gnss_signal_t sid, double *elev) {
  tp_azel_entry_t entry = {0};
  /* If elevation cache entry is loaded, do the entry age check */
  bool db_load = track_sid_db_load_azel(sid, &entry);
  bool up_to_date =
      (nap_timing_count() - entry.timestamp_tk) < SEC2TICK(MAX_AZ_EL_AGE_SEC);
  if (db_load && up_to_date) {
    *elev = entry.elevation_d;
    return true;
  }
  return false;
}

/**
 * Computes ToW estimate according to previous ToW and time interval.
 *
 * \param[in]  old_TOW_ms Previous ToW value [ms].
 * \param[in]  delta_tk   Time interval from the previous ToW [ticks]
 * \param[in]  ms_align   ToW alignment flag [ms]:
 *                        - 1 for 1ms alignment (default)
 *                        - 20 for GPS bit alignment
 *                        - Other values for word, frame, message alignments.
 * \param[out] error_ms   Optional alignment error: difference between
 *                        computed and aligned results.
 *
 * \return Computed ToW if >= 0 or \a TOW_UNKNOWN on error.
 */
s32 tp_tow_compute(s32 old_TOW_ms,
                   u64 delta_tk,
                   u8 ms_align,
                   double *error_ms) {
  s32 TOW_ms = TOW_UNKNOWN;

  if (old_TOW_ms == TOW_UNKNOWN) {
    return TOW_ms;
  }

  /* Propagate ToW time from cached entry according to estimated time
   * jump. */
  double delta_d = nap_count_to_ms(delta_tk);

  /* Check interval sanity and validity */
  if (delta_d < 0 || delta_d > MAXIMUM_DB_CACHE_USE_INTERVAL_MS) {
    return TOW_ms;
  }

  double tmp_TOW_ms = old_TOW_ms + delta_d;
  TOW_ms = (s32)round(tmp_TOW_ms);

  if (ms_align > 1) {
    /* If the result is known to be aligned by some interval, do it here. */
    s32 round = TOW_ms % ms_align;
    if (round < (ms_align >> 1)) {
      TOW_ms -= round;
    } else {
      TOW_ms += ms_align - round;
    }
  }

  if (NULL != error_ms) {
    /* Compute rounding/aligning error */
    *error_ms = TOW_ms - tmp_TOW_ms;
  }

  /* Fix up ToW */
  TOW_ms %= WEEK_MS;

  if (!tp_tow_is_sane(TOW_ms)) {
    TOW_ms = TOW_UNKNOWN;
  }

  return TOW_ms;
}

/**
 * Test ToW for validity.
 *
 * \param[in] tow_ms ToW value to check
 *
 * \retval true  ToW is in range of [0; WEEK_MS) or has a value of TOW_UNKNOWN.
 * \retval false ToW is not valid.
 */
bool tp_tow_is_sane(s32 tow_ms) {
  return tow_ms == TOW_UNKNOWN || (tow_ms >= 0 && tow_ms < WEEK_MS);
}

/**
 * Loads SV almanac-based positions from the cache.
 *
 * \param[in]  sid            GNSS signal identifier.
 * \param[out] position_entry Container for loaded data.
 *
 * \retval true  If position entry has been loaded.
 * \retval false If position entry is not present.
 *
 * \sa track_sid_db_update_positions
 */
bool track_sid_db_load_positions(const gnss_signal_t sid,
                                 xcorr_positions_t *position_entry) {
  bool result = false;

  if (NULL != position_entry) {
    u16 sv_index = sid_to_sv_index(sid);
    chMtxLock(&sid_db_cache.mutex);
    *position_entry = sid_db_cache.entries[sv_index].positions;
    chMtxUnlock(&sid_db_cache.mutex);
    result = true;
  }

  return result;
}

/**
 * Stores SV position data into the cache.
 *
 * \param[in] sid            GNSS signal identifier.
 * \param[in] position_entry Data to store.
 *
 * \retval true  If position entry has been updated.
 * \retval false If position entry is not present.
 *
 * \sa track_sid_db_load_positions
 */
bool track_sid_db_update_positions(const gnss_signal_t sid,
                                   const xcorr_positions_t *position_entry) {
  bool result = false;

  if (NULL != position_entry) {
    u16 sv_index = sid_to_sv_index(sid);
    chMtxLock(&sid_db_cache.mutex);
    sid_db_cache.entries[sv_index].positions = *position_entry;
    chMtxUnlock(&sid_db_cache.mutex);
    result = true;
  }

  return result;
}

static bool tow_cache_sid_available(tracker_t *tracker, gnss_signal_t *sid) {
  me_gnss_signal_t mesid = tracker->mesid;
  u16 glo_orbit_slot = 0;

  if (IS_GLO(mesid)) {
    /* Check that GLO orbit slot ID is available */
    glo_orbit_slot = tracker_glo_orbit_slot_get(tracker);
    if (!glo_slot_id_is_valid(glo_orbit_slot)) {
      /* If no GLO orbit slot ID is available,
       * then cannot proceed with TOW cache write. */
      return false;
    }
    *sid = construct_sid(mesid.code, glo_orbit_slot);
  } else {
    *sid = construct_sid(mesid.code, mesid.sat);
  }
  return true;
}

/**
 * Stores TOW info into the cache.
 *
 * \param[in] tracker Tracker channel data
 */
void update_tow_in_sid_db(tracker_t *tracker) {
  gnss_signal_t sid;
  if (!tow_cache_sid_available(tracker, &sid)) {
    return;
  }

  u64 sample_time_tk = nap_sample_time_to_count(tracker->sample_count);

  /* Update TOW cache */
  tp_tow_entry_t tow_entry = {.TOW_ms = tracker->TOW_ms,
                              .TOW_residual_ns = tracker->TOW_residual_ns,
                              .sample_time_tk = sample_time_tk};
  track_sid_db_update_tow(sid, &tow_entry);
}

/**
 * Reads TOW info from the cache.
 *
 * \param[in] tracker Tracker channel data
 */
void propagate_tow_from_sid_db(tracker_t *tracker) {
  me_gnss_signal_t mesid = tracker->mesid;
  gnss_signal_t sid;
  if (!tow_cache_sid_available(tracker, &sid)) {
    return;
  }

  /* Read TOW cache */
  tp_tow_entry_t tow_entry;
  track_sid_db_load_tow(sid, &tow_entry);
  if (TOW_UNKNOWN == tow_entry.TOW_ms) {
    /* No valid cached TOW value found */
    return;
  }

  /* There is a valid cached TOW value */
  double error_ms = 0;
  u8 bit_length = tracker_bit_length_get(tracker);
  u64 sample_time_tk = nap_sample_time_to_count(tracker->sample_count);
  u64 time_delta_tk = sample_time_tk - tow_entry.sample_time_tk;
  s32 TOW_ms =
      tp_tow_compute(tow_entry.TOW_ms, time_delta_tk, bit_length, &error_ms);

  if (TOW_UNKNOWN == TOW_ms) {
    return;
  }

  log_debug_mesid(mesid,
                  "[+%" PRIu32 "ms] Initializing TOW from cache [%" PRIu8
                  "ms]"
                  " delta=%.2lfms ToW=%" PRId32 "ms error=%lf",
                  tracker->update_count,
                  bit_length,
                  nap_count_to_ms(time_delta_tk),
                  TOW_ms,
                  error_ms);

  if (tp_tow_is_sane(TOW_ms)) {
    tracker->TOW_residual_ns = tow_entry.TOW_residual_ns;
    tracker->TOW_ms = TOW_ms;
    tracker->flags |= TRACKER_FLAG_TOW_VALID;
  } else {
    log_error_mesid(mesid,
                    "[+%" PRIu32 "ms] Error TOW propagation %" PRId32,
                    tracker->update_count,
                    tracker->TOW_ms);
    tracker->TOW_residual_ns = 0;
    tracker->TOW_ms = TOW_UNKNOWN;
    tracker->flags &= ~TRACKER_FLAG_TOW_VALID;
  }
}

/**
 * Clear TOW info from the cache.
 *
 * \param[in] sid   Target GNSS signal identifier.
 */
void clear_tow_in_sid_db(const gnss_signal_t sid) {
  /* Update TOW cache */
  tp_tow_entry_t tow_entry = {
      .TOW_ms = TOW_UNKNOWN, .TOW_residual_ns = 0, .sample_time_tk = 0};
  track_sid_db_update_tow(sid, &tow_entry);
}
