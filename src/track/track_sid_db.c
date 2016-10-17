/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri.atamaniouk@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/track.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/time.h>

#include <track.h>

#include "track_profiles.h"
#include "track_profile_utils.h"
#include "track_sid_db.h"

#include <math.h>
#include <string.h>
#include <assert.h>

/** Maximum interval for reusing cached ToW values [ms] */
#define MAXIMUM_DB_CACHE_USE_INTERVAL_MS (3 * MINUTE_SECS * 1000)

/**
 * Cache entry.
 *
 * Container for information, that is accessed by SID, not by tracker number.
 * The type is \a volatile to eliminate reordering.
 */
typedef struct
{
  tp_tow_entry_t       tow;        /**< ToW cache entry */
  tp_elevation_entry_t elevation;  /**< SV elevation cache entry */
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
  mutex_t              mutex;                     /**< DB mutex */
  sid_db_cache_entry_t gps_entries[NUM_SATS_GPS]; /**< Cache entries for GPS */
} sid_db_cache_t;

static sid_db_cache_t sid_db_cache;

/**
 * Initializes ToW computation subsystem.
 */
void track_sid_db_init(void)
{
  chMtxObjectInit(&sid_db_cache.mutex);
  for (size_t i = 0; i < NUM_SIGNALS_GPS_L1CA; ++i) {
    volatile sid_db_cache_entry_t *entry = &sid_db_cache.gps_entries[i];
    entry->tow.TOW_ms = TOW_UNKNOWN;
    entry->elevation.elevation_d = TRACKING_ELEVATION_UNKNOWN;
  }
}

/**
 * Provides access to ToW cache entry.
 *
 * \param[in]  sid       GNSS signal identifier.
 * \param[out] tow_entry ToW entry for the given signal.
 *
 * \retval true  If ToW entry has been loaded.
 * \retval false If ToW entry is not present.
 */
bool track_sid_db_load_tow(gnss_signal_t sid, tp_tow_entry_t *tow_entry)
{
  bool result = false;

  if (NULL != tow_entry && CONSTELLATION_GPS == sid_to_constellation(sid)) {
    u8 sv_index = sid_to_code_index(sid);
    chMtxLock(&sid_db_cache.mutex);
    *tow_entry = sid_db_cache.gps_entries[sv_index].tow;
    chMtxUnlock(&sid_db_cache.mutex);
    result = true;
  }

  return result;
}

/**
 * Provides update of ToW cache entry.
 *
 * \param[in] sid       GNSS signal identifier.
 * \param[in] tow_entry ToW data.
 *
 * \retval true  If ToW entry has been updated.
 * \retval false If ToW entry is not present.
 */
bool track_sid_db_update_tow(gnss_signal_t sid, const tp_tow_entry_t *tow_entry)
{
  bool result = false;

  if (NULL != tow_entry && CONSTELLATION_GPS == sid_to_constellation(sid)) {
    u8 sv_index = sid_to_code_index(sid);
    chMtxLock(&sid_db_cache.mutex);
    sid_db_cache.gps_entries[sv_index].tow = *tow_entry;
    chMtxUnlock(&sid_db_cache.mutex);
    result = true;
  }

  return result;
}


/**
 * Loads SV elevation data from the cache.
 *
 * \param[in]  sid             GNSS signal identifier.
 * \param[out] elevation_entry Container for loaded data.
 *
 * \retval true  If elevation entry has been loaded.
 * \retval false If elevation entry is not present.
 */
bool track_sid_db_load_elevation(gnss_signal_t sid,
                                 tp_elevation_entry_t *elevation_entry)
{
  bool result = false;

  if (NULL != elevation_entry &&
      CONSTELLATION_GPS == sid_to_constellation(sid)) {
    u8 sv_index = sid_to_code_index(sid);
    chMtxLock(&sid_db_cache.mutex);
    *elevation_entry = sid_db_cache.gps_entries[sv_index].elevation;
    chMtxUnlock(&sid_db_cache.mutex);
    result = true;
  }

  return result;
}

/**
 * Stores SV elevation data into the cache.
 *
 * \param[in] sid             GNSS signal identifier.
 * \param[in] elevation_entry Data to store.
 *
 * \retval true  If elevation entry has been updated.
 * \retval false If elevation entry is not present.
 */
bool track_sid_db_update_elevation(gnss_signal_t sid,
                                   const tp_elevation_entry_t *elevation_entry)
{
  bool result = false;

  if (NULL != elevation_entry &&
      CONSTELLATION_GPS == sid_to_constellation(sid)) {
    u8 sv_index = sid_to_code_index(sid);
    chMtxLock(&sid_db_cache.mutex);
    sid_db_cache.gps_entries[sv_index].elevation = *elevation_entry;
    chMtxUnlock(&sid_db_cache.mutex);
    result = true;
  }

  return result;
}

/**
 * Computes ToW estimate according to previous ToW and time interval.
 *
 * \param[in]  old_ToW_ms Previous ToW value [ms].
 * \param[in]  delta_tk   Time interval from the previous ToW [ticks]
 * \param[in]  ms_align   ToW alignment flag [ms]:
 *                        - 1 for 1ms alignment (default)
 *                        - 20 for GPS bit alignment
 *                        - Other values for word, frame, message alignments.
 * \param[out] error_ms   Optional alignment error: difference between
 *                        computed and aligned results.
 *
 * \return Computed ToW if >= 0 or #TOW_UNKNOWN on error.
 */
s32 tp_tow_compute(s32 old_ToW_ms, u64 delta_tk, u8 ms_align, double *error_ms)
{
  s32 ToW_ms = TOW_UNKNOWN;

  if (TOW_UNKNOWN != old_ToW_ms) {
    /* Propagate ToW time from cached entry according to estimated time
     * jump. */
    double delta_d = nap_count_to_ms(delta_tk);

    if (delta_d <= MAXIMUM_DB_CACHE_USE_INTERVAL_MS) {
      double tmp_ToW_ms = old_ToW_ms + delta_d;
      ToW_ms = (s32)round(tmp_ToW_ms);

      if (ms_align > 1) {
        /* If the result is known to be aligned by some interval, do it here. */
        s32 round = ToW_ms % ms_align;
        if (round < (ms_align >> 1)) {
          ToW_ms -= round;
        } else {
          ToW_ms += ms_align - round;
        }
      }

      if (NULL != error_ms) {
        /* Compute rounding/aligning error */
        *error_ms = ToW_ms - tmp_ToW_ms;
      }

      /* Fix up ToW */
      ToW_ms %= WEEK_MS;
    }
  }

  return ToW_ms;
}
