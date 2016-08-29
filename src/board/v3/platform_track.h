/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Valeri Atamaniouk <valeri@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_PLATFORM_TRACK_H
#define SWIFTNAV_PLATFORM_TRACK_H

/** Enable DLL error lock detector on V3 */
#define USE_DLL_ERROR 1

/* Memory segments for V3 in the order of preference:
 * - empty -- Default data segment
 * - _CCM  -- CCM segment
 * - _BCKP -- Data backup segment
 */
/** Tracker main data segment */
#define PLATFORM_TRACK_DATA_TRACKER
/** Tracker profiles data segment */
#define PLATFORM_TRACK_DATA_PROFILES
/** Tracker aux parameters data segment */
#define PLATFORM_TRACK_DATA_FILTERS

#endif /* SWIFTNAV_PLATFORM_TRACK_H */

