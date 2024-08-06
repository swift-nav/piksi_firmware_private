/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* This is an auto-generated file. */

#ifndef STARLING_BUILD_CONFIG_H
#define STARLING_BUILD_CONFIG_H

#include <swiftnav/signal.h>

#define BUILD_CONFIG_ENABLE_BDS 1
#define BUILD_CONFIG_ENABLE_GAL 1
#define BUILD_CONFIG_ENABLE_GLO 1
#define BUILD_CONFIG_ENABLE_GPS 1
#define BUILD_CONFIG_ENABLE_QZSS 1
#define BUILD_CONFIG_ENABLE_SBAS 0

#define BUILD_CONFIG_L1_ONLY 0

#define BUILD_CONFIG_ENABLE_AMBIGUITY_FIXING_AND_TIME_MATCHING 1
#define BUILD_CONFIG_ENABLE_INSIGHTS 1
#define BUILD_CONFIG_ENABLE_RTK_DIFFERENTIAL 1
#define BUILD_CONFIG_ENABLE_SSR 1
#define BUILD_CONFIG_ENABLE_STARLING_LITE 0

#define BUILD_CONFIG_ASSERT_STATEMENTS 1
#define BUILD_CONFIG_LIMITED_LOGGING 1
#define BUILD_CONFIG_COMPILED_OTL 0
#define BUILD_CONFIG_DATA_FLOAT_TYPE double

#define BUILD_CONFIG_TUNING_PROFILE_MOBILE 0

#define BUILD_CONFIG_INSTANTANEOUS_VELOCITY 1

#ifdef NETWORK
#define MAX_CHANNELS (300)
#define STARLING_MAX_CHANNEL_COUNT (80)
#else
#define MAX_CHANNELS (79)
#define STARLING_MAX_CHANNEL_COUNT (44)
#endif

#ifdef __cplusplus
extern "C" {
#endif

static const int cNumSat = (BUILD_CONFIG_ENABLE_BDS ? NUM_SATS_BDS : 0) +
                           (BUILD_CONFIG_ENABLE_GAL ? NUM_SATS_GAL : 0) +
                           (BUILD_CONFIG_ENABLE_GLO ? NUM_SATS_GLO : 0) +
                           (BUILD_CONFIG_ENABLE_GPS ? NUM_SATS_GPS : 0) +
                           (BUILD_CONFIG_ENABLE_QZSS ? NUM_SATS_QZS : 0) +
                           (BUILD_CONFIG_ENABLE_SBAS ? NUM_SATS_SBAS : 0);

static const int cNumSignals = (BUILD_CONFIG_ENABLE_BDS ? NUM_SIGNALS_BDS : 0) +
                               (BUILD_CONFIG_ENABLE_GAL ? NUM_SIGNALS_GAL : 0) +
                               (BUILD_CONFIG_ENABLE_GLO ? NUM_SIGNALS_GLO : 0) +
                               (BUILD_CONFIG_ENABLE_GPS ? NUM_SIGNALS_GPS : 0) +
                               (BUILD_CONFIG_ENABLE_QZSS ? NUM_SIGNALS_QZS : 0) +
                               (BUILD_CONFIG_ENABLE_SBAS ? NUM_SIGNALS_SBAS : 0);

static const constellation_t cAllConstellations[] = {
#if BUILD_CONFIG_ENABLE_BDS
    CONSTELLATION_BDS,
#endif
#if BUILD_CONFIG_ENABLE_GAL
    CONSTELLATION_GAL,
#endif
#if BUILD_CONFIG_ENABLE_GLO
    CONSTELLATION_GLO,
#endif
#if BUILD_CONFIG_ENABLE_GPS
    CONSTELLATION_GPS,
#endif
#if BUILD_CONFIG_ENABLE_QZSS
    CONSTELLATION_QZS,
#endif
#if BUILD_CONFIG_ENABLE_SBAS
    CONSTELLATION_SBAS,
#endif
};

struct StarlingBuildSetting {
  const char *name;
  const char *value;
};

static const struct StarlingBuildSetting cStarlingBuildSettings[] = {
    {"enable_bds", BUILD_CONFIG_ENABLE_BDS ? "True" : "False"},
    {"enable_gal", BUILD_CONFIG_ENABLE_GAL ? "True" : "False"},
    {"enable_glo", BUILD_CONFIG_ENABLE_GLO ? "True" : "False"},
    {"enable_gps", BUILD_CONFIG_ENABLE_GPS ? "True" : "False"},
    {"enable_qzss", BUILD_CONFIG_ENABLE_QZSS ? "True" : "False"},
    {"enable_sbas", BUILD_CONFIG_ENABLE_SBAS ? "True" : "False"},
    {"enable_l1_only", BUILD_CONFIG_L1_ONLY ? "True" : "False"},
    {"enable_ambiguity_fixing_and_time_matching", BUILD_CONFIG_ENABLE_AMBIGUITY_FIXING_AND_TIME_MATCHING ? "True" : "False"},
    {"enable_insights", BUILD_CONFIG_ENABLE_INSIGHTS ? "True" : "False"},
    {"enable_rtk_differential", BUILD_CONFIG_ENABLE_RTK_DIFFERENTIAL ? "True" : "False"},
    {"enable_ssr", BUILD_CONFIG_ENABLE_SSR ? "True" : "False"},
    {"enable_starling_lite", BUILD_CONFIG_ENABLE_STARLING_LITE ? "True" : "False"},
    {"max_channels", "79"},
    {"internal_max_channels", "44"},
    {"has_assert", BUILD_CONFIG_ASSERT_STATEMENTS ? "True" : "False"},
    {"instantanous_velocity", BUILD_CONFIG_INSTANTANEOUS_VELOCITY ? "True" : "False"},
    {"compiled_otl", BUILD_CONFIG_COMPILED_OTL ? "True" : "False"},
    {"float_type", "double"},
    {"tuning_profile_mobile", BUILD_CONFIG_TUNING_PROFILE_MOBILE ? "True" : "False"},
};


/* We now "expand" tuning profiles into more fine grained settings.
 * The fine grained settings will not be logged at runtime, only the settings
 * in the above cStarlingBuildSettings array are logged. One can figure out
 * the fine grained settings by having a version tag and the tuning profile.
 */


#if !BUILD_CONFIG_TUNING_PROFILE_MOBILE /* The default case */

#define MAX_POS_ACCURACY_THRESHOLD_M (UINT16_MAX/1000)
#define MAX_VEL_ACCURACY_THRESHOLD_M_PER_S (5.0)
#define TIME_MATCH_THRESHOLD_S (5e-2)
#define MAX_CORRECTION_AGE_S (30)
#define PVT_LABEL_DGNSS_AS_FLOAT 0
#define PVT_OVERRIDE_RTK_WITH_SPP_IF_BETTER 1
#define PVT_MAX_SATS_1_HZ (22)
#define DEFAULT_THREAD_STACK_SIZE (3*1024*1024)
#define INS_STREAM_REQUIRED_CAPACITY (1*100+60)
#define IMU_STREAM_REQUIRED_CAPACITY (1*100+60)
#define ALIGNMENT_IMU_BUFFER_SIZE (1)

#else /* The mobile case */

#define MAX_POS_ACCURACY_THRESHOLD_M (UINT16_MAX)
#define MAX_VEL_ACCURACY_THRESHOLD_M_PER_S (100.0)
#define TIME_MATCH_THRESHOLD_S (0.5)
#define MAX_CORRECTION_AGE_S (300)
#define PVT_LABEL_DGNSS_AS_FLOAT 1
#define PVT_OVERRIDE_RTK_WITH_SPP_IF_BETTER 0
#define PVT_MAX_SATS_1_HZ (50)
#define DEFAULT_THREAD_STACK_SIZE (12*1024*1024)
#define INS_STREAM_REQUIRED_CAPACITY (1*400+240)
#define IMU_STREAM_REQUIRED_CAPACITY (1*400+240)
#define ALIGNMENT_IMU_BUFFER_SIZE (10)

#endif

#ifdef __cplusplus
}
#endif

#endif /* STARLING_BUILD_CONFIG_H */
