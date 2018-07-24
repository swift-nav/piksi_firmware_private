/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SIGNAL_H
#define SWIFTNAV_SIGNAL_H

#include <stdbool.h>
#include <swiftnav/common.h>
#include <swiftnav/signal.h>

#include "board/v3/platform_signal.h"

/** \addtogroup signal
 * \{ */

/* Number of signals on each code supported
 * on the current hardware platform. */
#define PLATFORM_SIGNAL_COUNT_GPS_L1CA \
  (CODE_GPS_L1CA_SUPPORT ? NUM_SIGNALS_GPS_L1CA : 0)
#define PLATFORM_SIGNAL_COUNT_GPS_L2C \
  (CODE_GPS_L2C_SUPPORT ? NUM_SIGNALS_GPS_L2C : 0)
#define PLATFORM_SIGNAL_COUNT_GPS_L5 \
  (CODE_GPS_L5_SUPPORT ? NUM_SIGNALS_GPS_L5 : 0)
#define PLATFORM_SIGNAL_COUNT_GPS_AUX \
  (CODE_GPS_AUX_SUPPORT ? NUM_SIGNALS_GPS_L1CA : 0)

#define PLATFORM_SIGNAL_COUNT_SBAS_L1CA \
  (CODE_SBAS_L1CA_SUPPORT ? NUM_SIGNALS_SBAS_L1CA : 0)

#define PLATFORM_SIGNAL_COUNT_GLO_L1OF \
  (CODE_GLO_L1OF_SUPPORT ? NUM_SIGNALS_GLO_L1OF : 0)
#define PLATFORM_SIGNAL_COUNT_GLO_L2OF \
  (CODE_GLO_L2OF_SUPPORT ? NUM_SIGNALS_GLO_L2OF : 0)

#define PLATFORM_SIGNAL_COUNT_BDS2_B1 \
  (CODE_BDS2_B1_SUPPORT ? NUM_SIGNALS_BDS2_B1 : 0)
#define PLATFORM_SIGNAL_COUNT_BDS2_B2 \
  (CODE_BDS2_B2_SUPPORT ? NUM_SIGNALS_BDS2_B2 : 0)
#define PLATFORM_SIGNAL_COUNT_BDS3_B5 \
  (CODE_BDS3_B5_SUPPORT ? NUM_SIGNALS_BDS3_B5 : 0)

#define PLATFORM_SIGNAL_COUNT_QZS_L1CA \
  (CODE_QZSS_L1CA_SUPPORT ? NUM_SIGNALS_QZS_L1 : 0)
#define PLATFORM_SIGNAL_COUNT_QZS_L2C \
  (CODE_QZSS_L2C_SUPPORT ? NUM_SIGNALS_QZS_L2C : 0)
#define PLATFORM_SIGNAL_COUNT_QZS_L5 \
  (CODE_QZSS_L5_SUPPORT ? NUM_SIGNALS_QZS_L5 : 0)

#define PLATFORM_SIGNAL_COUNT_GAL_E1 \
  (CODE_GAL_E1_SUPPORT ? NUM_SIGNALS_GAL_E1 : 0)
#define PLATFORM_SIGNAL_COUNT_GAL_E5 \
  (CODE_GAL_E5_SUPPORT ? NUM_SIGNALS_GAL_E5 : 0)
#define PLATFORM_SIGNAL_COUNT_GAL_E7 \
  (CODE_GAL_E7_SUPPORT ? NUM_SIGNALS_GAL_E7 : 0)

/* Number of GLO frequencies supported
 * on the current hardware platform. */
#define PLATFORM_FREQ_COUNT_GLO_L1OF \
  (CODE_GLO_L1OF_SUPPORT ? NUM_FREQ_GLO_L1OF : 0)
#define PLATFORM_FREQ_COUNT_GLO_L2OF \
  (CODE_GLO_L2OF_SUPPORT ? NUM_FREQ_GLO_L2OF : 0)

/* Number of signals on each constellation supported
 * on the current hardware platform. */
#define PLATFORM_SIGNAL_COUNT_GPS                                   \
  (PLATFORM_SIGNAL_COUNT_GPS_L1CA + PLATFORM_SIGNAL_COUNT_GPS_L2C + \
   PLATFORM_SIGNAL_COUNT_GPS_L5 + PLATFORM_SIGNAL_COUNT_GPS_AUX)

#define PLATFORM_SIGNAL_COUNT_SBAS (PLATFORM_SIGNAL_COUNT_SBAS_L1CA)

#define PLATFORM_SIGNAL_COUNT_GLO \
  (PLATFORM_SIGNAL_COUNT_GLO_L1OF + PLATFORM_SIGNAL_COUNT_GLO_L2OF)

#define PLATFORM_SIGNAL_COUNT_BDS2                                 \
  (PLATFORM_SIGNAL_COUNT_BDS2_B1 + PLATFORM_SIGNAL_COUNT_BDS2_B2 + \
   PLATFORM_SIGNAL_COUNT_BDS3_B5)

#define PLATFORM_SIGNAL_COUNT_QZS                                   \
  (PLATFORM_SIGNAL_COUNT_QZS_L1CA + PLATFORM_SIGNAL_COUNT_QZS_L2C + \
   PLATFORM_SIGNAL_COUNT_QZS_L5)

#define PLATFORM_SIGNAL_COUNT_GAL                                \
  (PLATFORM_SIGNAL_COUNT_GAL_E1 + PLATFORM_SIGNAL_COUNT_GAL_E7 + \
   PLATFORM_SIGNAL_COUNT_GAL_E5)

/* Total number of GLO frequencies supported
 * on the current hardware platform. */
#define PLATFORM_FREQ_COUNT_GLO \
  (PLATFORM_FREQ_COUNT_GLO_L1OF + PLATFORM_FREQ_COUNT_GLO_L2OF)

/* Total number of signal supported
 * on the current hardware platform. */
#define PLATFORM_SIGNAL_COUNT                               \
  (PLATFORM_SIGNAL_COUNT_GPS + PLATFORM_SIGNAL_COUNT_SBAS + \
   PLATFORM_SIGNAL_COUNT_GLO + PLATFORM_SIGNAL_COUNT_BDS2 + \
   PLATFORM_SIGNAL_COUNT_QZS + PLATFORM_SIGNAL_COUNT_GAL)

/* Total number of simultaneous acquisition / tracking resources supported
 * on the current hardware platform. */
#define PLATFORM_ACQ_TRACK_COUNT                            \
  (PLATFORM_SIGNAL_COUNT_GPS + PLATFORM_SIGNAL_COUNT_SBAS + \
   PLATFORM_FREQ_COUNT_GLO + PLATFORM_SIGNAL_COUNT_BDS2 +   \
   PLATFORM_SIGNAL_COUNT_QZS + PLATFORM_SIGNAL_COUNT_GAL)

/** GNSS signal identifier for internal ME processing.
 *
 *  All signals except GLO are encoded similarly with gnss_signal_t
 *  GLO signals have their sat field represent the frequency slot FCN,
 *  sat range 1 - 14 (which is FCN -7..6 shifted by 8).
 */
typedef struct {
  u16 sat;
  code_t code;
} me_gnss_signal_t;

/* \} */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void signal_db_init(void);
gnss_signal_t sid_from_global_index(u16 global_index);
me_gnss_signal_t mesid_from_global_index(u16 me_global_index);
gnss_signal_t sid_from_constellation_index(constellation_t constellation,
                                           u16 constellation_index);
u16 mesid_to_global_index(me_gnss_signal_t mesid);
u16 sid_to_constellation_index(gnss_signal_t sid);
bool sid_supported(gnss_signal_t sid);
bool code_supported(code_t code);

float code_to_tcxo_doppler_min(code_t code);
float code_to_tcxo_doppler_max(code_t code);

gnss_signal_t sv_index_to_sid(u16 sv_index);
u16 sid_to_sv_index(gnss_signal_t sid);
double mesid_to_carr_fcn_hz(me_gnss_signal_t mesid);

constellation_t mesid_to_constellation(me_gnss_signal_t mesid);
int mesid_compare(me_gnss_signal_t a, me_gnss_signal_t b);
bool mesid_is_equal(me_gnss_signal_t a, me_gnss_signal_t b);
me_gnss_signal_t construct_mesid(code_t code, u16 sat);
gnss_signal_t mesid2sid(me_gnss_signal_t mesid, u16 glo_slot_id);
int mesid_to_string(char *s, int n, me_gnss_signal_t mesid);
bool mesid_valid(me_gnss_signal_t mesid);
me_gnss_signal_t mesid_from_code_index(code_t code, u16 me_code_index);
double mesid_to_carr_freq(me_gnss_signal_t mesid);
double mesid_to_carr_to_code(me_gnss_signal_t mesid);
u16 mesid_to_code_index(me_gnss_signal_t mesid);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_SIGNAL_H */
