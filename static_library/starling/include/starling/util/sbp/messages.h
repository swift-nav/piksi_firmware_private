/**
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_UTIL_SBP_MESSAGES_H
#define STARLING_UTIL_SBP_MESSAGES_H

#include <libsbp/navigation.h>
#include <libsbp/observation.h>
#include <libsbp/orientation.h>
#include <libsbp/sbas.h>
#include <libsbp/system.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)
#define MSG_OBS_HEADER_MAX_SIZE MSG_OBS_HEADER_SEQ_MASK

typedef struct {
  union {
    msg_ephemeris_gps_t gps;
    msg_ephemeris_bds_t bds;
    msg_ephemeris_gal_t gal;
    msg_ephemeris_sbas_t sbas;
    msg_ephemeris_glo_t glo;
    msg_ephemeris_qzss_t qzss;
  };
} msg_ephemeris_t;

typedef struct {
  u16 msg_id;
  u16 size;
} msg_info_t;

typedef union {
  msg_almanac_gps_t gps;
  msg_almanac_glo_t glo;
} msg_almanac_t;

#ifdef __cplusplus
}
#endif

#endif  // STARLING_UTIL_SBP_MESSAGES_H
