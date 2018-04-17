/*
 * Copyright (C) 2010, 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_NAV_MSG_GAL_H
#define SWIFTNAV_NAV_MSG_GAL_H

#include <libswiftnav/almanac.h>
#include <libswiftnav/common.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/ionosphere.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Minimum GAL valid subframe number */
#define GAL_INAV_SUBFR_MIN 1
/** Maximum GAL valid subframe number */
#define GAL_INAV_SUBFR_MAX 24
#define GAL_INAV_SUBFR_CNT (GAL_INAV_SUBFR_MAX - GAL_INAV_SUBFR_MIN + 1)

/** I/NAV Number of pages in a subframe */
#define GAL_INAV_PAGES_SUBFR 15

/** Number of 32 bit words of one Galileo I/NAV page */
#define GAL_INAV_PAGE_WORDS 8

/** GST week offset to GPS */
#define GAL_WEEK_TO_GPS_WEEK 1024

/**
 * Galileo message decoder object.
 *
 * The object for decoding Galileo symbols
 *
 * \sa nav_msg_init
 * \sa nav_msg_update
 * \sa subframe_ready
 * \sa process_subframe
 */
typedef struct {
  u8 prn;
  /**< Decoder buffer (256 bits) */
  u32 subframe_bits[GAL_INAV_PAGE_WORDS];
  /**< Received bit counter */
  u16 bit_index;
} nav_msg_gal_t;

void gal_inav_msg_init(nav_msg_gal_t *n, u8 prn);
void gal_inav_msg_clear_decoded(nav_msg_gal_t *n);
bool gal_inav_msg_update(nav_msg_gal_t *n, bool bit_val);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_NAV_MSG_GAL_H */
