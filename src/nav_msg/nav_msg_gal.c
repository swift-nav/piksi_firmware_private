/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libswiftnav/bits.h>
#include <libswiftnav/constants.h>
#include <libswiftnav/ionosphere.h>
#include <libswiftnav/logging.h>

#include "nav_msg/nav_msg_gal.h"
#include "timing/timing.h"

static void dw_1bit_pushr(u32 *words, u8 numel, bool bitval);

/**
 * Initializes Galileo message decoder.
 *
 * \param[in] n   GAL message decoder object
 * \param[in] prn Galileo PRN id
 */
void gal_inav_msg_init(nav_msg_gal_t *n, u8 prn) {
  /* Initialize the necessary parts of the nav message state structure. */
  memset(n, 0, sizeof(*n));
  n->prn = prn;
}

/**
 * Re-initializes Galileo message decoder.
 *
 * \param n Galileo message decoder object
 */
void gal_inav_msg_clear_decoded(nav_msg_gal_t *n) { (void)n; }

/** Navigation message decoding update.
 * Called once per nav bit interval. Performs the necessary steps to
 * store the nav bits and decode them.
 *
 * \param n Nav message decode state struct
 * \param bit_val State of the nav bit to process
 *
 * \return true if a new subframe started with this bit
 */
bool gal_inav_msg_update(nav_msg_gal_t *n, bool bit_val) {
  /* add new bit to buffer */
  dw_1bit_pushr(n->subframe_bits, GAL_INAV_PAGE_WORDS, bit_val);

  n->bit_index++;
  if (GAL_INAV_PAGE_WORDS * 32 == n->bit_index) {
    log_info("E%02u %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx",
             n->prn,
             n->subframe_bits[0],
             n->subframe_bits[1],
             n->subframe_bits[2],
             n->subframe_bits[3],
             n->subframe_bits[4],
             n->subframe_bits[5],
             n->subframe_bits[6],
             n->subframe_bits[7]);
    n->bit_index = 0;
  }

  /* data decoding not implemented yet, but soon! */
  return false;
}

/** Shifts bits properly in the bit array */
static void dw_1bit_pushr(u32 *words, u8 numel, bool bitval) {
  u8 k;

  assert(words);
  assert(numel);

  for (k = 0; k < (numel - 1U); k++) {
    words[k] <<= 1;
    words[k] |= (words[k + 1] >> 31) & 0x1;
  }
  words[k] <<= 1;
  words[k] |= (bitval & 0x1);
}
