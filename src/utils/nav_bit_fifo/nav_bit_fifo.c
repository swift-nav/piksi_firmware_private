/*
 * Copyright (C) 2011-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "nav_bit_fifo/nav_bit_fifo.h"
#include "main.h"

/** Return navigation data bit FIFO size for code_t
 * \param code code_t to retrieve the size for
 * \return The FIFO size (variable so all system have 400 ms max latency)
 */
static u8 code_to_nav_bit_fifo_size(const code_t code) {
  if ((CODE_GPS_L1CA == code) || (CODE_GPS_L2CM == code) ||
      (CODE_GPS_L5I == code) || (CODE_QZS_L1CA == code) ||
      (CODE_QZS_L2CM == code) || (CODE_QZS_L5I == code)) {
    return 20;
  } else if ((CODE_GLO_L1OF == code) || (CODE_GLO_L2OF == code)) {
    return 40;
  } else if (CODE_SBAS_L1CA == code) {
    return 200;
  } else if ((CODE_BDS2_B1 == code) || (CODE_BDS2_B2 == code)) {
    return 20;
  } else if ((CODE_GAL_E1B == code) || (CODE_GAL_E7I == code)) {
    return 100;
  } else if ((CODE_GAL_E5I == code)) {
    return 20;
  }
  log_error("unknown code %d in code_to_nav_bit_fifo_size()", code);
  return 255;
}

/** Return occupied navigation data bit FIFO size for a given read index
 * \param  fifo object pointer
 * \param  rd The read index the FIFO length is computed for
 * \return Number of bits in FIFO.
 */
u8 nav_bit_fifo_length_for_rd_index(const nav_bit_fifo_t *fifo, const u8 rd) {
  const u8 size = fifo->size;
  const u8 wr = fifo->write_index;

  if (nav_bit_fifo_full(fifo)) {
    return fifo->size;
  }
  return (wr >= rd) ? (wr - rd) : ((size - rd) + wr);
}

/** Initialize a nav_bit_fifo_t struct.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param code        code of the signal for which the FIFO is
 *
 */
void nav_bit_fifo_init(nav_bit_fifo_t *fifo, const code_t code) {
  assert(NULL != fifo);
  assert(code_valid(code));

  fifo->read_index = 0;
  fifo->write_index = 0;
  fifo->size = code_to_nav_bit_fifo_size(code);
  fifo->length = 0;
}

/** Determine if a nav bit FIFO is full.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 *
 * \return true if the nav bit FIFO is full, false otherwise.
 */
bool nav_bit_fifo_full(const nav_bit_fifo_t *fifo) {
  return (fifo->length == fifo->size);
}

/** Write data to the nav bit FIFO.
 *
 * \note This function should only be called internally by the tracking thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Element to write to the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
bool nav_bit_fifo_write(nav_bit_fifo_t *fifo, const nav_bit_t *element) {
  if (fifo->length < fifo->size) {
    fifo->elements[fifo->write_index] = (*element);
    fifo->write_index = (fifo->write_index + 1) % (fifo->size);
    fifo->length++;
    return true;
  }

  return false;
}

/** Read pending data from the nav bit FIFO.
 *
 * \note This function should only be called externally by the decoder thread.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 * \param element     Output element read from the FIFO.
 *
 * \return true if element was read, false otherwise.
 */
bool nav_bit_fifo_read(nav_bit_fifo_t *fifo, nav_bit_t *element) {
  if (fifo->length > 0) {
    (*element) = fifo->elements[fifo->read_index];
    fifo->read_index = (fifo->read_index + 1) % (fifo->size);
    fifo->length--;
    return true;
  }

  return false;
}
