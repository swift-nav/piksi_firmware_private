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

#define NAV_BIT_FIFO_INDEX_MASK ((NAV_BIT_FIFO_SIZE)-1)
#define NAV_BIT_FIFO_LENGTH(p_fifo) \
  (NAV_BIT_FIFO_INDEX_DIFF((p_fifo)->write_index, (p_fifo)->read_index))

/** Initialize a nav_bit_fifo_t struct.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 */
void nav_bit_fifo_init(nav_bit_fifo_t *fifo) {
  fifo->read_index = 0;
  fifo->write_index = 0;
}

/** Determine if a nav bit FIFO is full.
 *
 * \param fifo        nav_bit_fifo_t struct to use.
 *
 * \return true if the nav bit FIFO is full, false otherwise.
 */
bool nav_bit_fifo_full(nav_bit_fifo_t *fifo) {
  return (NAV_BIT_FIFO_LENGTH(fifo) == NAV_BIT_FIFO_SIZE);
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
bool nav_bit_fifo_write(nav_bit_fifo_t *fifo,
                        const nav_bit_fifo_element_t *element) {
  if (NAV_BIT_FIFO_LENGTH(fifo) < NAV_BIT_FIFO_SIZE) {
    fifo->elements[fifo->write_index & NAV_BIT_FIFO_INDEX_MASK] = (*element);
    fifo->write_index++;
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
bool nav_bit_fifo_read(nav_bit_fifo_t *fifo, nav_bit_fifo_element_t *element) {
  if (NAV_BIT_FIFO_LENGTH(fifo) > 0) {
    (*element) = fifo->elements[fifo->read_index & NAV_BIT_FIFO_INDEX_MASK];
    fifo->read_index++;
    return true;
  }

  return false;
}
