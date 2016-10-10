/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <string.h>

#include <libswiftnav/common.h>

#include "fifo.h"

#define COMPILER_BARRIER() asm volatile ("" : : : "memory")

#define INDEX_MASK(p_fifo) ((p_fifo)->buffer_size - 1)
#define LENGTH(p_fifo) \
          ((fifo_size_t)((p_fifo)->write_index - (p_fifo)->read_index))
#define SPACE(p_fifo) \
          ((fifo_size_t)((p_fifo)->buffer_size - LENGTH(p_fifo)))

typedef u16 rec_size_t;

/*
 * This file implements a lock-free single-producer, single-consumer FIFO
 * using a circular buffer.
 */

static void read_index_advance(fifo_t *fifo, fifo_size_t length)
{
  if (length > 0) {
    /* Atomic write of read_index */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->read_index += length;
  }
}

static void write_index_advance(fifo_t *fifo, fifo_size_t length)
{
  if (length > 0) {
    /* Atomic write of write_index */
    COMPILER_BARRIER(); /* Prevent compiler reordering */
    fifo->write_index += length;
  }
}

static void copy_out_offset(fifo_t *fifo, fifo_size_t offset,
                            u8 *buffer, fifo_size_t length)
{
  fifo_size_t read_index_masked =
      (fifo->read_index + offset) & INDEX_MASK(fifo);
  if (read_index_masked + length <= fifo->buffer_size) {
    /* One contiguous block */
    memcpy(buffer, &fifo->buffer[read_index_masked], length);
  } else {
    /* Two contiguous blocks */
    fifo_size_t copy_len_a = fifo->buffer_size - read_index_masked;
    memcpy(buffer, &fifo->buffer[read_index_masked], copy_len_a);
    memcpy(&buffer[copy_len_a], fifo->buffer, length - copy_len_a);
  }
}

static void copy_out(fifo_t *fifo, u8 *buffer, fifo_size_t length)
{
  copy_out_offset(fifo, 0, buffer, length);
}

static void copy_in_offset(fifo_t *fifo, fifo_size_t offset,
                           const u8 *buffer, fifo_size_t length)
{
  fifo_size_t write_index_masked =
      (fifo->write_index + offset) & INDEX_MASK(fifo);
  if (write_index_masked + length <= fifo->buffer_size) {
    /* One contiguous block */
    memcpy(&fifo->buffer[write_index_masked], buffer, length);
  } else {
    /* Tow contiguous blocks */
    fifo_size_t copy_len_a = fifo->buffer_size - write_index_masked;
    memcpy(&fifo->buffer[write_index_masked], buffer, copy_len_a);
    memcpy(fifo->buffer, &buffer[copy_len_a], length - copy_len_a);
  }
}

static void copy_in(fifo_t *fifo, const u8 *buffer, fifo_size_t length)
{
  copy_in_offset(fifo, 0, buffer, length);
}

static bool rec_size_get(fifo_t *fifo, fifo_size_t fifo_length,
                         rec_size_t *rec_size)
{
  if (fifo_length < sizeof(rec_size_t)) {
    return false;
  }

  copy_out(fifo, (u8 *)rec_size, sizeof(rec_size_t));
  assert(fifo_length >= sizeof(rec_size_t) + *rec_size);
  return true;
}

static bool rec_size_set(fifo_t *fifo, fifo_size_t fifo_space,
                         rec_size_t rec_size)
{
  /* Do not proceed if the full record will not fit */
  if (fifo_space < sizeof(rec_size_t) + rec_size) {
    return false;
  }

  copy_in(fifo, (u8 *)&rec_size, sizeof(rec_size_t));
  return true;
}

/** Initialize a FIFO.
 *
 * \param fifo        fifo_t struct to use.
 * \param mode        Mode to use. In standard mode, the FIFO is byte-oriented.
 *                    In record mode, data is containerized and read from the
 *                    FIFO in the same block sizes as it was written.
 * \param buffer      Buffer to use for the FIFO data. Must remain valid
 *                    while the FIFO is in use.
 * \param buffer_size Size of buffer. Must be a power of two.
 */
void fifo_init(fifo_t *fifo, fifo_mode_t mode,
               u8 *buffer, fifo_size_t buffer_size)
{
  /* Require buffer_size to be a power of two */
  assert((buffer_size & (buffer_size - 1)) == 0);
  assert((mode == FIFO_MODE_STANDARD) || (mode == FIFO_MODE_RECORD));

  fifo->mode = mode;
  fifo->read_index = 0;
  fifo->write_index = 0;
  fifo->buffer_size = buffer_size;
  fifo->buffer = buffer;
}

/** Get the length for a FIFO.
 *
 * \note If called from the consumer thread, the length is a lower bound.
 * If called from the producer thread, the length is an upper bound.
 *
 * \param fifo        fifo_t struct to use.
 *
 * \return Number of bytes that may be read from the FIFO.
 */
fifo_size_t fifo_length(fifo_t *fifo)
{
  fifo_size_t fifo_length = LENGTH(fifo);
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return fifo_length;
}

/** Get the space for a FIFO.
 *
 * \note If called from the consumer thread, the space is an upper bound.
 * If called from the producer thread, the space is a lower bound.
 *
 * \param fifo        fifo_t struct to use.
 *
 * \return Number of bytes that may be written to the FIFO.
 */
fifo_size_t fifo_space(fifo_t *fifo)
{
  fifo_size_t fifo_space = SPACE(fifo);
  COMPILER_BARRIER(); /* Prevent compiler reordering */
  return fifo_space;
}

/** Read data from a FIFO.
 *
 * \note This function should only be called from a single consumer thread.
 *
 * \param fifo        fifo_t struct to use.
 * \param buffer      Output buffer.
 * \param length      Maximum number of bytes to read.
 *
 * \return Number of bytes read from the FIFO.
 */
fifo_size_t fifo_read(fifo_t *fifo, u8 *buffer, fifo_size_t length)
{
  fifo_size_t read_length = fifo_peek(fifo, buffer, length);
  read_length = fifo_remove(fifo, read_length);
  return read_length;
}

/** Read data from a FIFO without removing it.
 *
 * \note This function should only be called from a single consumer thread.
 *
 * \param fifo        fifo_t struct to use.
 * \param buffer      Output buffer.
 * \param length      Maximum number of bytes to read.
 *
 * \return Number of bytes read from the FIFO.
 */
fifo_size_t fifo_peek(fifo_t *fifo, u8 *buffer, fifo_size_t length)
{
  /* Atomic read of write_index to get fifo_length */
  fifo_size_t fifo_length = LENGTH(fifo);
  COMPILER_BARRIER(); /* Prevent compiler reordering */

  switch (fifo->mode) {
    case FIFO_MODE_STANDARD: {
      fifo_size_t read_length = MIN(length, fifo_length);
      if (read_length > 0) {
        copy_out(fifo, buffer, read_length);
      }
      return read_length;
    }
    break;

    case FIFO_MODE_RECORD: {
      /* Read record size */
      rec_size_t rec_size;
      if (!rec_size_get(fifo, fifo_length, &rec_size)) {
        return 0;
      }

      /* Read record data */
      fifo_size_t read_length = MIN(length, rec_size);
      if (read_length > 0) {
        copy_out_offset(fifo, sizeof(rec_size_t), buffer, read_length);
      }

      return read_length;
    }
    break;

    default: {
      assert(!"Invalid FIFO mode");
    }
    break;
  }

  return 0;
}

/** Remove data from a FIFO.
 *
 * \note This function should only be called from a single consumer thread.
 *
 * \param fifo        fifo_t struct to use.
 * \param length      Maximum number of bytes to remove.
 *
 * \return Number of bytes removed from the FIFO.
 */
fifo_size_t fifo_remove(fifo_t *fifo, fifo_size_t length)
{
  /* Atomic read of write_index to get fifo_length */
  fifo_size_t fifo_length = LENGTH(fifo);
  COMPILER_BARRIER(); /* Prevent compiler reordering */

  switch (fifo->mode) {
    case FIFO_MODE_STANDARD: {
      fifo_size_t read_length = MIN(length, fifo_length);
      read_index_advance(fifo, read_length);
      return read_length;
    }
    break;

    case FIFO_MODE_RECORD: {
      /* Read record size */
      rec_size_t rec_size;
      if (!rec_size_get(fifo, fifo_length, &rec_size)) {
        return 0;
      }

      /* Always remove full record from the FIFO */
      read_index_advance(fifo, sizeof(rec_size_t) + rec_size);

      /* Report the record size as the number of bytes read */
      return rec_size;
    }
    break;

    default: {
      assert(!"Invalid FIFO mode");
    }
    break;
  }

  return 0;
}

/** Write data to a FIFO.
 *
 * \note This function should only be called from a single producer thread.
 *
 * \param fifo        fifo_t struct to use.
 * \param buffer      Input buffer.
 * \param length      Maximum number of bytes to write.
 *
 * \return Number of bytes written to the FIFO.
 */
fifo_size_t fifo_write(fifo_t *fifo, const u8 *buffer, fifo_size_t length)
{
  fifo_size_t write_length = fifo_poke(fifo, buffer, length);
  write_length = fifo_add(fifo, write_length);
  return write_length;
}

/** Write data to a FIFO without adding it.
 *
 * \note This function should only be called from a single producer thread.
 *
 * \param fifo        fifo_t struct to use.
 * \param buffer      Input buffer.
 * \param length      Maximum number of bytes to write.
 *
 * \return Number of bytes written to the FIFO.
 */
fifo_size_t fifo_poke(fifo_t *fifo, const u8 *buffer, fifo_size_t length)
{
  /* Atomic read of read_index to get fifo_space */
  fifo_size_t fifo_space = SPACE(fifo);
  COMPILER_BARRIER(); /* Prevent compiler reordering */

  switch (fifo->mode) {
    case FIFO_MODE_STANDARD: {
      fifo_size_t write_length = MIN(length, fifo_space);
      if (write_length > 0) {
        copy_in(fifo, buffer, write_length);
      }
      return write_length;
    }
    break;

    case FIFO_MODE_RECORD: {
      /* Write record size */
      rec_size_t rec_size = length;
      if (!rec_size_set(fifo, fifo_space, rec_size)) {
        return 0;
      }

      /* Write record data */
      if (rec_size > 0) {
        copy_in_offset(fifo, sizeof(rec_size_t), buffer, rec_size);
      }

      return rec_size;
    }
    break;

    default: {
      assert(!"Invalid FIFO mode");
    }
    break;
  }

  return 0;
}

/** Add data to a FIFO.
 *
 * \note This function should only be called from a single producer thread.
 *
 * \param fifo        fifo_t struct to use.
 * \param length      Maximum number of bytes to add.
 *
 * \return Number of bytes added to the FIFO.
 */
fifo_size_t fifo_add(fifo_t *fifo, fifo_size_t length)
{
  /* Atomic read of read_index to get fifo_space */
  fifo_size_t fifo_space = SPACE(fifo);
  COMPILER_BARRIER(); /* Prevent compiler reordering */

  switch (fifo->mode) {
    case FIFO_MODE_STANDARD: {
      fifo_size_t write_length = MIN(length, fifo_space);
      write_index_advance(fifo, write_length);
      return write_length;
    }
    break;

    case FIFO_MODE_RECORD: {
      /* Write record size */
      rec_size_t rec_size = length;
      if (!rec_size_set(fifo, fifo_space, rec_size)) {
        return 0;
      }

      write_index_advance(fifo, sizeof(rec_size_t) + rec_size);
      return rec_size;
    }
    break;

    default: {
      assert(!"Invalid FIFO mode");
    }
    break;
  }

  return 0;
}
