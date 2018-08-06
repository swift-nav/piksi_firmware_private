/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <alloca.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <ch.h>

#include <libsbp/file_io.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/memcpy_s.h>

#include "sbp.h"
#include "sbp_fileio.h"
#include "sbp_utils.h"

#define SBP_FILEIO_TIMEOUT MS2ST(5000)
#define SBP_FILEIO_TRIES 5

static u8 next_seq(void) {
  static MUTEX_DECL(seq_mtx);
  static u8 seq;
  u8 ret;

  if (seq == 0) {
    seq = rand();
  }

  chMtxLock(&seq_mtx);
  ret = seq++;
  chMtxUnlock(&seq_mtx);
  return ret;
}

#define SBP_FILEIO_MSG_LEN 256

struct sbp_fileio_closure {
  u8 seq;
  binary_semaphore_t sem;
  u8 msg[SBP_FILEIO_MSG_LEN];
  u8 len;
};

static void sbp_fileio_callback(u16 sender_id,
                                u8 len,
                                u8 msg_raw[],
                                void *context) {
  ASSERT(NULL != context);
  struct sbp_fileio_closure *closure = context;
  msg_fileio_write_resp_t *msg = (msg_fileio_write_resp_t *)msg_raw;
  (void)sender_id;

  if (msg->sequence == closure->seq) {
    MEMCPY_S(closure->msg, SBP_FILEIO_MSG_LEN, msg_raw, len);
    closure->len = len;
    chBSemSignal(&closure->sem);
  } else {
    log_error("sbp_fileio_callback()  sender %5u  msg->sequence %08" PRIx32
              " closure->seq %02" PRIx8 " len %3" PRIu8,
              sender_id,
              msg->sequence,
              closure->seq,
              len);
  }
}

void sbp_fileio_remove(const char *fn) {
  sbp_send_msg(SBP_MSG_FILEIO_REMOVE, strlen(fn), (u8 *)fn);
}

ssize_t sbp_fileio_write(const char *filename,
                         off_t offset,
                         const u8 *buf,
                         size_t size) {
  size_t s = 0;
  u8 payload_offset = sizeof(msg_fileio_write_req_t) + strlen(filename) + 1;
  ssize_t chunksize = 255 - payload_offset;
  struct sbp_fileio_closure closure;
  msg_fileio_write_req_t *msg = alloca(256);
  char dbg_filename[100];
  chBSemObjectInit(&closure.sem, true);
  u8 *msg_pt;
  s8 ret;

  strncpy(dbg_filename, filename, sizeof(dbg_filename));
  dbg_filename[sizeof(dbg_filename) - 1] = '\0';

  sbp_msg_callbacks_node_t node;
  sbp_register_cbk_with_closure(
      SBP_MSG_FILEIO_WRITE_RESP, sbp_fileio_callback, &node, &closure);

  while (s < size) {
    msg->sequence = closure.seq = next_seq();
    msg->offset = offset + s;
    strcpy(msg->filename, filename);
    msg_pt = (u8 *)msg;
    chunksize = MIN(chunksize, (ssize_t)(size - s));
    MEMCPY_S(msg_pt + payload_offset, 256 - payload_offset, buf + s, chunksize);

    u8 tries = 0;
    bool success = false;
    do {
      ret = sbp_send_msg(
          SBP_MSG_FILEIO_WRITE_REQ, payload_offset + chunksize, msg_pt);
      if (ret < 0) {
        log_error("sbp_send_msg(): error %d", ret);
      }

      if (chBSemWaitTimeout(&closure.sem, SBP_FILEIO_TIMEOUT) == MSG_OK) {
        success = true;
        break;
      }
    } while (++tries < SBP_FILEIO_TRIES);

    if (!success) {
      log_error("sbp_fileio_write(): fn %s  offset %" PRIi32
                " buf %p  size %zu  tries %" PRIu8 "  chunksize %zd   s %zu",
                dbg_filename,
                (int32_t)offset,
                buf,
                size,
                tries,
                chunksize,
                s);
      s = -1;
      break;
    }
    s += chunksize;
  }

  sbp_remove_cbk(&node);

  return s;
}

ssize_t sbp_fileio_read(const char *filename,
                        off_t offset,
                        u8 *buf,
                        size_t size) {
  size_t s = 0;
  struct sbp_fileio_closure closure;
  msg_fileio_read_req_t *msg = alloca(256);
  chBSemObjectInit(&closure.sem, true);

  sbp_msg_callbacks_node_t node;
  sbp_register_cbk_with_closure(
      SBP_MSG_FILEIO_READ_RESP, sbp_fileio_callback, &node, &closure);

  while (s < size) {
    msg->sequence = closure.seq = next_seq();
    msg->offset = offset + s;
    msg->chunk_size = MIN(255, size - s);
    strcpy(msg->filename, filename);

    u8 tries = 0;
    bool success = false;
    do {
      sbp_send_msg(SBP_MSG_FILEIO_READ_REQ,
                   sizeof(msg_fileio_read_req_t) + strlen(filename),
                   (u8 *)msg);
      if (chBSemWaitTimeout(&closure.sem, SBP_FILEIO_TIMEOUT) == MSG_OK) {
        success = true;
        break;
      }
    } while (++tries < SBP_FILEIO_TRIES);

    if (!success) {
      s = -1;
      break;
    }

    ssize_t chunksize = MIN(closure.len - 4, (ssize_t)(size - s));
    if (chunksize == 0) break;

    MEMCPY_S(buf + s, size - s, closure.msg + 4, chunksize);
    s += chunksize;
  }

  sbp_remove_cbk(&node);

  return s;
}
