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

#include "sbp_fileio.h"

#include <alloca.h>
#include <assert.h>
#include <ch.h>
#include <inttypes.h>
#include <libsbp/file_io.h>
#include <stdio.h>
#include <string.h>
#include <swiftnav/logging.h>
#include <swiftnav/memcpy_s.h>

#include "sbp.h"
#include "sbp_utils.h"

#define SBP_FILEIO_TIMEOUT MS2ST(5000)
#define SBP_FILEIO_TRIES 5

#define SBP_FILEIO_WRITE_FILENAME_LEN_MAX 128u

#define BREAK_ON_FAIL(res)                                                   \
  do {                                                                       \
    if (0 != (res)) {                                                        \
      log_error("%s: fn %s  offset %" PRIu32 "  buf %" PRIuPTR               \
                "  size %" PRIu32 "  chunksize %" PRIu32 "  total %" PRIu32, \
                __FUNCTION__,                                                \
                filename,                                                    \
                offset,                                                      \
                (uintptr_t)buf,                                              \
                (u32)size,                                                   \
                (u32)chunksize,                                              \
                (u32)total);                                                 \
      total = -1;                                                            \
      break;                                                                 \
    }                                                                        \
  } while (0)

static u32 write_seq_next(void) {
  static u32 seq = 0;
  return seq++;
}

static u32 read_seq_next(void) {
  static u32 seq = 0;
  return seq++;
}

typedef struct sbp_fileio_closure_s {
  u32 seq;
  binary_semaphore_t sem;
  u8 msg[SBP_FRAMING_MAX_PAYLOAD_SIZE];
  u8 len;
} sbp_fileio_closure_t;

static void sbp_fileio_callback(u16 sender_id,
                                u8 len,
                                u8 *msg_raw,
                                void *context) {
  (void)sender_id;

  assert(NULL != context);
  sbp_fileio_closure_t *closure = context;
  u32 seq = (*(u32 *)msg_raw);

  if (seq == closure->seq) {
    MEMCPY_S(closure->msg, SBP_FRAMING_MAX_PAYLOAD_SIZE, msg_raw, len);
    closure->len = len;
    chBSemSignal(&closure->sem);
  } else {
    log_error("sbp_fileio_callback()  sender %5u  msg->seq %" PRIu32
              " closure->seq %" PRIu32 " len %" PRIu8,
              sender_id,
              seq,
              closure->seq,
              len);
  }
}

void sbp_fileio_remove(const char *fn) {
  sbp_send_msg(SBP_MSG_FILEIO_REMOVE, strlen(fn), (u8 *)fn);
}

static int sbp_fileio_send(sbp_fileio_closure_t *closure,
                           u16 msg_id,
                           u8 size,
                           u8 *buf) {
  u8 tries = SBP_FILEIO_TRIES;

  do {
    s8 ret = sbp_send_msg(msg_id, size, buf);
    if (ret < 0) {
      log_error("sbp_send_msg(): error %d, retrying", ret);
      continue;
    }

    if (chBSemWaitTimeout(&closure->sem, SBP_FILEIO_TIMEOUT) == MSG_OK) {
      return 0;
    }
  } while (--tries > 0);

  return -1;
}

ssize_t sbp_fileio_write(const char *filename,
                         u32 offset,
                         const u8 *buf,
                         size_t size) {
  /* Check if typecast is safe */
  assert((ssize_t)size >= 0l);

  const size_t filename_len = strlen(filename) + 1;
  assert(filename_len <= SBP_FILEIO_WRITE_FILENAME_LEN_MAX);
  const size_t filename_offset = offsetof(msg_fileio_write_req_t, filename);

  const size_t data_offset = filename_offset + filename_len;
  const size_t data_size = SBP_FRAMING_MAX_PAYLOAD_SIZE - data_offset;

  msg_fileio_write_req_t *msg = alloca(SBP_FRAMING_MAX_PAYLOAD_SIZE);
  strncpy((char *)msg + filename_offset, filename, filename_len);

  sbp_fileio_closure_t closure = {0};
  chBSemObjectInit(&closure.sem, true);

  sbp_msg_callbacks_node_t node = {0};
  sbp_register_cbk_with_closure(
      SBP_MSG_FILEIO_WRITE_RESP, sbp_fileio_callback, &node, &closure);

  ssize_t total = 0;
  while (total < (ssize_t)size) {
    msg->sequence = closure.seq = write_seq_next();
    msg->offset = offset + total;
    size_t chunksize = MIN(data_size, size - total);

    MEMCPY_S((char *)msg + data_offset, data_size, buf + total, chunksize);

    int res = sbp_fileio_send(
        &closure, SBP_MSG_FILEIO_WRITE_REQ, data_offset + chunksize, (u8 *)msg);

    BREAK_ON_FAIL(res);

    total += chunksize;
  }

  sbp_remove_cbk(&node);

  return total;
}

ssize_t sbp_fileio_read(const char *filename,
                        u32 offset,
                        u8 *buf,
                        size_t size) {
  /* Check if typecast is safe */
  assert((ssize_t)size >= 0l);

  const size_t filename_len = strlen(filename) + 1;
  const size_t filename_offset = offsetof(msg_fileio_read_req_t, filename);
  assert(filename_len <= SBP_FRAMING_MAX_PAYLOAD_SIZE - filename_offset);

  msg_fileio_read_req_t *msg = alloca(SBP_FRAMING_MAX_PAYLOAD_SIZE);
  strncpy((char *)msg + filename_offset, filename, filename_len);

  sbp_fileio_closure_t closure = {0};
  chBSemObjectInit(&closure.sem, true);

  sbp_msg_callbacks_node_t node = {0};
  sbp_register_cbk_with_closure(
      SBP_MSG_FILEIO_READ_RESP, sbp_fileio_callback, &node, &closure);

  ssize_t total = 0;
  while (total < (ssize_t)size) {
    msg->sequence = closure.seq = read_seq_next();
    msg->offset = offset + total;
    size_t chunksize = msg->chunk_size =
        MIN(SBP_FRAMING_MAX_PAYLOAD_SIZE, size - total);

    int res = sbp_fileio_send(&closure,
                              SBP_MSG_FILEIO_READ_REQ,
                              filename_offset + filename_len,
                              (u8 *)msg);

    BREAK_ON_FAIL(res);

    const msg_fileio_read_resp_t *resp = (msg_fileio_read_resp_t *)closure.msg;
    const size_t contents_len =
        closure.len - offsetof(msg_fileio_read_resp_t, contents);

    if (0 == contents_len) {
      /* File is exhausted */
      break;
    }

    MEMCPY_S(buf + total, size - total, resp->contents, contents_len);
    total += contents_len;
  }

  sbp_remove_cbk(&node);

  return total;
}
