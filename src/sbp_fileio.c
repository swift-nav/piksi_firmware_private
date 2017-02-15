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

#include <stdio.h>
#include <string.h>
#include <alloca.h>

#include <ch.h>

#include <libsbp/file_io.h>
#include <libswiftnav/logging.h>

#include "sbp.h"
#include "sbp_fileio.h"
#include "sbp_utils.h"

#define SBP_FILEIO_TIMEOUT 5000
#define SBP_FILEIO_TRIES   5

static u8 next_seq(void)
{
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

struct sbp_fileio_closure {
  u8 seq;
  binary_semaphore_t sem;
  u8 msg[256];
  u8 len;
};

static void sbp_fileio_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  struct sbp_fileio_closure *closure = context;
  (void)sender_id;

  if (((msg_fileio_write_resp_t*)msg)->sequence == closure->seq) {
    memcpy(closure->msg, msg, len);
    closure->len = len;
    chBSemSignal(&closure->sem);
  }
}

void sbp_fileio_remove(const char *fn)
{
  sbp_send_msg(SBP_MSG_FILEIO_REMOVE, strlen(fn), (u8*)fn);
}

ssize_t sbp_fileio_write(const char *filename, off_t offset, const u8 *buf, size_t size)
{
  size_t s = 0;
  u8 payload_offset = sizeof(msg_fileio_write_req_t) + strlen(filename) + 1;
  ssize_t chunksize = 255 - payload_offset;
  if (chunksize < 0)
    return -1;
  struct sbp_fileio_closure closure;
  msg_fileio_write_req_t *msg = alloca(256);
  chBSemObjectInit(&closure.sem, true);

  sbp_msg_callbacks_node_t node;
  sbp_register_cbk_with_closure(SBP_MSG_FILEIO_WRITE_RESP,
                                sbp_fileio_callback, &node, &closure);

  while (s < size) {
    msg->sequence = closure.seq = next_seq();
    msg->offset = offset + s;
    strcpy(msg->filename, filename);
    chunksize = MIN(chunksize, (ssize_t)(size - s));
    memcpy((u8*)msg + payload_offset,
           (u8*)buf + s, chunksize);

    u8 tries = 0;
    bool success = false;
    do {
      sbp_send_msg(SBP_MSG_FILEIO_WRITE_REQ, payload_offset + chunksize,
                   (u8*)msg);
      if (chBSemWaitTimeout(&closure.sem, SBP_FILEIO_TIMEOUT) == MSG_OK) {
        success = true;
        break;
      }
    } while (++tries < SBP_FILEIO_TRIES);

    if (!success) {
      s = -1;
      break;
    }
    s += chunksize;
  }

  sbp_remove_cbk(&node);

  return s;
}

ssize_t sbp_fileio_read(const char *filename, off_t offset, u8 *buf, size_t size)
{
  size_t s = 0;
  struct sbp_fileio_closure closure;
  msg_fileio_read_req_t *msg = alloca(256);
  chBSemObjectInit(&closure.sem, true);

  sbp_msg_callbacks_node_t node;
  sbp_register_cbk_with_closure(SBP_MSG_FILEIO_READ_RESP,
                                sbp_fileio_callback, &node, &closure);

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
                   (u8*)msg);
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
    if (chunksize == 0)
      break;

    memcpy(buf + s, closure.msg + 4, chunksize);
    s += chunksize;
  }

  sbp_remove_cbk(&node);

  return s;
}
