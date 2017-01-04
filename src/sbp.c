/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <hal.h>
#include <ch.h>

#include <libsbp/sbp.h>
#include <libswiftnav/logging.h>

#include "peripherals/leds.h"
#include "error.h"
#include "sbp.h"
#include "sbp_utils.h"
#include "settings.h"
#include "main.h"
#include "timing.h"
#include "error.h"
#include "io_support.h"

/** \defgroup io Input/Output
 * Communications to and from host.
 * \{ */

/** \defgroup sbp Swift Binary Protocol
 * Send and receive messages using Swift Binary Protocol.
 * \{ */

u16 my_sender_id;

/* TODO: Make new message type containing only latency and obs period */
msg_uart_state_t corr_stats;

#define LATENCY_SMOOTHING 0.5
#define PERIOD_SMOOTHING 0.5
#define LOG_OBS_WINDOW_DURATION 3.0

double latency_count;
double latency_accum_ms;
double period_count;
double period_accum_ms;

systime_t last_obs_msg_ticks = 0;

sbp_state_t sbp_state;

static u8 sbp_buffer[264];
static u32 sbp_buffer_length;

static MUTEX_DECL(sbp_cb_mutex);

static WORKING_AREA_CCM(wa_sbp_thread, 7168);
static void sbp_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("SBP");

  corr_stats.latency.avg = -1;
  corr_stats.latency.lmin = 0;
  corr_stats.latency.lmax = 0;
  corr_stats.latency.current = -1;
  corr_stats.obs_period.avg = -1;
  corr_stats.obs_period.pmin = 0;
  corr_stats.obs_period.pmax = 0;
  corr_stats.obs_period.current = -1;

  while (TRUE) {
    chThdSleepMilliseconds(10);
    sbp_process_messages();

    DO_EVERY(100,
      if (latency_count > 0) {
        corr_stats.latency.avg = (s32) (latency_accum_ms / latency_count);
        corr_stats.obs_period.avg = (s32) (period_accum_ms / latency_count);
      }

      sbp_send_msg(SBP_MSG_UART_STATE, sizeof(msg_uart_state_t),
                   (u8*)&corr_stats);

      log_obs_latency_tick();
    );
  }
}

/** Setup the SBP interface.
 * Initializes state and starts the SBP message processing
 * thread.
 *
 * \param use_settings If 0 use default baud rate, else use baud rates in
 *                     flash settings
 */
void sbp_setup(void)
{
  sbp_state_init(&sbp_state);

  chThdCreateStatic(wa_sbp_thread, sizeof(wa_sbp_thread),
                    HIGHPRIO-22, sbp_thread, NULL);
}

void sbp_sender_id_set(u16 sender_id)
{
  my_sender_id = sender_id;
}

void sbp_register_cbk_with_closure(u16 msg_type, sbp_msg_callback_t cb,
                                   sbp_msg_callbacks_node_t *node,
                                   void *context)
{
  chMtxLock(&sbp_cb_mutex);
  sbp_register_callback(&sbp_state , msg_type, cb, context, node);
  chMtxUnlock(&sbp_cb_mutex);
}

void sbp_register_cbk(u16 msg_type, sbp_msg_callback_t cb,
                      sbp_msg_callbacks_node_t *node)
{
  sbp_register_cbk_with_closure(msg_type, cb, node, NULL);
}

void sbp_remove_cbk(sbp_msg_callbacks_node_t *node)
{
  chMtxLock(&sbp_cb_mutex);
  sbp_remove_callback(&sbp_state, node);
  chMtxUnlock(&sbp_cb_mutex);
}

static void sbp_buffer_reset(void)
{
  sbp_buffer_length = 0;
}

static u32 sbp_buffer_write(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 len = MIN(sizeof(sbp_buffer) - sbp_buffer_length, n);
  memcpy(&sbp_buffer[sbp_buffer_length], buff, len);
  sbp_buffer_length += len;
  return len;
}

/** Send an SBP message
 *
 * \param msg_type Message ID
 * \param len      Length of message data
 * \param buff     Pointer to message data array
 *
 * \return         Error code
 */
u32 sbp_send_msg(u16 msg_type, u8 len, u8 buff[])
{
  return sbp_send_msg_(msg_type, len, buff, my_sender_id);
}

u32 sbp_send_msg_(u16 msg_type, u8 len, u8 buff[], u16 sender_id)
{
  static MUTEX_DECL(send_mutex);
  chMtxLock(&send_mutex);

  u16 ret = 0;

  /* Write message into buffer */
  sbp_buffer_reset();
  ret |= sbp_send_message(&sbp_state, msg_type, sender_id,
                          len, buff, &sbp_buffer_write);

  /* TODO: Put back check for sender_id == 0 somewhere */
  io_support_write(SD_SBP, sbp_buffer, sbp_buffer_length);

  chMtxUnlock(&send_mutex);
  return ret;
}

static u32 sbp_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  return io_support_read_timeout(SD_SBP, buff, n, TIME_IMMEDIATE);
}

/** Process SBP messages received.
 * This function should be called periodically.
 */
void sbp_process_messages()
{
  s8 ret;

  chMtxLock(&sbp_cb_mutex);

  while (io_support_n_read(SD_SBP) > 0) {
    ret = sbp_process(&sbp_state, &sbp_read);
    if (ret == SBP_CRC_ERROR) {
      /* TODO: Expose this somehow */
    }
  }

  chMtxUnlock(&sbp_cb_mutex);
}

/** Directs printf's output to the SBP interface */
int _write(int file, char *ptr, int len)
{
  switch (file) {
  /* Direct stdout and stderr to MSG_PRINT */
  case 1:
  case 2:
    if (len > 255) len = 255;   /* Send maximum of 255 chars at a time */
    sbp_send_msg(SBP_MSG_PRINT_DEP, len, (u8 *)ptr);
    return len;

  default:
    errno = EIO;
    return -1;
  }
}

/** Directs log_ output to the SBP log message */
void log_(u8 level, const char *msg, ...)
{
  msg_log_t *log;
  va_list ap;
  char buf[SBP_FRAMING_MAX_PAYLOAD_SIZE];

  log = (msg_log_t *)buf;
  log->level = level;

  va_start(ap, msg);
  int n = vsnprintf(log->text, SBP_FRAMING_MAX_PAYLOAD_SIZE-sizeof(msg_log_t), msg, ap);
  va_end(ap);

  if (n < 0)
    return;

  sbp_send_msg(SBP_MSG_LOG, n+sizeof(msg_log_t), (u8 *)buf);
}

void detailed_log_(u8 level, const char *file_path, const int line_number,
                   const char *msg, ...)
{
  msg_log_t *log;
  va_list ap;
  char buf[SBP_FRAMING_MAX_PAYLOAD_SIZE];

  log = (msg_log_t *)buf;
  log->level = level;

  int n = 0;
  n += snprintf(&log->text[n], SBP_FRAMING_MAX_PAYLOAD_SIZE-sizeof(msg_log_t)-n,
                "(%s:%d) ", file_path, line_number);

  va_start(ap, msg);
  n += vsnprintf(&log->text[n], SBP_FRAMING_MAX_PAYLOAD_SIZE-sizeof(msg_log_t)-n, msg, ap);
  va_end(ap);

  if (n < 0)
    return;

  sbp_send_msg(SBP_MSG_LOG, n + sizeof(msg_log_t), (u8 *)buf);
}

void log_obs_latency(float latency_ms)
{
  systime_t now = chVTGetSystemTime();
  float obs_period_ms = 0;
  if (last_obs_msg_ticks != 0) {
    obs_period_ms = (now - last_obs_msg_ticks) / (double)CH_CFG_ST_FREQUENCY * 1000;
  }

  last_obs_msg_ticks = now;
  latency_accum_ms += (double) latency_ms;
  period_accum_ms += (double) obs_period_ms;
  latency_count += 1;

  corr_stats.latency.current = (s32) ((LATENCY_SMOOTHING * ((float)latency_ms)) +
    ((1 - LATENCY_SMOOTHING) * (float) (corr_stats.latency.current)));

  corr_stats.obs_period.current = (s32) ((PERIOD_SMOOTHING * ((float) (obs_period_ms)) +
    (1 - PERIOD_SMOOTHING) * (float) (corr_stats.obs_period.current)));

  /* Don't change the min and max latencies if we appear to have a zero latency
   * speed. */
  if (latency_ms <= 0 || (last_obs_msg_ticks != 0 && obs_period_ms == 0)) {
    log_warn("Incoherent observation reception: latency: %f, period: %f", latency_ms, obs_period_ms);
    return;
  }
  if (corr_stats.latency.lmin > latency_ms ||
      corr_stats.latency.lmin == 0) {
    corr_stats.latency.lmin = latency_ms;
  }
  if (corr_stats.latency.lmax < latency_ms) {
    corr_stats.latency.lmax = latency_ms;
  }
  if (corr_stats.obs_period.pmin > obs_period_ms ||
      corr_stats.obs_period.pmin == 0) {
    corr_stats.obs_period.pmin = obs_period_ms;
  }
  if (obs_period_ms > corr_stats.obs_period.pmax) {
    corr_stats.obs_period.pmax = obs_period_ms;
  }
}

void log_obs_latency_tick(void)
{
  double elapsed = chVTTimeElapsedSinceX(last_obs_msg_ticks) / (double)CH_CFG_ST_FREQUENCY;

  if (last_obs_msg_ticks == 0 || elapsed > LOG_OBS_WINDOW_DURATION) {
    corr_stats.latency.current = -1;
    corr_stats.obs_period.current = -1;
  }

}

/** \} */

/** \} */
