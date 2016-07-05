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

#include <openamp/open_amp.h>

#include <assert.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/common.h>

#include <ch.h>
#include <hal.h>

#include "rpmsg.h"
#include "remoteproc_config.h"
#include "remoteproc_env.h"
#include "lib/fifo.h"

#define ENDPOINT_FIFO_SIZE 4096
#define RPMSG_BUFFER_SIZE_MAX 512

#define RPMSG_THD_PRIO (HIGHPRIO-21)
#define RPMSG_THD_STACK_SIZE 4096
#define RPMSG_THD_PERIOD_ms 10

static const u32 endpoint_addr_config[RPMSG_ENDPOINT__COUNT] = {
  [RPMSG_ENDPOINT_A] = 100,
  [RPMSG_ENDPOINT_B] = 101,
  [RPMSG_ENDPOINT_C] = 102,
};

typedef struct {
  struct rpmsg_endpoint *rpmsg_endpoint;
  u32 addr;
  fifo_t tx_fifo;
  fifo_t rx_fifo;
  u8 tx_buf[ENDPOINT_FIFO_SIZE];
  u8 rx_buf[ENDPOINT_FIFO_SIZE];
} endpoint_data_t;
static endpoint_data_t endpoint_data[RPMSG_ENDPOINT__COUNT];

static u32 rpmsg_buffer_size = 0;

extern struct remote_resource_table resource_table;
static struct remote_proc *remote_proc = NULL;
static struct rsc_table_info rsc_table_info = {
  .rsc_tab = (struct resource_table *)&resource_table,
  .size = sizeof(resource_table)
};

static BSEMAPHORE_DECL(rpmsg_thd_bsem, false);

static void remoteproc_env_irq_callback(void);

static void rpmsg_channel_created(struct rpmsg_channel *rpmsg_channel);
static void rpmsg_channel_destroyed(struct rpmsg_channel *rpmsg_channel);
static void rpmsg_default_rx(struct rpmsg_channel *rpmsg_channel, void *data,
                             int len, void *priv, unsigned long src);
static void rpmsg_endpoint_rx(struct rpmsg_channel *rpmsg_channel, void *data,
                              int len, void *priv, unsigned long src);

static THD_WORKING_AREA(wa_rpmsg_thread, RPMSG_THD_STACK_SIZE);
static THD_FUNCTION(rpmsg_thread, arg);

void rpmsg_setup(void)
{
  for (u32 i=0; i<RPMSG_ENDPOINT__COUNT; i++) {
    endpoint_data_t *d = &endpoint_data[i];
    d->rpmsg_endpoint = NULL;
    d->addr = endpoint_addr_config[i];
    fifo_init(&d->tx_fifo, d->tx_buf, sizeof(d->tx_buf));
    fifo_init(&d->rx_fifo, d->rx_buf, sizeof(d->rx_buf));
  }

  remoteproc_env_irq_callback_set(remoteproc_env_irq_callback);

  int status = remoteproc_resource_init(&rsc_table_info,
                                        rpmsg_channel_created,
                                        rpmsg_channel_destroyed,
                                        rpmsg_default_rx,
                                        &remote_proc);
  assert(status == 0);

  chThdCreateStatic(wa_rpmsg_thread, sizeof(wa_rpmsg_thread),
                    RPMSG_THD_PRIO, rpmsg_thread, NULL);
}

u32 rpmsg_rx_fifo_length(rpmsg_endpoint_t rpmsg_endpoint)
{
  return fifo_length(&endpoint_data[rpmsg_endpoint].rx_fifo);
}

u32 rpmsg_tx_fifo_space(rpmsg_endpoint_t rpmsg_endpoint)
{
  return fifo_space(&endpoint_data[rpmsg_endpoint].tx_fifo);
}

u32 rpmsg_rx_fifo_read(rpmsg_endpoint_t rpmsg_endpoint, u8 *buffer, u32 length)
{
  return fifo_read(&endpoint_data[rpmsg_endpoint].rx_fifo, buffer, length);
}

u32 rpmsg_tx_fifo_write(rpmsg_endpoint_t rpmsg_endpoint,
                        const u8 *buffer, u32 length)
{
  return fifo_write(&endpoint_data[rpmsg_endpoint].tx_fifo, buffer, length);
}

static void remoteproc_env_irq_callback(void)
{
  chSysLockFromISR();
  chBSemSignalI(&rpmsg_thd_bsem);
  chSysUnlockFromISR();
}

static void rpmsg_channel_created(struct rpmsg_channel *rpmsg_channel)
{
  for (u32 i=0; i<RPMSG_ENDPOINT__COUNT; i++) {
    endpoint_data_t *d = &endpoint_data[i];
    d->rpmsg_endpoint =
        rpmsg_create_ept(rpmsg_channel, rpmsg_endpoint_rx, d, d->addr);
  }

  rpmsg_buffer_size = rpmsg_get_buffer_size(rpmsg_channel);
}

static void rpmsg_channel_destroyed(struct rpmsg_channel *rpmsg_channel)
{
  (void)rpmsg_channel;

  for (u32 i=0; i<RPMSG_ENDPOINT__COUNT; i++) {
    endpoint_data_t *d = &endpoint_data[i];
    rpmsg_destroy_ept(d->rpmsg_endpoint);
    d->rpmsg_endpoint = NULL;
  }

  rpmsg_buffer_size = 0;
}

static void rpmsg_default_rx(struct rpmsg_channel *rpmsg_channel, void *data,
                             int len, void *priv, unsigned long src)
{
  (void)rpmsg_channel;
  (void)data;
  (void)len;
  (void)priv;
  (void)src;
}

static void rpmsg_endpoint_rx(struct rpmsg_channel *rpmsg_channel, void *data,
                              int len, void *priv, unsigned long src)
{
  (void)rpmsg_channel;
  (void)src;

  endpoint_data_t *d = (endpoint_data_t *)priv;
  fifo_write(&d->rx_fifo, data, len);
}

static THD_FUNCTION(rpmsg_thread, arg)
{
  (void)arg;
  chRegSetThreadName("rpmsg");

  while (1) {
    chBSemWaitTimeout(&rpmsg_thd_bsem, MS2ST(RPMSG_THD_PERIOD_ms));

    remoteproc_env_irq_process();

    for (u32 i=0; i<RPMSG_ENDPOINT__COUNT; i++) {
      endpoint_data_t *d = &endpoint_data[i];

      if (d->rpmsg_endpoint == NULL) {
        continue;
      }

      fifo_t *fifo = &d->tx_fifo;
      u32 length = fifo_length(fifo);

      while (length > 0) {
        u8 buffer[RPMSG_BUFFER_SIZE_MAX];
        u32 buffer_length = fifo_peek(fifo, buffer,
                                      MIN(rpmsg_buffer_size, sizeof(buffer)));

        if (buffer_length == 0) {
          break;
        }

        if (rpmsg_trysendto(d->rpmsg_endpoint->rp_chnl,
                            buffer, buffer_length, d->addr) != 0) {
          break;
        }

        fifo_remove(fifo, buffer_length);
        length -= buffer_length;
      }
    }
  }
}
