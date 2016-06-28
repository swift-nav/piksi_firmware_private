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

#include <openamp/open_amp.h>

#include <libswiftnav/logging.h>
#include <libswiftnav/common.h>

#include "remoteproc.h"
#include "remoteproc_config.h"

static struct rpmsg_channel *app_rp_chnl = 0;
static struct rpmsg_endpoint *rp_ept;
static struct remote_proc *proc = NULL;
static struct rsc_table_info rsc_info;
extern struct remote_resource_table resource_table;

static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl);
static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl);
static void rpmsg_read_cb(struct rpmsg_channel *rpmsg_channel,
                          void *data, int len, void *priv, unsigned long src);

void remoteproc_setup(void)
{
  rsc_info.rsc_tab = (struct resource_table *)&resource_table;
  rsc_info.size = sizeof(resource_table);

  int status = remoteproc_resource_init(&rsc_info, rpmsg_channel_created,
                                        rpmsg_channel_deleted, rpmsg_read_cb,
                                        &proc);

  assert(status == 0);
}

static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl)
{
  app_rp_chnl = rp_chnl;
  rp_ept = rpmsg_create_ept(rp_chnl, rpmsg_read_cb, RPMSG_NULL,
                            RPMSG_ADDR_ANY);
}

static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl)
{
  (void)rp_chnl;
}

static void rpmsg_read_cb(struct rpmsg_channel *rpmsg_channel,
                          void *data, int len, void *priv, unsigned long src)
{
  (void)rpmsg_channel;
  (void)data;
  (void)len;
  (void)priv;
  (void)src;
}
