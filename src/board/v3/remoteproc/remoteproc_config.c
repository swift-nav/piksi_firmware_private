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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#include <openamp/open_amp.h>
#pragma GCC diagnostic pop

#include "remoteproc_config.h"
#include "gic.h"

extern struct hil_platform_ops proc_ops;

struct remote_resource_table __attribute__((section (".resource_table")))
resource_table = {
  .version = 1,
  .num = NUM_TABLE_ENTRIES,
  .reserved = {0, 0},
  .offset = {
    offsetof(struct remote_resource_table, elf_cout),
    offsetof(struct remote_resource_table, rpmsg_vdev)
  },
  .elf_cout = {
    .type = RSC_CARVEOUT,
    .da = ELF_START,
    .pa = ELF_START,
    .len = ELF_SIZE,
    .flags = 0,
    .reserved = 0,
    .name = "ELF_COUT"
  },
  .rpmsg_vdev = {
    .type = RSC_VDEV,
    .id = VIRTIO_ID_RPMSG,
    .notifyid = 0,
    .dfeatures = RPMSG_IPU_C0_FEATURES,
    .gfeatures = 0,
    .config_len = 0,
    .status = 0,
    .num_of_vrings = NUM_VRINGS,
    .reserved = {0, 0}
  },
  .rpmsg_vring0 = {
    .da = 0xffffffff,
    .align = VRING_ALIGN,
    .num = VRING_SIZE,
    .notifyid = VRING0_IRQ,
    .reserved = 0
  },
  .rpmsg_vring1 = {
    .da = 0xffffffff,
    .align = VRING_ALIGN,
    .num = VRING_SIZE,
    .notifyid = VRING1_IRQ,
    .reserved = 0
  }
};

const struct hil_proc hil_proc = {
  .cpu_id = MASTER_CPU_ID,
  .sh_buff = {
    .start_addr = 0,
    .size = 0,
    .flags = 0
  },
  .vdev = {
    .num_vrings = 0,
    .dfeatures = 0,
    .gfeatures = 0,
    .vring_info[0] = {
      .vq = NULL,
      .phy_addr = NULL,
      .num_descs = 0,
      .align = 0,
      .intr_info = {
        .vect_id = VRING0_IRQ,
        .priority = VRING0_IRQ_PRIO,
        .trigger_type = IRQ_SENSITIVITY_EDGE,
        .data = NULL
      },
    },
    .vring_info[1] = {
      .vq = NULL,
      .phy_addr = NULL,
      .num_descs = 0,
      .align = 0,
      .intr_info = {
        .vect_id = VRING1_IRQ,
        .priority = VRING1_IRQ_PRIO,
        .trigger_type = IRQ_SENSITIVITY_EDGE,
        .data = NULL
      },
    },
  },
  .num_chnls = NUM_CHANNELS,
  .chnls[0] = {
    .name = CHANNEL0_NAME
  },
  .ops = &proc_ops,
  .attr = 0,
  .cpu_bitmask = 0,
  .slock = NULL
};
