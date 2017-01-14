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

#ifndef SWIFTNAV_REMOTEPROC_CONFIG_H
#define SWIFTNAV_REMOTEPROC_CONFIG_H

#include <stddef.h>
#include <openamp/open_amp.h>

/* Resource table entries */
#define NUM_TABLE_ENTRIES           2
#define ELF_START                   0x1C000000
#define ELF_SIZE                    0x00800000
#define NUM_VRINGS                  2
#define VRING_ALIGN                 0x00100000
/* Number of buffers per vring. Must be a power of 2. Max = 256. */
#define VRING_SIZE                  256
#define VRING0_IRQ                  15
#define VRING1_IRQ                  14
#define VRING0_IRQ_PRIO             4
#define VRING1_IRQ_PRIO             4
#define RPMSG_IPU_C0_FEATURES       (1 << VIRTIO_RPMSG_F_NS)
#define MASTER_CPU_ID               0
#define REMOTE_CPU_ID               1
#define NUM_CHANNELS                1
#define CHANNEL0_NAME               "piksi"

/* Resource table for the given remote */
struct remote_resource_table {
  unsigned int version;
  unsigned int num;
  unsigned int reserved[2];
  unsigned int offset[NUM_TABLE_ENTRIES];
  /* text carveout entry */
  struct fw_rsc_carveout elf_cout;
  /* rpmsg vdev entry */
  struct fw_rsc_vdev rpmsg_vdev;
  struct fw_rsc_vdev_vring rpmsg_vring0;
  struct fw_rsc_vdev_vring rpmsg_vring1;
};

#endif /* SWIFTNAV_REMOTEPROC_CONFIG_H */
