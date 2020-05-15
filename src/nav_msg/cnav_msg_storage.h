/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_CNAV_MSG_STORAGE_H
#define LIBSWIFTNAV_CNAV_MSG_STORAGE_H

#include <swiftnav/signal.h>

#include "nav_msg/cnav_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

void cnav_msg_put(const cnav_msg_t *msg);
bool cnav_msg_get(gnss_signal_t sid, cnav_msg_type_t type, cnav_msg_t *msg);
void cnav_msg_clear(gnss_signal_t sid, bool skip_health_info);

#ifdef __cplusplus
}
#endif

#endif /* LIBSWIFTNAV_CNAV_MSG_STORAGE_H */
