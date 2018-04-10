/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact:Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SBAS_REACQ_H
#define SBAS_REACQ_H

#include <libswiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

void sbas_reacq_prioritize(const me_gnss_signal_t *mesid);
u32 sbas_reacq_get_priority_mask(u32 mask);

#ifdef __cplusplus
}
#endif

#endif /* SBAS_REACQ_H */
