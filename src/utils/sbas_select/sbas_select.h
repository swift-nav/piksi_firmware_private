/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact:Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SBAS_SELECT_H
#define SBAS_SELECT_H

#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/signal.h>

#include "position/position.h"

#ifdef __cplusplus
extern "C" {
#endif

sbas_system_t sbas_select_provider(const last_good_fix_t *lgf);
u32 sbas_select_prn_mask(sbas_system_t sbas);

#ifdef __cplusplus
}
#endif
#endif /* SBAS_SELECT_H */
