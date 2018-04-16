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

#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>

#include "position/position.h"

typedef enum sbas_e {
  SBAS_UNKNOWN = -1,
  SBAS_WAAS = 0,
  SBAS_EGNOS,
  SBAS_GAGAN,
  SBAS_MSAS,
  SBAS_COUNT
} sbas_type_t;

#ifdef __cplusplus
extern "C" {
#endif

sbas_type_t sbas_select_provider(const last_good_fix_t *lgf);
u32 sbas_select_prn_mask(sbas_type_t sbas);

#ifdef __cplusplus
}
#endif
#endif /* SBAS_SELECT_H */
