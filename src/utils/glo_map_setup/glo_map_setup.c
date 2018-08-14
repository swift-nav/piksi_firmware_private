/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ch.h>
#include <libswiftnav/glo_map.h>

/* GLO to FCN look up table, index 0 unused, index 1 -- SV 1, index 28 -- SV 28
 */
static u8 glo_sv_id_fcn_map[NUM_GLO_MAP_INDICES] = {GLO_FCN_UNKNOWN};

static MUTEX_DECL(glo_map_mutex);

static void glo_map_lock(void) { chMtxLock(&glo_map_mutex); }

static void glo_map_unlock(void) { chMtxUnlock(&glo_map_mutex); }

void glo_map_setup(void) {
  glo_map_init(glo_map_lock, glo_map_unlock, glo_sv_id_fcn_map);
}
