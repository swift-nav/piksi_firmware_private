/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <libswiftnav/signal.h>

#include "glo_map.h"

/* GLO to FCN look up table, index 0 -- SV 1, index 27 -- SV 28 */
static u8 glo_sv_id_fcn_map[NUM_SATS_GLO] = { GLO_FCN_UNKNOWN };

/** The function maps GLO orbital slot and frequency slot
 *
 * @param[in] mesid ME signal identifier
 * @param[in] glo_slot_id GLO orbital slot
 * @return GLO SID
 */
gnss_signal_t glo_map_set_slot_id(me_gnss_signal_t mesid, u16 glo_slot_id)
{
  assert(is_glo_sid(mesid));
  assert(glo_slot_id_is_valid(glo_slot_id));
  assert(glo_fcn_is_valid(mesid.sat));

  gnss_signal_t sid = construct_sid(mesid.code, glo_slot_id);

  glo_sv_id_fcn_map[glo_slot_id - 1] = mesid.sat;

  return sid;
}

/** The function returns GLO frequency slot corresponds to the GLO SV ID
 *
 * @param[in] sid Signal identifier
 * @return GLO frequency slot corresponds to the GLO SV ID (1..14)
 */
u16 glo_map_get_fcn(gnss_signal_t sid)
{
  assert(is_glo_sid(construct_mesid(sid.code, sid.sat)));
  u16 fcn = (u16)glo_sv_id_fcn_map[sid.sat - 1];
  assert(glo_fcn_is_valid(fcn) && "GLO SV ID and frequency slot were not mapped");

  return fcn;
}

/** The function clears mapping between GLO SV ID and GLO FCN
 *
 * @param glo_slot_id GLO orbital slot
 */
void glo_map_clear_slot_id(u16 glo_slot_id)
{
  assert(glo_slot_id_is_valid(glo_slot_id));
  glo_sv_id_fcn_map[glo_slot_id - 1] = GLO_FCN_UNKNOWN;
}
