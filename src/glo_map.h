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

#ifndef SRC_GLO_MAP_H_
#define SRC_GLO_MAP_H_

#include <libswiftnav/signal.h>

gnss_signal_t glo_map_set_slot_id(me_gnss_signal_t mesid, u16 glo_slot_id);
u16 glo_map_get_fcn(gnss_signal_t sid);
void glo_map_clear_slot_id(u16 glo_slot_id);

#endif /* SRC_GLO_MAP_H_ */
