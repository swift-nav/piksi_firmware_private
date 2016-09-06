/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
 *          Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "cnav_msg_storage.h"

/** Stub implementation for V2 board
 *
 * \param sid Signal ID
 * \param type message type to be retrieved
 * \param msg pointer to message to be retrieved
 *
 * \returns always false as V2 doesn't decode GPS L2CM
 */
bool cnav_msg_get(gnss_signal_t sid, cnav_msg_type_t type, cnav_msg_t *msg)
{
  (void)(sid);
  (void)(type);
  (void)(msg);

  return false;
}
