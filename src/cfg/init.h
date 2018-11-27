/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_INIT_H
#define SWIFTNAV_INIT_H

void pre_init(void);
void nap_init(void);
void init(void);

u16 sender_id_get(void);
u8 hw_revision_string_get(char *hw_revision_string);
u8 hw_version_string_get(char *hw_version_string);
u8 nap_version_string_get(char *nap_version_string);
u8 nap_date_string_get(char *nap_date_string);
u8 uuid_string_get(char *uuid_string);
u8 mfg_id_string_get(char *mfg_id_string, size_t size);
u8 mac_address_string_get(char *mac_string);

#endif
