/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_SBP_SETTINGS_CLIENT_H_
#define STARLING_SBP_SETTINGS_CLIENT_H_

#include "sbp_duplex_link.h"

#include <libsettings/settings.h>
#include <stddef.h>

typedef struct SbpSettingsClient SbpSettingsClient;

SbpSettingsClient *sbp_settings_client_create(const SbpDuplexLink *sbp_link);

int sbp_settings_client_register_enum(SbpSettingsClient *client,
                                      const char *const enum_names[],
                                      settings_type_t *type);

int sbp_settings_client_register(SbpSettingsClient *client,
                                 const char *section,
                                 const char *name,
                                 void *var,
                                 size_t var_len,
                                 settings_type_t type,
                                 settings_notify_fn notify,
                                 void *notify_context);

#endif
