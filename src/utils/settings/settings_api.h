/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_SETTINGS_H
#define SWIFTNAV_SETTINGS_H

#include <stdbool.h>

#include <libsettings/settings.h>

#include <swiftnav/common.h>

struct setting {
  const char *section;
  const char *name;
  void *addr;
  int len;
  settings_notify_fn notify;
  void *notify_ctx;
};

#define SETTING_WATCH(section, name, var, type, notify)            \
  do {                                                             \
    static struct setting setting = {                              \
        (section), (name), &(var), sizeof(var), (notify), (NULL)}; \
    settings_api_watch(&(setting), (type));                        \
  } while (0)

#define SETTING_NOTIFY_CTX(section, name, var, type, notify, ctx) \
  do {                                                            \
    static struct setting setting = {                             \
        (section), (name), &(var), sizeof(var), (notify), (ctx)}; \
    settings_api_register(&(setting), (type));                    \
  } while (0)

#define SETTING_NOTIFY(section, name, var, type, notify)           \
  do {                                                             \
    static struct setting setting = {                              \
        (section), (name), &(var), sizeof(var), (notify), (NULL)}; \
    settings_api_register(&(setting), (type));                     \
  } while (0)

#define SETTING(section, name, var, type)                        \
  do {                                                           \
    static struct setting setting = {                            \
        (section), (name), &(var), sizeof(var), (NULL), (NULL)}; \
    settings_api_register(&(setting), (type));                   \
  } while (0)

#define READ_ONLY_PARAMETER(section, name, var, type)            \
  do {                                                           \
    static struct setting setting = {                            \
        (section), (name), &(var), sizeof(var), (NULL), (NULL)}; \
    settings_api_register_readonly(&(setting), type);            \
  } while (0)

void settings_api_setup(void);

int settings_api_register_enum(const char *const enumnames[],
                               settings_type_t *type);

int settings_api_register(struct setting *s, settings_type_t type);
int settings_api_register_readonly(struct setting *s, settings_type_t type);
int settings_api_watch(struct setting *s, settings_type_t type);

#endif /* SWIFTNAV_SETTINGS_H */
