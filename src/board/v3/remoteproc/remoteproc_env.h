/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_REMOTEPROC_ENV_H
#define SWIFTNAV_REMOTEPROC_ENV_H

typedef void (*remoteproc_env_irq_callback_t)(void);

void remoteproc_env_irq_callback_set(remoteproc_env_irq_callback_t callback);
void remoteproc_env_irq_process(void);

#endif /* SWIFTNAV_REMOTEPROC_ENV_H */
