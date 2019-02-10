/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef SWIFTNAV_TASK_GENERATOR_API_H
#define SWIFTNAV_TASK_GENERATOR_API_H

#include "search_manager_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void tg_fill_task(acq_job_t *job);
void tg_check_uncertainty_change(acq_job_t *job);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* SWIFTNAV_TASK_GENERATOR_API_H */
