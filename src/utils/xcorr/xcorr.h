/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef XCORR_H
#define XCORR_H

#include <track/track_common.h>
#include <track/tracker.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void tracker_xcorr_update(tracker_t *tracker, tp_tracker_config_t *config);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* XCORR_H */
