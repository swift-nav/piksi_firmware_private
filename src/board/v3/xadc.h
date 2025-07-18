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

#ifndef SWIFTNAV_XADC_H
#define SWIFTNAV_XADC_H

#include <stdbool.h>
#include <swiftnav/common.h>

void xadc_init(void);

float xadc_vin_get(void);
float xadc_vccint_get(void);
float xadc_vccaux_get(void);

float xadc_die_temp_get(void);
bool xadc_die_temp_warning(void);
bool xadc_die_temp_critical(void);

#endif /* SWIFTNAV_XADC_H */
