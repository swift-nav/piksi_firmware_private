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
#ifndef STARLING_EFILTER_H
#define STARLING_EFILTER_H

typedef bool (*sbas_has_corrections_t)(const gnss_signal_t *sid, u8 IODE);

void starling_efilter_set_sbas_cb(sbas_has_corrections_t sbas_has_corrections);
void starling_efilter_set_ephe(const ephemeris_t *e);
ephemeris_t starling_efilter_get_ephe(const gnss_signal_t *sid);

#endif /* #ifndef STARLING_EFILTER_H */
