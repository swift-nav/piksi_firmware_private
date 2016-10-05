/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Dmitry Tatarinov <dmitry.tatarinov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SRC_REACQ_REACQ_SBP_UTILITY_H_
#define SRC_REACQ_REACQ_SBP_UTILITY_H_

#include <sbp.h>

void reacq_sbp_init(void);
void reacq_sbp_data_process(const acq_sv_profile_t *profile);

#endif /* SRC_REACQ_REACQ_SBP_UTILITY_H_ */

