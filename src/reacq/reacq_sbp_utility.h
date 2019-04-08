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

#ifndef SRC_REACQ_REACQ_SBP_UTILITY_H_
#define SRC_REACQ_REACQ_SBP_UTILITY_H_

#include "sbp/sbp.h"
#include "search_manager_api.h"
#include "soft_macq/soft_macq_main.h"

void reacq_sbp_init(void);
void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found);

#endif /* SRC_REACQ_REACQ_SBP_UTILITY_H_ */
