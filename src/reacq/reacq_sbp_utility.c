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

#include "reacq_sbp_utility.h"

#include <string.h>

#include "board/nap/nap_common.h"
#include "sbp/sbp_utils.h"
#include "swiftnav/glo_map.h"
#include "swiftnav/memcpy_s.h"
#include "timing/timing.h"

/** Size of reacquisition-related SBP buffer */
#define REACQ_SBP_BUFF_SIZE ((u8)(255 / sizeof(acq_sv_profile_t)))
/** SBP sending period in sec */
#define REACQ_SBP_PERIOD 1

static acq_sv_profile_t reacq_sbp_buffer[REACQ_SBP_BUFF_SIZE];
static u8 amount = 0; /* how many data chunks stored in buffer */
static u64 last_time_s = 0;

static void reacq_sbp_send(void) {
  sbp_send_msg(SBP_MSG_ACQ_SV_PROFILE,
               sizeof(acq_sv_profile_t) * amount,
               (u8 *)&reacq_sbp_buffer);
  amount = 0;
}

/**
 * The function initialize timer for reacq SBP message
 */
void reacq_sbp_init(void) {
  last_time_s = nap_timing_count() * RX_DT_NOMINAL;
  amount = 0;
}

/**
 * The function process reacquisition-related sbp message: store data into
 * buffer and send SBP when buffer is full or timer expired. Timer period is
 * #REACQ_SBP_PERIOD.
 * The function is called periodically from reacq manager.
 * \param[in] profile pointer to acquisition profile for a specific SV, if NULL
 *            no data stored in buffer, the function checks timer only.
 *
 * \return none
 */
static void reacq_sbp_data_process(const acq_sv_profile_t *profile) {
  u64 time_s = nap_timing_count() * RX_DT_NOMINAL; /* get current time */
  bool expired = ((time_s - last_time_s) >= REACQ_SBP_PERIOD);
  if ((amount == REACQ_SBP_BUFF_SIZE) || ((amount > 0) && expired)) {
    reacq_sbp_send();
    last_time_s = time_s;
  }

  if (profile != NULL) {
    assert(amount < REACQ_SBP_BUFF_SIZE);
    /* put new data to buffer */
    MEMCPY_S(&reacq_sbp_buffer[amount],
             sizeof(acq_sv_profile_t),
             profile,
             sizeof(acq_sv_profile_t));
    amount++;
  }
}

/** Populate acq_sv_profile message and send it out
 *
 * \param job job data which is filled in message
 * \param acq_result acquisition results
 * \param peak_found true if job found peak and acq_result contains
 *        peak details, otherwise false
 *
 * \return none
 */
void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found) {
  acq_sv_profile_t prof;
  /* In Phase 1, task 0 covers full job search space range.
     If there were more tasks, they all together would cover job search
     space range */
  const acq_task_search_params_t *acq_params = &job->task_data;

  prof.job_type = 0;
  prof.status = peak_found;
  prof.cn0 = (u16)(10 * acq_result->cn0);
  prof.int_time = acq_params->integration_time_ms;
  gnss_signal_t sid;
  if (IS_GLO(job->mesid)) {
    sid = construct_sid(job->mesid.code, GLO_ORBIT_SLOT_UNKNOWN);
  } else {
    sid = construct_sid(job->mesid.code, job->mesid.sat);
  }
  prof.sid = sid_to_sbp(sid);
  prof.bin_width = acq_params->freq_bin_size_hz;
  prof.timestamp = (u32)job->stop_time_ms;
  prof.time_spent = (u32)1000 * (job->stop_time_ms - job->start_time_ms);
  prof.cf_min = (s32)acq_params->doppler_min_hz;
  prof.cf_max = (s32)acq_params->doppler_max_hz;
  prof.cf = (s32)acq_result->df_hz;
  prof.cp = (u32)acq_result->cp;

  reacq_sbp_data_process(&prof);
}
