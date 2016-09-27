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

#include <string.h>
#include <board/nap/nap_common.h>
#include "reacq_sbp_utility.h"
#include <timing.h>

/** Size of reacquisition-related SBP buffer */
#define REACQ_SBP_BUFF_SIZE ((u8)(255 / sizeof(acq_sv_profile_t)))
/** SBP sending period in sec */
#define REACQ_SBP_PERIOD 1

static acq_sv_profile_t reacq_sbp_buffer[REACQ_SBP_BUFF_SIZE];
static u8 amount = 0; /* how many data chunks stored in buffer */
static u64 count = 0;

static void reacq_sbp_send(void)
{
  sbp_send_msg(SBP_MSG_ACQ_SV_PROFILE,
               sizeof(acq_sv_profile_t) * amount,
               (u8 *)&reacq_sbp_buffer);
  amount = 0;
}

/**
 * The function initialize timer for reacq SBP message
 */
void reacq_sbp_init(void)
{
  count = nap_timing_count() * RX_DT_NOMINAL;
  amount = 0;
}

/**
 * The function process reacquisituon-related sbp message: store data into
 * buffer and send SBP when buffer is full or timer expired. Timer period is
 * #REACQ_SBP_PERIOD.
 * The function is called periodically from reacq manager.
 * \param[in] profile pointer to acquisition profile for a specific SV, if NULL
 *            no data stored in buffer, the function checks timer only.
 *
 * \return none
 */
void reacq_sbp_data_process(const acq_sv_profile_t *profile)
{
  u64 cnt = nap_timing_count() * RX_DT_NOMINAL; /* get current time */

  if ((cnt - count) >= REACQ_SBP_PERIOD && amount > 0) {
    /* timer expired and we have something to sent */
    reacq_sbp_send();
    count = cnt;
  }

  if (profile != NULL) {
    /* put new data to buffer */
    memcpy(&reacq_sbp_buffer[amount], profile, sizeof(acq_sv_profile_t));
    amount++; /* increase data counter */
    /* was this last data slot? */
    if (amount == REACQ_SBP_BUFF_SIZE) {
      reacq_sbp_send();
      count = cnt;
    }
  }
}

