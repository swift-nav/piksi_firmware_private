/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Perttu Salmela <psalmela@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <string.h>
#include "search_manager_api.h"

/* Compile unit test in src/reacq directory with:
 gcc  -Wall -std=c99 search_manager_unittest.c search_manager.c\
 ../../libswiftnav/src/signal.c \
 -I.. -I../../libswiftnav/include -I ../../src/board/v3/ \
 -I../../ChibiOS/os/rt/include/ -I../../ChibiOS/os/rt/templates -o sm_unittest
*/

/** Unit test input data type */
struct test_case_t {
  u32 now_ms;
  u32 health_mask;
  u32 vis_mask, known_mask, track_mask;
  u32 lgf_stamp_ms;
  u32 deep_mask, fallback_mask; /**< Expected results */
};

/** Current unit test case */
static struct test_case_t *test_case;

/** Unit test cases */
static struct test_case_t test_cases[] = {
  /* No SV tracked, health does not effect */
  {0,0x0, 0x0, 0x0, 0x0, 0,   0xffffffff, 0x0}, 
  {10, 0xffffffff, 0x0, 0x0, 0x0, 0,   0xffffffff,0x0},/* All healthy
                                                          not tracked */
  {10, 0xffffffff, 0x0, 0x0, 0xffffffff, 0,   0x0, 0x0},/* All healthy all
                                                           tracked */
  {20, 0x0, 0x0, 0x0, 0xffffffff, 0,   0x0, 0x0},/* All unhealthy all tracked */
  {30, 0x1, 0x1, 0x1, 0x0, 0,   0xffffffff, 0x0},/* 1 healthy, not tracked, visible */
  {30, 0x1, 0x0, 0x1, 0x0, 0,   0xfffffffe, 0x0},/* 1 healthy, not tracked,
						    invisible */
  {30, 0x1, 0x0, 0x0, 0x0, 0,   0xffffffff, 0x0},/* 1 healthy, not tracked, unknown */
  /* Timeouts */
  /* 1 healthy, not tracked, visible */
  {ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS+1, 0x1, 0x1, 0x1, 0x0, 0,
   0xffffffff,0xffffffff},
  /* 1 healthy,  not tracked,invisible */
  {ACQ_LGF_TIMEOUT_INVIS_MS+1, 0x1, 0x0, 0x1, 0x0, 0,
   0xfffffffe,0xffffffff},
  /* 1 healthy, not tracked, unknown */
  {ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS+1, 0x1, 0x0, 0x0, 0x0, 0,
   0xffffffff,0xffffffff}
};

/** Get SV visibility flags
 *
 * \param[in] sid GNSS signal identifier
 * \param[out] visible set if SV is visible
 * \param[out] known set if SV is known visible or known invisible
 */
void sm_get_visibility_flags(gnss_signal_t sid,
			     bool *visible, bool *known)
{
  if (0 != (test_case->vis_mask & (1 << sid_to_code_index(sid)))) {
    *visible = true;
  } else {
    *visible = false;
  }
  if (0 != (test_case->known_mask & (1 << sid_to_code_index(sid)))) {
    *known = true;
  } else {
    *known = false;
  }
}
/** Check if SV is healthy
 *
 * \param sid GNSS signal identifier
 *
 * \return true is SV is healthy, false otherwise
 */
bool sm_is_healthy(gnss_signal_t sid)
{
  if (0 != (test_case->health_mask & (1 << sid_to_code_index(sid)))) {
    return true;
  }
  return false;
}

/** Check if SV is tracked
 *
 * \param sid SV identifier
 *
 * \return true is SV is tracked, false otherwise
 */
bool sid_is_tracked(gnss_signal_t sid)
{
  if (0 != (test_case->track_mask & (1 << sid_to_code_index(sid)))) {
    return true;
  }
  return false;
}
/** Get current HW time in milliseconds
 *
 * \return HW time in milliseconds
 */
u64 timing_getms(void)
{
  return (u64)test_case->now_ms;
}

/** Get HW time of the last good fix (LGF)
 *
 * \param[out] lgf_stamp time of LGF (ms)
 * \return true lgf_stamp is valid, false otherwise
 */
bool sm_lgf_stamp(u64 *lgf_stamp)
{
  *lgf_stamp = (u64)test_case->lgf_stamp_ms; 
  return true;
}

/** Test program checking search manager operation
 *
 * \param argc Unused
 * \param argv Unused
 *
 * \return 1 on failure, 0 othersiwe
 */
int main(int argc, char **argv)
{
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  int test_ix;

  sm_init(data);

  for(test_ix = 0;
      test_ix < sizeof(test_cases)/sizeof(test_cases[0]);
      test_ix++) {
    u32 gps_run_mask[ACQ_NUM_JOB_TYPES];
    acq_job_types_e type;
    u32 i;

    test_case = &test_cases[test_ix];

    printf("now=%u health=0x%x vis=0x%x knw=0x%x trk=0x%x lgf=%u\n",
           test_case->now_ms, test_case->health_mask, test_case->vis_mask,
           test_case->known_mask, test_case->track_mask,
           test_case->lgf_stamp_ms);

    sm_run(data);

    /* Fill bit masks of jobs which are flagged to run */
    memset(gps_run_mask, 0, sizeof(gps_run_mask));
    for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
      for (i = 0; i < ACQ_NUM_SVS; i++) {
        if (data->jobs[type][i].needs_to_run) {
          if (CODE_GPS_L1CA == data->jobs[type][i].sid.code) {
            gps_run_mask[type] |=
              1 << (data->jobs[type][i].sid.sat - GPS_FIRST_PRN);
          }
        }
      }
    }
    printf("Search gps deep=0x%08x fallback=0x%08x",
           gps_run_mask[ACQ_JOB_DEEP_SEARCH],
           gps_run_mask[ACQ_JOB_FALLBACK_SEARCH]);
    if (gps_run_mask[ACQ_JOB_DEEP_SEARCH] != test_case->deep_mask ||
        gps_run_mask[ACQ_JOB_FALLBACK_SEARCH] != test_case->fallback_mask) {
      printf("\tFAIL\n");
      return 1;
    } else {
      printf("\tOK\n");
    }
  } /* for */

  return 0;
}
