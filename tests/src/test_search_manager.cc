/*
 * Copyright (C) 2016-2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
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
#include "gtest/gtest.h"
#include "manage.h"
#include "search_manager_api.h"

//#define DBGOUT

extern u32 stubs_now_ms;

/** Current unit test case */
test_case_t *test_case;

/** Unit test cases */
test_case_t test_cases[] = {
    /* No SV tracked */
    {0, 0x0, 0x0, 0x0, 0, 0x0, 0x0},
    {ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS + 10,
     0x1,
     0x1,
     0x0,
     ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS + 9,
     0x1,
     0xffffffff}, /* not tracked, visible */
    {ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS + 20,
     0x0,
     0x1,
     0x0,
     ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS + 9,
     0x0,
     0xfffffffe}, /* not tracked, invisible, known */
    {ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS + 30,
     0x0,
     0x0,
     0x0,
     ACQ_FALLBACK_SEARCH_TIMEOUT_VIS_AND_UNKNOWN_MS + 9,
     0x0,
     0xffffffff}, /* not tracked, unknown */
    /* Timeouts */
    /* not tracked, visible */
    {ACQ_LGF_TIMEOUT_VIS_AND_UNKNOWN_MS + 1, 0x1, 0x1, 0x0, 0, 0x1, 0x0}};

/** Test program checking search manager operation
 *
 * \param argc Unused
 * \param argv Unused
 *
 * \return 1 on failure, 0 othersiwe
 */
TEST(search_manager_test, test_search_manager) {
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  u16 test_ix;

  sm_init(data);

  for (test_ix = 0; test_ix < sizeof(test_cases) / sizeof(test_cases[0]);
       test_ix++) {
    u32 gps_run_mask[ACQ_NUM_JOB_TYPES];
    u8 type;
    u32 i;

    test_case = &test_cases[test_ix];
    stubs_now_ms = test_case->now_ms;
#ifdef DBGOUT
    printf("now=%u vis=0x%x knw=0x%x trk=0x%x lgf=%u\n",
           test_case->now_ms,
           test_case->vis_mask,
           test_case->known_mask,
           test_case->track_mask,
           test_case->lgf_stamp_ms);
#endif

    data->constellation = CONSTELLATION_GPS;
    sm_run(data);

    /* Fill bit masks of jobs which are flagged to run */
    memset(gps_run_mask, 0, sizeof(gps_run_mask));
    for (type = (u8)ACQ_JOB_DEEP_SEARCH; type < (u8)ACQ_NUM_JOB_TYPES; type++) {
      for (i = 0; i < ACQ_NUM_SVS; i++) {
        if (data->jobs_gps[type][i].needs_to_run) {
          if (CODE_GPS_L1CA == data->jobs_gps[type][i].sid.code) {
            gps_run_mask[type] |=
                1 << (data->jobs_gps[type][i].sid.sat - GPS_FIRST_PRN);
          }
        }
      }
    }
#ifdef DBGOUT
    printf(
        "Search gps deep=0x%08x fallback=0x%08x, expected::deep=0x%08x, "
        "expected::fallback=0x%08x\n",
        gps_run_mask[ACQ_JOB_DEEP_SEARCH],
        gps_run_mask[ACQ_JOB_FALLBACK_SEARCH],
        test_case->deep_mask,
        test_case->fallback_mask);
#endif
    EXPECT_TRUE(gps_run_mask[ACQ_JOB_DEEP_SEARCH] == test_case->deep_mask &&
                gps_run_mask[ACQ_JOB_FALLBACK_SEARCH] ==
                    test_case->fallback_mask);
  } /* for */
}
