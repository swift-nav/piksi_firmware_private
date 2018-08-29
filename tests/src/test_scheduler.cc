/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "gtest/gtest.h"
#include "manage.h"
#include "scheduler_api.h"
#include "soft_macq_main.h"

#define EXPECTED_COST_DELTA_MS 50
#define EXPECTED_SLEEP_TIMEOUT_MS 100

extern u32 stubs_now_ms;

/* Unit test data */
acq_jobs_state_t acq_all_jobs_state_data;

bool hw_has_run;   /** Set if acq_search is called */
u32 hw_code_index; /** Set to code index for which hw was run */

/** Test cost initialization
 *
 *  Test failure triggers assertion
 *
 * \return none
 */
TEST(scheduler_test, test_sch_cost_init) {
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  acq_job_t *init_job = &data->jobs[0][10];
  sm_init(data);
  data->constellation = CONSTELLATION_GPS;
  data->jobs[0][1].cost = 100;
  data->jobs[0][1].needs_to_run = true;
  data->jobs[0][1].state = ACQ_STATE_WAIT;
  data->jobs[0][3].cost = 70;
  data->jobs[0][3].needs_to_run = true;
  data->jobs[0][3].state = ACQ_STATE_WAIT;
  data->jobs[0][4].cost = 110;
  data->jobs[0][4].needs_to_run = true;
  data->jobs[0][4].state = ACQ_STATE_WAIT;
  data->jobs[1][1].cost = 101;
  data->jobs[1][1].needs_to_run = true;
  data->jobs[1][1].state = ACQ_STATE_WAIT;
  data->jobs[1][4].cost = 120;
  data->jobs[1][4].needs_to_run = true;
  data->jobs[1][4].state = ACQ_STATE_WAIT;
  data->jobs[1][5].cost = 200;
  data->jobs[1][5].needs_to_run = true;
  data->jobs[1][5].state = ACQ_STATE_IDLE;
  data->jobs[0][5].cost = 10;
  data->jobs[0][5].needs_to_run = true;
  data->jobs[0][5].state = ACQ_STATE_IDLE;

  /* Check min, max, avg, max_plus */
  init_job->cost_hint = ACQ_COST_MIN;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(70, init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(120, init_job->cost);

  init_job->cost_hint = ACQ_COST_AVG;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ((70 + 100 + 110 + 101 + 120) / 5, init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX_PLUS;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(MIN(init_job->cost, 120 + init_job->cost_delta), init_job->cost);

  /* Check that initialized job does not effect on itself */
  init_job->cost_hint = ACQ_COST_MIN;
  init_job->needs_to_run = true;
  init_job->cost = 5;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(70, init_job->cost);

  /* Check when there are no other jobs */
  sm_init(data);
  init_job->cost_hint = ACQ_COST_MIN;
  data->constellation = CONSTELLATION_GPS;

  sch_initialize_cost(init_job, data);
  EXPECT_EQ(0, init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(0, init_job->cost);

  init_job->cost_hint = ACQ_COST_AVG;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(0, init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX_PLUS;
  sch_initialize_cost(init_job, data);
  EXPECT_EQ(0 + init_job->cost_delta, init_job->cost);
}

/** Test job selection
 *
 *  Test failure triggers assertion
 *
 * \return none
 */
TEST(scheduler_test, test_sch_job_select) {
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  acq_job_t *sel;
  sm_init(data);
  data->constellation = CONSTELLATION_GPS;
  data->jobs[0][1].cost = 100;
  data->jobs[0][1].needs_to_run = true;
  data->jobs[0][1].state = ACQ_STATE_WAIT;
  data->jobs[1][1].cost = 90;
  data->jobs[1][1].needs_to_run = true;
  data->jobs[1][1].state = ACQ_STATE_WAIT;
  data->jobs[0][2].cost = 110;
  data->jobs[0][2].needs_to_run = true;
  data->jobs[0][2].state = ACQ_STATE_WAIT;
  data->jobs[1][2].cost = 80;
  data->jobs[1][2].needs_to_run = true;
  data->jobs[1][2].state = ACQ_STATE_WAIT;

  /* Find job with minimum cost */
  sel = sch_select_job(data);
  EXPECT_EQ(sel, &data->jobs[1][2]);

  /* Exclude if it should not run */
  data->jobs[1][2].state = ACQ_STATE_IDLE;
  data->jobs[1][2].needs_to_run = false;
  sel = sch_select_job(data);
  EXPECT_EQ(sel, &data->jobs[1][1]);

  /* Exclude if it should not run */
  data->jobs[1][2].state = ACQ_STATE_WAIT;
  data->jobs[1][2].needs_to_run = false;
  sel = sch_select_job(data);
  EXPECT_EQ(sel, &data->jobs[1][1]);

  /* Check enable running */
  data->jobs[1][2].needs_to_run = true;
  data->jobs[1][2].state = ACQ_STATE_IDLE;
  data->jobs[1][2].task_data.task_index = 255;
  data->jobs[1][2].task_data.task_index = 255;
  data->jobs[1][2].cost_hint = ACQ_COST_MIN;
  sel = sch_select_job(data);
  /* There are two jobs with minimum cost */
  EXPECT_TRUE(sel == &data->jobs[1][2] || sel == &data->jobs[1][1]);
  EXPECT_EQ(ACQ_STATE_WAIT, data->jobs[1][2].state);
  EXPECT_EQ(ACQ_UNINITIALIZED_TASKS, data->jobs[1][2].task_data.task_index);
}
/** Run scheduler and check that HW ran expected code_index
 *
 *  Test failure triggers assertion
 *
 * \parma run expect that hw run (true) / does not run (false)
 * \param code_index expected code_index for which hw is run
 *
 * \return none
 */
static void sch_expect_hw_run(bool run, u32 code_index) {
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  hw_has_run = false;
  sch_run(data);
  if (run) {
    EXPECT_NE(hw_has_run, false);
    EXPECT_EQ(code_index, hw_code_index);
  } else {
    EXPECT_EQ(hw_has_run, false);
  }
}

/** Test job scheduling
 *
 *  Test failure triggers assertion
 *
 * \return none
 */
TEST(scheduler_test, test_sch_job_scheduling) {
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  {
    /* Check that nothing is run */
    sm_init(data);
    data->constellation = CONSTELLATION_GPS;
    sch_run(data);
    sch_expect_hw_run(false, 0);
  }
  {
    /* Check that if nothing runs, time flies */
    stubs_now_ms = 0;
    sm_init(data);
    data->constellation = CONSTELLATION_GPS;
    sch_run(data);
    sch_run(data);
    EXPECT_EQ(2 * EXPECTED_SLEEP_TIMEOUT_MS, stubs_now_ms);
  }
  { /* Check that if peak is found it does not continue */
    sm_init(data);
    data->constellation = CONSTELLATION_GPS;
    data->jobs[0][20].needs_to_run = true;
    sch_expect_hw_run(true, 20);
    sch_expect_hw_run(false, 20);
  }
  { /* Check that if peak is not found, search continues */
    sm_init(data);
    data->constellation = CONSTELLATION_GPS;
    data->jobs[0][10].needs_to_run = true;
    sch_expect_hw_run(true, 10);
    sch_expect_hw_run(true, 10);
    sch_expect_hw_run(true, 10);
  }
  { /* Check scheduling order */
    sm_init(data);
    data->constellation = CONSTELLATION_GPS;
    data->jobs[0][10].needs_to_run = true;
    data->jobs[0][10].cost_hint = ACQ_COST_MIN;
    data->jobs[0][11].needs_to_run = true;
    data->jobs[0][11].cost_hint = ACQ_COST_MIN;
    data->jobs[0][12].needs_to_run = true;
    data->jobs[0][12].cost_hint = ACQ_COST_MIN;
    data->jobs[0][13].needs_to_run = true;
    data->jobs[0][13].cost_hint = ACQ_COST_MAX_PLUS;
    data->jobs[0][14].needs_to_run = true;
    data->jobs[0][14].cost_hint = ACQ_COST_MAX_PLUS;
    sch_expect_hw_run(true, 10);
    sch_expect_hw_run(true, 11);
    sch_expect_hw_run(true, 12);
    sch_expect_hw_run(true, 13);
    sch_expect_hw_run(true, 14);
  }
  { /* Check scheduling order. If peak is found, start
        immediately next job even if there is large initial
        cost difference. */
    sm_init(data);
    data->constellation = CONSTELLATION_GPS;
    data->jobs[0][20].needs_to_run = true;
    data->jobs[0][20].cost_hint = ACQ_COST_MIN;
    data->jobs[0][11].needs_to_run = true;
    data->jobs[0][11].cost_hint = ACQ_COST_MAX_PLUS;
    sch_expect_hw_run(true, 11);
    sch_expect_hw_run(true, 20);
  }
}
