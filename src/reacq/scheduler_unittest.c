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
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "scheduler_api.h"
#include <timing.h>
#include <manage.h>
#include <ndb.h>

/* Compile unit test in src/reacq directory with:
 gcc  -Wall -std=c99 scheduler_unittest.c scheduler.c task_generator.c \
 ../../libswiftnav/src/signal.c \
 -I../../libsbp/c/include -I../../libswiftnav/libfec/include \
 -I.. -I../../libswiftnav/include -I ../../src/board/v3 \
 -I../../libswiftnav/libfec/include -I../../ChibiOS/os/rt/include \
 -I../../ChibiOS/os/rt/templates -o sch_unittest

*/

void sch_initialize_cost(acq_job_t *init_job,
			 const acq_jobs_state_t *all_jobs_data);
acq_job_t *sch_select_job(acq_jobs_state_t *jobs_data);

#define EXPECTED_COST_DELTA_MS 50
#define EXPECTED_SLEEP_TIMEOUT_MS 1000

/* Unit test data */
acq_jobs_state_t acq_all_jobs_state_data;

static u32 now_ms; /** Current time (ms) */
static bool hw_has_run; /** Set if acq_search is called */
static u32 hw_code_index; /** Set to code index for which hw was run */

/* Stub functions for unit testing*/
void chThdSleep(systime_t time)
{
  now_ms += (u32)time;
}
u64 timing_getms(void)
{
  return (u64)now_ms;
}
u64 nap_timing_count(void)
{
  return timing_getms() / (RX_DT_NOMINAL * 1000.0);
}
bool acq_search(gnss_signal_t sid, float cf_min, float cf_max,
                float cf_bin_width, acq_result_t *acq_result)
{
  u32 i = sid_to_code_index(sid);
  (void)cf_min;
  (void)cf_max;
  (void)cf_bin_width;

  hw_has_run = true;
  hw_code_index = i;
  now_ms++;

  if (i <= 15) {
    return false;
  } 
  acq_result->cf = i * 100;
  acq_result->cn0 = 30.0f + (float)i;
  acq_result->cp = i * 10;
  return true;
}

u8 tracking_startup_request(const tracking_startup_params_t *startup_params)
{
  /* Remove from acquisition */
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  data->jobs[0][sid_to_code_index(startup_params->sid)].needs_to_run = false;
  data->jobs[1][sid_to_code_index(startup_params->sid)].needs_to_run = false;
  return 0;
}
void sch_send_acq_profile_msg(const acq_job_t *job,
                              const acq_result_t *acq_result,
                              bool peak_found)
{
  (void)job;
  (void)acq_result;
  (void)peak_found;
}
void sm_init(acq_jobs_state_t *data)
{
  memset(data, 0, sizeof(acq_jobs_state_t));

  acq_job_types_e type;

  for (type = 0; type < ACQ_NUM_JOB_TYPES; type++) {
    u32 i;
    for (i = 0; i < NUM_SATS_GPS; i++) {
      data->jobs[type][i].sid = construct_sid(CODE_GPS_L1CA,
                                              GPS_FIRST_PRN + i);
      data->jobs[type][i].job_type = type;
    }
  }
}
gps_time_t get_current_time(void) {
  gps_time_t t;
  t.wn = 0;
  t.tow = TOW_UNKNOWN;
  return t;
}

s8 calc_sat_doppler_wndw(const ephemeris_t* e, const gps_time_t *t,
                         const gnss_solution *lgf, u8 fails, float *radius,
                         float *doppler_min, float *doppler_max) {
  (void)e;
  (void)t;
  (void)lgf;
  (void)fails;
  (void)radius;
  *doppler_min = 100;
  *doppler_max = 200;
  return 0;
}

/** Test cost initialization
 *
 *  Test failure triggers assertion 
 *
 * \param none
 *
 * \return none
 */
static void sch_test_cost_init(void)
{
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  acq_job_t *init_job = &data->jobs[0][10];
  printf("%s\n",__FUNCTION__);
  sm_init(data);
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
  assert(70 == init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX;
  sch_initialize_cost(init_job, data);
  assert(120 == init_job->cost);

  init_job->cost_hint = ACQ_COST_AVG;
  sch_initialize_cost(init_job, data);
  assert((70 + 100 + 110 + 101 + 120) / 5 == init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX_PLUS;
  sch_initialize_cost(init_job, data);
  assert(120 + EXPECTED_COST_DELTA_MS == init_job->cost);

  /* Check that initalized job does not effect on itself */
  init_job->cost_hint = ACQ_COST_MIN;
  init_job->needs_to_run = true;
  init_job->cost = 5;
  sch_initialize_cost(init_job, data);
  assert(70 == init_job->cost);

  /* Check when there are no other jobs */
  sm_init(data);
  init_job->cost_hint = ACQ_COST_MIN;
  sch_initialize_cost(init_job, data);
  assert(0 == init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX;
  sch_initialize_cost(init_job, data);
  assert(0 == init_job->cost);

  init_job->cost_hint = ACQ_COST_AVG;
  sch_initialize_cost(init_job, data);
  assert(0 == init_job->cost);

  init_job->cost_hint = ACQ_COST_MAX_PLUS;
  sch_initialize_cost(init_job, data);
  assert(EXPECTED_COST_DELTA_MS == init_job->cost);
}

/** Test job selection
 *
 *  Test failure triggers assertion 
 *
 * \param none
 *
 * \return none
 */
static void sch_test_job_select(void)
{
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  acq_job_t *sel;
  printf("%s\n",__FUNCTION__);
  sm_init(data);
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
  assert(sel == &data->jobs[1][2]);

  /* Exclude if it should not run */
  data->jobs[1][2].state = ACQ_STATE_IDLE;
  data->jobs[1][2].needs_to_run = false;
  sel = sch_select_job(data);
  assert(sel == &data->jobs[1][1]);

  /* Exclude if it should not run */
  data->jobs[1][2].state = ACQ_STATE_WAIT;
  data->jobs[1][2].needs_to_run = false;
  sel = sch_select_job(data);
  assert(sel == &data->jobs[1][1]);
  
  /* Check enable running */
  data->jobs[1][2].needs_to_run = true;
  data->jobs[1][2].state = ACQ_STATE_IDLE;
  data->jobs[1][2].task_data.task_index = 255;
  data->jobs[1][2].task_data.task_index = 255;
  data->jobs[1][2].cost_hint = ACQ_COST_MIN;
  sel = sch_select_job(data);
  /* There are two jobs with minimum cost */
  assert(sel == &data->jobs[1][2] || sel == &data->jobs[1][1]);
  assert(ACQ_STATE_WAIT == data->jobs[1][2].state);
  assert(ACQ_MAX_UNITIALIZED_TASKS == data->jobs[1][2].task_data.task_index);
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
static void sch_expect_hw_run(bool run, u32 code_index)
{
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  hw_has_run = false;
  sch_run(data);
  if (run) {
    assert(hw_has_run);
    assert(hw_code_index == code_index);
  } else {
    assert(!hw_has_run);
  }
}

/** Test job scheduling
 *
 *  Test failure triggers assertion 
 *
 * \param none
 *
 * \return none
 */
static void sch_test_job_scheduling(void)
{
  acq_jobs_state_t *data = &acq_all_jobs_state_data;
  printf("%s\n",__FUNCTION__);
  {
    /* Check that nothing is run */
    sm_init(data);
    sch_run(data);
    sch_expect_hw_run(false, 0);
  }
  {
    /* Check that if nothing runs, time flies */
    now_ms = 0;
    sm_init(data);
    sch_run(data);
    sch_run(data);
    assert(2*EXPECTED_SLEEP_TIMEOUT_MS == now_ms);
  }
  { /* Check that if peak is found it does not continue */
    sm_init(data);
    data->jobs[0][20].needs_to_run = true;
    sch_expect_hw_run(true, 20);
    sch_expect_hw_run(false, 20);
  }
  { /* Check that if peak is not found, search continues */
    sm_init(data);
    data->jobs[0][10].needs_to_run = true;
    sch_expect_hw_run(true, 10);
    sch_expect_hw_run(true, 10);
    sch_expect_hw_run(true, 10);
  } 
  { /* Check scheduling order */
    int i;
    sm_init(data);
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
    for(i = 0; i <= EXPECTED_COST_DELTA_MS; i++) {
      sch_expect_hw_run(true, 10);
      sch_expect_hw_run(true, 11);
      sch_expect_hw_run(true, 12);      
    }
    sch_expect_hw_run(true, 13);
    for(i = 0; i < EXPECTED_COST_DELTA_MS; i++) {
      sch_expect_hw_run(true, 10);
      sch_expect_hw_run(true, 11);
      sch_expect_hw_run(true, 12);
      sch_expect_hw_run(true, 13);
    }
    sch_expect_hw_run(true, 14);
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
    data->jobs[0][20].needs_to_run = true;
    data->jobs[0][20].cost_hint = ACQ_COST_MIN;
    data->jobs[0][11].needs_to_run = true;
    data->jobs[0][11].cost_hint = ACQ_COST_MAX_PLUS;
    sch_expect_hw_run(true, 20);
    sch_expect_hw_run(true, 11);
   }
}
/** Test program checking scheduler operation
 *
 * \param none
 *
 * \return 1 on failure, 0 othersiwe
 */
int main(int argc, char **argv)
{
  sch_test_cost_init();
  sch_test_job_select();
  sch_test_job_scheduling();
  return 0;
}
