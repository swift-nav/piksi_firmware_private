/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef STARLING_CALC_PVT_H
#define STARLING_CALC_PVT_H

#include <libsbp/navigation.h>
#include <libsbp/orientation.h>
#include <libsbp/system.h>
#include <libswiftnav/common.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/observation.h>
#include <libswiftnav/pvt_engine/firmware_binding.h>
#include <libswiftnav/sbas_raw_data.h>
#include <libswiftnav/single_epoch_solver.h>

/**
 * Various solution output modes supported by the Starling Engine.
 *
 * LOW_LATENCY:  Output a solution immediately for every rover observation.
 * TIME_MATCHED: Output a solution whenever a base observation is received.
 * NO_DGNSS:     Output single-point low-latency solution.
 */
typedef enum {
  STARLING_SOLN_MODE_LOW_LATENCY,
  STARLING_SOLN_MODE_TIME_MATCHED,
  STARLING_SOLN_MODE_NO_DGNSS
} dgnss_solution_mode_t;

typedef enum {
  FILTER_FLOAT,
  FILTER_FIXED,
} dgnss_filter_t;

/**
 * Base observation input type. Starling engine operates
 * on base observations provided in this format.
 *
 * TODO(kevin) for now...
 */
typedef struct {
  /** GPS system time of the observation. */
  gps_time_t tor;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** The known, surveyed base position. */
  double known_pos_ecef[3];
  /** Observation Solution */
  pvt_engine_result_t soln;

  /** Number of observations in the set. */
  u8 n;
  u8 sender_id;
  /** Set of observations. */
  navigation_measurement_t nm[MAX_CHANNELS];
} obss_t;

#define MAX_REMOTE_OBS 150
/**
 * Uncollapsed observation input type.
 * Remote observations may contain multiple useful signals for satellites.
 * This observation type can fit more observations than the obss_t.
 * Eventually signals in uncollapsed_obss_t are collapsed, and copied to obss_t.
 */
typedef struct {
  /** GPS system time of the observation. */
  gps_time_t tor;
  /** Approximate base station position.
   * This may be the position as reported by the base station itself or the
   * position obtained from doing a single point solution using the base
   * station observations. */
  double pos_ecef[3];
  /** Is the `pos_ecef` field valid? */
  u8 has_pos;
  /** The known, surveyed base position. */
  double known_pos_ecef[3];
  /** Observation Solution */
  pvt_engine_result_t soln;

  /** Number of observations in the set. */
  u8 n;
  u8 sender_id;
  /** Set of observations. */
  navigation_measurement_t nm[MAX_REMOTE_OBS];
} uncollapsed_obss_t;

/**
 * Filter result data type returned by various API functions.
 */
typedef struct StarlingFilterSolution {
  dops_t dops;
  pvt_engine_result_t result;
} StarlingFilterSolution;

/** Maximum difference between observation times to consider them matched. */
#define TIME_MATCH_THRESHOLD 2e-3

/** number of milliseconds before SPP resumes in pseudo-absolute mode */
#define DGNSS_TIMEOUT_MS 5000

/** Maximum time that an observation will be propagated for to align it with a
 * solution epoch before it is discarded.  */
#define OBS_PROPAGATION_LIMIT 10e-3

/* Warn on 15 second base station observation latency */
#define BASE_LATENCY_TIMEOUT 15

/* Make the buffer large enough to handle 15 second latency at 10Hz */
#define STARLING_OBS_N_BUFF BASE_LATENCY_TIMEOUT * 10

/* Size of an spp solution in ECEF. */
#define SPP_ECEF_SIZE 3

void reset_rtk_filter(void);
/*******************************************************************************
 * Formal Starling API
 ******************************************************************************/

/* Initialize the Starling API.
 *
 * IMPORTANT:
 * This function should be called *once* at the start of the program.
 * Failure to do so before invoking other Starling API functions will
 * result in undefined behavior. Calling this function multiple times
 * also results in undefined behavior. */
void starling_initialize_api(void);

/* Run the starling engine on the current thread. Blocks indefinitely. */
void starling_run(void);

/*******************************************************************************
 * Starling Data API
 ******************************************************************************/

/*
 * DATA API
 * ========
 * All regularly occurring inputs to the Starling Engine are provided through a
 * uniform interface. Namely, the user must provide a "read" function for each
 * supported input type which conforms to the following interface.
 *
 * 1. BLOCKING / NONBLOCKING
 *      -- Read functions must accept an argument indicating whether or not
 *         blocking behavior is desired. The implementation may determine
 *         the length of any blocking timeout. By invoking a read function
 *         in blocking mode, the Starling Engine is merely indicating that
 *         it has no work to do until it receives something.
 * 2. COPY BEHAVIOR
 *      -- Starling Engine requires that data is copied in. Support for
 *         message passing will come later.
 * 3. RETURNS
 *      -- Read functions may return implementation specific return codes. 
 *         Zero must always indicate success.
 */

/* Return codes for read functions. */
#define STARLING_READ_OK                   0

/* Blocking indicator values. */
#define STARLING_READ_NONBLOCKING 0
#define STARLING_READ_BLOCKING    1

/* Table of read functions for regularly occurring data streams. */
typedef struct StarlingInputFunctionTable {
  int (*read_obs_rover) (int blocking, obss_t *o);
  int (*read_obs_base)  (int blocking, obss_t *o);
} StarlingInputFunctionTable;

#if 0
// TODO(kevin) future work..
  int (*read_ephemeris)  (int flags, ephemeris_t *e);
  int (*read_utc_params) (int flags, utc_params_t *p);
  int (*read_sbas_data)  (int flags, sbas_data_t *s);
  int (*read_imu_data)   (int flags, imu_data_t *i);
#endif

/* Set the table of read functions. Should only be called on startup. 
 * TODO(kevin) make this a parameter to initialize or run. */
void starling_set_input_functions(const StarlingInputFunctionTable *functions);

/* Add raw sbas data to the starling engine. */
void starling_add_sbas_data(const sbas_raw_data_t *sbas_data,
                            const size_t n_sbas_data);

/**
 * These functions must be implemented by the integration layer
 * in order to process the Starling engine outputs.
 *
 * TODO(kevin) Change to more formal callback mechanism.
 */
void send_solution_time_matched(const StarlingFilterSolution *solution,
                                const obss_t *obss_base,
                                const obss_t *obss_rover);

void send_solution_low_latency(const StarlingFilterSolution *spp_solution,
                               const StarlingFilterSolution *rtk_solution,
                               const gps_time_t *solution_epoch_time,
                               const navigation_measurement_t *nav_meas,
                               const size_t num_nav_meas);

/*******************************************************************************
 * Starling Configuration API
 ******************************************************************************/

/* Enable glonass constellation in the Starling engine. */
void starling_set_is_glonass_enabled(bool is_glonass_enabled);
/* Enable galileo constellation in the Starling engine. */
void starling_set_is_galileo_enabled(bool is_galileo_enabled);
/* Enable beidou constellation in the Starling engine. */
void starling_set_is_beidou_enabled(bool is_beidou_enabled);
/* Enable fixed RTK mode in the Starling engine. */
void starling_set_is_fix_enabled(bool is_fix_enabled);
/* Enable klobuchar corrections in the time-matched filter. */
void starling_set_is_time_matched_klobuchar_enabled(bool is_klobuchar_enabled);
/* Indicate for how long corrections should persist. */
void starling_set_max_correction_age(int max_age);
/* Modify the relative weighting of glonass observations. */
void starling_set_glonass_downweight_factor(float factor);
/* Set the elevation mask used to filter satellites from the solution. */
void starling_set_elevation_mask(float elevation_mask);
/* Set the rate at which the filter calculates solutions. */
void starling_set_solution_frequency(double frequency);
/* Set a surveyed reference position for the base station. */
void starling_set_known_ref_pos(const double base_pos[3]);
/* Update the glonass biases. */
void starling_set_known_glonass_biases(const glo_biases_t biases);

/* Set the desired solution mode for the Starling engine. */
void starling_set_solution_mode(dgnss_solution_mode_t mode);
/* Get the current solution mode for the Starling engine. */
dgnss_solution_mode_t starling_get_solution_mode(void);

#endif
