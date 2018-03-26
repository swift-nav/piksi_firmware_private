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

#ifndef STARLING_CALC_STARLING_THREADS_H
#define STARLING_CALC_STARLING_THREADS_H

/**
 * The idea of this file is to specify the additional functionality
 * provided when we move "starling_threads" into libswiftnav.
 *
 * This is *not* a dumping ground for all the intermediary dependencies
 * of "starling_threads". Those will go into a shim layer.
 */

// Setup the starling engine threads.
void starling_calc_pvt_setup(void);

// Function to run on the starling thread.
void starling_thread(void *arg);

// Function to run on the time-matched thread.
void time_matched_obs_thread(void *arg);

// Reset the time matched filter.
void reset_rtk_filter(void);

// Set the reference position for the time-matched filter.
void set_known_ref_pos(const double base_pos[3]);

// Set the glonass biases for the time-matched filter. 
void set_known_glonass_biases(const glo_biases_t biases);

// Set the fix mode for the solution.
void starling_threads_set_enable_fix_mode(bool enable_fix);

// Set the maximum age for the starling threads.
void starling_threads_set_max_age(int value);


#endif
