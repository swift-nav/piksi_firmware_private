/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Kevin Dade <kevin@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef STARLING_INPUT_BRIDGE_H_
#define STARLING_INPUT_BRIDGE_H_

#include <starling/starling.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Starling Input Bridge
 * =====================
 * This is a two-sided module which provides a thread-safe mechanism
 * for pushing data into the Starling engine. A sequence of "send" functions
 * allow a user to transmit data to Starling from any context.
 *
 * Symmetric "read" functions are then provided to the Starling engine
 * so it may go about its business.
 *
 * The API will operate in bypass mode until the bridge has been
 * initialized.
 */

/* Values returned by the firmware-facing functions. */
#define STARLING_SEND_OK 0
#define STARLING_SEND_ERROR 1

/* Operational modes for the Starling input bridge. */
typedef enum starling_bridge_mode_e {
  STARLING_BRIDGE_MODE_DEFAULT,
  STARLING_BRIDGE_MODE_BYPASS,
} starling_bridge_mode_t;

void starling_input_bridge_init(void);

void starling_input_bridge_shutdown(void);

void starling_input_bridge_set_mode(starling_bridge_mode_t mode);

/*******************************************************************************
 * Observation Memory Management Functions
 *
 * The alloc functions are for the client to use before populating the
 * observation data and "sending" it with the functions below.
 *
 * The free functions are for Starling to use once it has received the
 * observation data and is done with it.
 ******************************************************************************/

obs_array_t *starling_alloc_rover_obs(void);

obs_array_t *starling_alloc_base_obs(void);

void starling_free_rover_obs(obs_array_t *obs_array);

void starling_free_base_obs(obs_array_t *obs_array);

/*******************************************************************************
 * Client-Facing "Send" Functions
 ******************************************************************************/

/**
 * This function takes ownership of `obs_array` in all cases, even when
 * an error occurs.
 */
int starling_send_rover_obs(obs_array_t *obs_array);

/**
 * This function takes ownership of `obs_array` in all cases, even when
 * an error occurs.
 */
int starling_send_base_obs(obs_array_t *obs_array);

int starling_send_ephemerides(const ephemeris_t *ephemerides, size_t n);

int starling_send_sbas_data(const sbas_raw_data_t *sbas_data);

int starling_send_imu_data(const imu_data_t *imu_data);

/* Sets if starling will internally synchronize its threads.
 * Should only be used for non-realtime situations (i.e. file replay) */
void starling_synchronize_threads(bool synchronize);
/* Checks to see if starling thread synchronization is enabled */
bool starling_are_threads_synchronized(void);
/* Waits some time for starling's internal data processing to be complete.
 * Useful for being told when it is acceptable to give starling more data
 * to processes, such as in file replay when more data is immediately
 * available and we run the risk of giving more data than starling can process.
 * Note: This function should only be called when starling thread
 * synchronization is enabled.
 * Returns PLATFORM_SEM_OK, PLATFORM_SEM_TIMEOUT, or PLATFORM_SEM_ERROR */
int starling_wait_for_processing_done(uint32_t milliseconds);

/*******************************************************************************
 * Starling-Facing "Read" Functions
 ******************************************************************************/

/**
 * This polls the data queue for data, and routes the data to the proper
 * function. Each function pointer is optional, though an error will be logged
 * if a message of a particular type is found and no handler function for that
 * type is given.
 *
 * Note: In contrast to the "send" functions above, none of the handlers are
 * expected to take ownership of the data.
 */
int starling_poll_for_data(rover_obs_handler_f rover_handler,
                           base_obs_handler_f base_handler,
                           sbas_data_handler_f sbas_handler,
                           ephemeris_array_handler_f eph_handler,
                           imu_data_handler_f imu_handler);

void starling_signal_processing_done(void);
void starling_signal_tm_done(void);
int starling_wait_for_tm_done(uint32_t milliseconds);

#ifdef __cplusplus
}
#endif

#endif
