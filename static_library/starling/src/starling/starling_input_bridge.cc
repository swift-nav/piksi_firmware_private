/*
 * Copyright (C) 2018 Swift Navigation Inc.
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
#include <libpal/pal.h>
#include <starling/starling_input_bridge.h>
#include <starling/util/memory_pool.h>
#include <starling/util/mutex.h>
#include <stdbool.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/memcpy_s.h>

#include <atomic>

constexpr uint64_t micros_per_milli = 1000;

/**************************************************
 * Mutex Strategy: This file exclusively uses the Mutex::PRIMARY_QUEUE mutex.
 * It should be locked when any of the operations are performed:
 *   * Allocating from any MemoryPool
 *   * Freeing to any MemoryPool
 *
 * As usual, the mutex should be locked for the shortest amount of time
 * possible.
 *************************************************/

/* Warn on lack of input after 10 seconds. */
#define STARLING_INPUT_TIMEOUT_UNTIL_WARN_SEC 10

using starling::util::MemoryPool;
using starling::util::Mutex;

pal_mq_t primary_data_mq = nullptr;

namespace {

// Use a simple lock-free atomic access boolean to signal across threads
// This flag is used to prevent incoming data from being queued until
// the consumer thread is ready
// Note: Only data that is produced in large amounts is throttled (i.e. IMU
// data)
static_assert(ATOMIC_BOOL_LOCK_FREE == 2,
              "std::atomic_bool isn't always lock free");
std::atomic_bool bridge_ready_for_data(false);

/* Required message queue buffer lengths (# of elements) */
constexpr std::size_t ME_OBS_MSG_N_BUFF = 6;
constexpr std::size_t BASE_OBS_N_BUFF = 5;
constexpr std::size_t SBAS_DATA_N_BUFF = 6;
constexpr std::size_t EPHEMERIS_N_BUFF = 10;
constexpr std::size_t IMU_N_BUFF = 10;
constexpr std::size_t PRIMARY_QUEUE_N_BUFF =
    ME_OBS_MSG_N_BUFF + BASE_OBS_N_BUFF + SBAS_DATA_N_BUFF + EPHEMERIS_N_BUFF +
    IMU_N_BUFF;

typedef enum {
  STARLING_BRIDGE_ROVER_OBS_ID = 1,
  STARLING_BRIDGE_BASE_OBS_ID,
  STARLING_BRIDGE_EPHEMERIS_DATA_ID,
  STARLING_BRIDGE_SBAS_DATA_ID,
  STARLING_BRIDGE_IMU_DATA_ID,
} bridge_message_type_t;

typedef struct {
  void *data;
  bridge_message_type_t type;
} bridge_message_t;

bool is_bypass_enabled = true;

MemoryPool<obs_array_t> rover_obs_pool;
MemoryPool<obs_array_t> base_obs_pool;
MemoryPool<ephemeris_array_t> eph_data_pool;
MemoryPool<sbas_raw_data_t> sbas_data_pool;
MemoryPool<imu_data_t> imu_data_pool;
MemoryPool<bridge_message_t> bridge_message_pool;

pal_cv_t data_processed = nullptr;
pal_mutex_t data_processed_lock = nullptr;
pal_cv_t time_matched_done = nullptr;
pal_mutex_t time_matched_done_lock = nullptr;
std::atomic_bool synchronize_threads(false);

template <typename T>
void alloc_memory_helper(MemoryPool<T> &pool, size_t element_count,
                         const char *msg_name) {
  const size_t buffer_size = pool.node_size * element_count;
  void *ptr;

  if (pal_mem_alloc(&ptr, buffer_size) != PAL_SUCCESS) {
    log_error("Failed to allocate memory for %s data pool", msg_name);
  } else {
    if (!pool.Init(ptr, buffer_size, msg_name)) {
      log_error("Error while initializing memory pool for %s data", msg_name);
    }
  }
}

}  // anonymous namespace

/******************************************************************************/
void starling_input_bridge_init(void) {
  alloc_memory_helper(rover_obs_pool, ME_OBS_MSG_N_BUFF, "ROVER");
  alloc_memory_helper(base_obs_pool, BASE_OBS_N_BUFF, "BASE");
  alloc_memory_helper(sbas_data_pool, SBAS_DATA_N_BUFF, "SBAS");
  alloc_memory_helper(eph_data_pool, EPHEMERIS_N_BUFF, "EPHEMERIS");
  alloc_memory_helper(imu_data_pool, IMU_N_BUFF, "IMU");
  alloc_memory_helper(bridge_message_pool, PRIMARY_QUEUE_N_BUFF,
                      "MESSAGEQUEUE");

  pal_mq_alloc(PRIMARY_QUEUE_N_BUFF, &primary_data_mq);

  starling_input_bridge_set_mode(STARLING_BRIDGE_MODE_DEFAULT);

  assert(pal_cv_alloc(&data_processed) == PAL_SUCCESS);
  data_processed_lock = Mutex::GetMutex(Mutex::DATA_PROCESSED);
  assert(data_processed_lock);
  assert(pal_cv_alloc(&time_matched_done) == PAL_SUCCESS);
  time_matched_done_lock = Mutex::GetMutex(Mutex::TIME_MATCHED_DONE);
  assert(time_matched_done_lock);
}

/******************************************************************************/
void starling_input_bridge_shutdown(void) {
  // Pushing a NULL message on to the queue will force the popping thread to
  // wake up and check that it's meant to exit now
  pal_mq_push(primary_data_mq, nullptr, PAL_MQ_BLOCKING, 0);
}

/******************************************************************************/
void starling_input_bridge_set_mode(starling_bridge_mode_t mode) {
  switch (mode) {
    case STARLING_BRIDGE_MODE_BYPASS:
      is_bypass_enabled = true;
      break;
    case STARLING_BRIDGE_MODE_DEFAULT:
    default:
      is_bypass_enabled = false;
      break;
  }
}

namespace {

using starling::util::LockGuard;
using starling::util::Mutex;

/**
 * This helper function performs all of the common actions when sending a
 * message. Note: This function takes ownership of the data pointer is it given,
 * and will properly free it if an error ocurrs.
 */
template <typename T>
int starling_send_message_helper(bridge_message_type_t type, T *data,
                                 MemoryPool<T> &data_pool,
                                 const char *msg_name) {
  if (is_bypass_enabled) {
    data_pool.Free(data);
    return STARLING_SEND_OK;
  }

  bridge_message_t *msg = nullptr;
  {
    LockGuard token(Mutex::PRIMARY_QUEUE);
    msg = bridge_message_pool.Alloc();
  }

  if (nullptr == msg) {
    log_error("Starling Input: Unable to allocate memory for %s message!",
              msg_name);
    data_pool.Free(data);
    data = nullptr;
    return STARLING_SEND_ERROR;
  }

  msg->type = type;
  msg->data = data;

  int push_error = pal_mq_push(primary_data_mq, msg, PAL_MQ_NONBLOCKING, 0);

  if (push_error != PAL_SUCCESS) {
    LockGuard token(Mutex::PRIMARY_QUEUE);
    log_error("Starling Input: Unable to push %s data!", msg_name);
    data_pool.Free(data);
    data = nullptr;
    bridge_message_pool.Free(msg);
    msg = nullptr;
    return STARLING_SEND_ERROR;
  }

  return STARLING_SEND_OK;
}

}  // anonymous namespace

/******************************************************************************/
int starling_send_rover_obs(obs_array_t *obs_array) {
  if (is_bypass_enabled) {
    rover_obs_pool.Free(obs_array);
    obs_array = nullptr;
    return STARLING_SEND_OK;
  }

  return starling_send_message_helper(STARLING_BRIDGE_ROVER_OBS_ID, obs_array,
                                      rover_obs_pool, "ROVER");
}

/******************************************************************************/
int starling_send_base_obs(obs_array_t *obs_array) {
  if (is_bypass_enabled) {
    base_obs_pool.Free(obs_array);
    obs_array = nullptr;
    return STARLING_SEND_OK;
  }

  return starling_send_message_helper(STARLING_BRIDGE_BASE_OBS_ID, obs_array,
                                      base_obs_pool, "BASE");
}

/******************************************************************************/
int starling_send_ephemerides(const ephemeris_t *ephemerides, size_t n) {
  if (is_bypass_enabled) {
    return STARLING_SEND_OK;
  }

  ephemeris_array_t *eph_array = nullptr;

  {
    LockGuard token(Mutex::PRIMARY_QUEUE);
    eph_array = eph_data_pool.Alloc();
  }

  if (nullptr == eph_array) {
    log_error("Unable to allocate ephemeris array.");
    return STARLING_SEND_ERROR;
  }

  assert(nullptr != eph_array);
  /* Copy in all of the information. */
  eph_array->n = n;
  if (n > 0) {
    memcpy_s_t memcpy_s_res =
        memcpy_s(eph_array->ephemerides, sizeof(eph_array->ephemerides),
                 ephemerides, n * sizeof(ephemeris_t));
    if (MEMCPY_S_OK != memcpy_s_res) {
      log_error("MEMCPY_S failed with code %d", memcpy_s_res);
      assert(MEMCPY_S_OK == memcpy_s_res);
    }
  }

  return starling_send_message_helper(STARLING_BRIDGE_EPHEMERIS_DATA_ID,
                                      eph_array, eph_data_pool, "EPHEMERIS");
}

/******************************************************************************/
int starling_send_sbas_data(const sbas_raw_data_t *sbas_data) {
  if (is_bypass_enabled) {
    return STARLING_SEND_OK;
  }

  sbas_raw_data_t *sbas_data_msg = nullptr;

  {
    LockGuard token(Mutex::PRIMARY_QUEUE);
    sbas_data_msg = sbas_data_pool.Alloc();
  }

  if (nullptr == sbas_data_msg) {
    log_error("Unable to allocate sbas data.");
    return STARLING_SEND_ERROR;
  }
  assert(sbas_data);
  *sbas_data_msg = *sbas_data;
  return starling_send_message_helper(STARLING_BRIDGE_SBAS_DATA_ID,
                                      sbas_data_msg, sbas_data_pool, "SBAS");
}

/******************************************************************************/
int starling_send_imu_data(const imu_data_t *imu_data) {
  if (is_bypass_enabled || !bridge_ready_for_data) {
    return STARLING_SEND_OK;
  }

  imu_data_t *imu_msg = nullptr;

  {
    LockGuard token(Mutex::PRIMARY_QUEUE);
    imu_msg = imu_data_pool.Alloc();
  }

  if (nullptr == imu_msg) {
    log_error("Unable to allocate imu data.");
    return STARLING_SEND_ERROR;
  }

  assert(imu_data);
  *imu_msg = *imu_data;
  return starling_send_message_helper(STARLING_BRIDGE_IMU_DATA_ID, imu_msg,
                                      imu_data_pool, "IMU");
}

obs_array_t *starling_alloc_rover_obs() {
  LockGuard token(Mutex::PRIMARY_QUEUE);
  return rover_obs_pool.Alloc();
}

void starling_free_rover_obs(obs_array_t *obs_array) {
  LockGuard token(Mutex::PRIMARY_QUEUE);
  return rover_obs_pool.Free(obs_array);
}

obs_array_t *starling_alloc_base_obs() {
  LockGuard token(Mutex::PRIMARY_QUEUE);
  return base_obs_pool.Alloc();
}

void starling_free_base_obs(obs_array_t *obs_array) {
  LockGuard token(Mutex::PRIMARY_QUEUE);
  return base_obs_pool.Free(obs_array);
}

namespace {

/**
 * This function assumes the type checking on the `bridge_message_t` has already
 * been done and it is safe to cast the `void *`.
 */
template <typename T>
void polling_handler_helper(bridge_message_t &msg, void (*handler)(T *),
                            MemoryPool<T> &pool, const char *msg_name) {
  if (handler) {
    handler(static_cast<T *>(msg.data));
  } else {
    log_error(
        "%s message received, but no handler was given. Dropping message.",
        msg_name);
  }

  {
    LockGuard token(Mutex::PRIMARY_QUEUE);
    pool.Free(msg.data);
  }

  msg.data = nullptr;
}

}  // anonymous namespace

/******************************************************************************/
int starling_poll_for_data(rover_obs_handler_f rover_handler,
                           base_obs_handler_f base_handler,
                           sbas_data_handler_f sbas_handler,
                           ephemeris_array_handler_f eph_handler,
                           imu_data_handler_f imu_handler) {
  int result = 0;

  void *ptr = nullptr;

  result = pal_mq_pop(primary_data_mq, &ptr, PAL_MQ_BLOCKING, 0);

  // A NULL message probably means something has called starling_stop(), it's
  // time to finish up this thread
  if (result == PAL_SUCCESS && ptr != nullptr) {
    bridge_message_t *msg = static_cast<bridge_message_t *>(ptr);

    switch (msg->type) {
      case STARLING_BRIDGE_ROVER_OBS_ID:
        polling_handler_helper(*msg, rover_handler, rover_obs_pool, "ROVER");
        break;
      case STARLING_BRIDGE_BASE_OBS_ID:
        polling_handler_helper(*msg, base_handler, base_obs_pool, "BASE");
        break;
      case STARLING_BRIDGE_EPHEMERIS_DATA_ID:
        polling_handler_helper(*msg, eph_handler, eph_data_pool, "EPHEMERIS");
        break;
      case STARLING_BRIDGE_SBAS_DATA_ID:
        polling_handler_helper(*msg, sbas_handler, sbas_data_pool, "SBAS");
        break;
      case STARLING_BRIDGE_IMU_DATA_ID:
        polling_handler_helper(*msg, imu_handler, imu_data_pool, "IMU");
        break;
      default:
        log_error(
            "starling_poll_for_data(): Unknown message type %d, we may be "
            "leaking memory",
            msg->type);
        break;
    }

    {
      LockGuard token(Mutex::PRIMARY_QUEUE);
      bridge_message_pool.Free(msg);
      msg = nullptr;
    }

    if (starling_are_threads_synchronized()) {
      starling_signal_processing_done();
    }
  }

  return result;
}

void starling_synchronize_threads(bool synchronize) {
  synchronize_threads = synchronize;
}

bool starling_are_threads_synchronized() { return synchronize_threads; }

int starling_wait_for_processing_done(uint32_t milliseconds) {
  int ret = PAL_ERROR;
  if (synchronize_threads) {
    assert(data_processed);
    assert(data_processed_lock);

    pal_mutex_lock(data_processed_lock);
    ret =
        pal_cv_wait_for(data_processed, data_processed_lock,
                        static_cast<uint64_t>(milliseconds) * micros_per_milli);
    pal_mutex_unlock(data_processed_lock);
  }
  return ret;
}

int starling_wait_for_tm_done(uint32_t milliseconds) {
  int ret = PAL_ERROR;
  if (synchronize_threads) {
    assert(time_matched_done);
    assert(time_matched_done_lock);

    pal_mutex_lock(time_matched_done_lock);
    ret =
        pal_cv_wait_for(time_matched_done, time_matched_done_lock,
                        static_cast<uint64_t>(milliseconds) * micros_per_milli);
    pal_mutex_unlock(time_matched_done_lock);
  }
  return ret;
}

void starling_signal_processing_done(void) {
  if (synchronize_threads) {
    assert(data_processed);
    pal_cv_notify_all(data_processed);
  }
}

void starling_signal_tm_done(void) {
  if (synchronize_threads) {
    assert(data_processed);
    pal_cv_notify_all(time_matched_done);
  }
}
