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

#ifndef STARLING_CALC_STARLING_PLATFORM_H
#define STARLING_CALC_STARLING_PLATFORM_H

/**
 * Starling Platform Abstraction Layer
 * ===================================
 * The Platform Abstraction Layer (PAL) serves to provide the Starling Engine
 * with everything it needs to run on a given software platform. Apart from these
 * platform specific "primitives", the Starling Engine is designed to be 
 * completely portable.
 *
 * The Starling Engine manages a handful of threads over which it spreads the
 * GNSS estimation workload. Interface with and between these threads is handled
 * by message passing over queues. In order to allow efficient platform-specific
 * implementation of these primitives, they are abstracted in this layer.
 *
 * PAL implementers should make the following assumptions:
 *
 * 1. All platform primitives will be allocated only at application start.
 * 2. The worst-case requirements for each primitive are provided as constants.
 * 3. Message passing is by pointer. Alloc/dealloc is handled by the allocator.
 *
 *
 * NOTE:
 * Functions which are guaranteed to be called only once at startup are
 * marked with a STARTUP_ONLY comment.
 */

/******************************************************************************
 * GENERAL 
 *****************************************************************************/

#define STARLING_PAL_OK  0x0

/** 
 * Generic return type used by many functions in the API.
 * If a function returns successfully, it should return
 * STARLING_PAL_OK. When unsuccessful, functions are permitted
 * to return any other value with implementation-specific meaning.
 */
typedef int pal_rc_t;

/** 
 * Handle an implementation-specific error. The Starling Engine
 * may invoke this function on any return code which is not
 * STARLING_PAL_OK.
 */
void pal_handle_error(pal_rc_t code);

/* 
 * Initialize the PAL. No primitive-related functions will be called
 * until after the PAL has been successfully initialized.
 *
 * STARTUP_ONLY
 */
pal_rc_t pal_init(void);

/******************************************************************************
 * THREAD
 *****************************************************************************/

// Maximum number of threads requested by the Starling engine.
#define STARLING_MAX_NUM_THREADS  0x2
// Maximum thread stack size requirement (in bytes) of Starling threads.
#define STARLING_MAX_THREAD_STACK 0x1000

/**
 * Run a function on a separate thread with given priority and 
 * user context. The implementation may adjust the actual thread 
 * priority in the OS, as long as the relative prioritization among
 * Starling threads is preserved. Lower value indicates higher priority.
 *
 * STARTUP_ONLY
 */
pal_rc_t pal_thread_run(unsigned int priority,
                        void (*fn)(void *), 
                        void *context);

/******************************************************************************
 * QUEUE 
 *****************************************************************************/

// Maximum number of queues requested by the Starling engine.
#define STARLING_MAX_NUM_QUEUES 0x10
// Maximum number of entries that queues must support.
#define STARLING_MAX_QUEUE_CAPACITY 0x4

void *pal_queue_create(const size_t capacity);

pal_rc_t pal_queue_push_back(void *q, const void *p);

pal_rc_t pal_queue_pop_front(void *q, void **p);

void pal_queue_destroy(void *q);

/******************************************************************************
 * ALLOCATOR
 *****************************************************************************/

// STARTUP_ONLY
pal_rc_t pal_allocator_register(const size_t block_size, 
                                const size_t n_blocks);

pal_rc_t pal_allocator_alloc(const size_t block_size,
                             void **p);

pal_rc_t pal_allocator_free(void *p);

#endif
