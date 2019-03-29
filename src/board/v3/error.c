/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "error.h"

#include <hal.h>
#include <libsbp/sbp.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/logging.h>

#include "peripherals/leds.h"
#include "piksi_systime.h"
#include "remoteproc/rpmsg.h"
#include "sbp.h"
#include "zynq7000.h"

/** \addtogroup error
 * System low-level error handling and reporting
 * \{ */

/** \addtogroup io
 * \{ */

/** A direct write function to rpmsg, for use by screaming_death
 */
#define PANIC_RPMSG_ENDPOINT RPMSG_ENDPOINT_A /* SBP rpmsg endpoint */
static s32 fallback_write_rpmsg(u8 *buff, u32 n, void *context) {
  (void)context;
  return rpmsg_halt_manual_send(PANIC_RPMSG_ENDPOINT, buff, n) ? n : 0;
}
/** \} */

/** Error message.
 * Halts the program while continually sending a fixed error message in SBP
 * message format to the sbp rpmsg endpoint, in a way that should get the
 * message through to any ports serving SBP
 *
 * \param fmt C string that contains the text to be written
 * \param ... Variadic arguments
 */
void _screaming_death(const char *fmt, ...) {
  __asm__("CPSID if;"); /* Disable all interrupts and faults */

#define SPEAKING_MSG_N 222 /* Maximum length of error message */

  static char err_msg[SPEAKING_MSG_N] = " ";

  va_list args;
  va_start(args, fmt);
  size_t len = strlen(err_msg);
  /* Accommodate newline and null chars (-1 - 1) */
  vsnprintf(err_msg + len, sizeof(err_msg) - len - 1 - 1, fmt, args);
  va_end(args);
  /* Accommodate null char (-1) */

  strncat(err_msg, "\n", SPEAKING_MSG_N - strlen(err_msg) - 1);

  len = strlen(err_msg);
  err_msg[0] = LOG_ERROR;

  static sbp_state_t sbp_state;
  sbp_state_init(&sbp_state);

/* Continuously send error message */
#define APPROX_ONE_SEC 200000000
  while (1) {
    for (u32 d = 0; d < APPROX_ONE_SEC; d++) {
      __asm__("nop");
    }
    sbp_send_message(
        &sbp_state, SBP_MSG_LOG, 0, len, (u8 *)err_msg, &fallback_write_rpmsg);
  }
}

/* OS syscall implementations related to error conditions */

/** Custom assert() failure function. Calls screaming_death(). */
void __assert_func(const char *_file,
                   int _line,
                   const char *_func,
                   const char *_expr) {
  thread_t *thread = chThdGetSelfX();
  const char *name = NULL;

  if (thread) {
    name = chRegGetThreadNameX(thread);
  }

  if (NULL == name) {
    name = "unknown";
  }

  _screaming_death(
      "%s:%s:%s():%d assertion '%s' failed", name, _file, _func, _line, _expr);
}

/** Required by exit() which is (hopefully not) called from BLAS/LAPACK. */
void _fini(void) { return; }

/** _exit(2) syscall handler.  Called by (at least) abort() and exit().
 * Calls screaming_death() to repeatedly print an ERROR until WDT reset.
 */
void __wrap__exit(int status) {
  (void)status;
  /* TODO: Perhaps print a backtrace; let's see if this ever actually
     occurs before implementing that. */
  screaming_death("abort() or exit() was called");
}

/** Enable and/or register handlers for system faults (hard fault, bus
 * fault, memory protection, usage (i.e. divide-by-zero) */
void fault_handling_setup(void) {}

/* Called by fault handlers in error_asm.s */
void fault_handler_screaming_death(const char *msg_str, u32 lr) {
  char msg[128];
  extern int fallback_sprintf(char *str, const char *fmt, ...);
  fallback_sprintf(msg, "%s lr=%08X", msg_str, (unsigned int)lr);
  screaming_death(msg);
}

/** \} */
