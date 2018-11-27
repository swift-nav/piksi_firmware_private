/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <swiftnav/common.h>
#include <swiftnav/constants.h>
#include <swiftnav/logging.h>

#include <hal.h>

#include "nap_conf.h"
#include "nap_constants.h"
#include "nap_hw.h"

/** \addtogroup nap
 * \{ */

/** \defgroup conf Configuration
 * Functions to get information about the SwiftNAP configuration.
 * \{ */

u32 nap_conf_rd_random(void) {
  chSysLock();
  NAP->CONTROL = SET_NAP_CONTROL_VERSION_ADDR(NAP->CONTROL, NAP_RANDOM_OFFSET);
  u32 version = NAP->VERSION;
  chSysUnlock();
  return version;
}

u32 nap_conf_rd_version(void) {
  chSysLock();
  NAP->CONTROL = SET_NAP_CONTROL_VERSION_ADDR(NAP->CONTROL, NAP_VERSION_OFFSET);
  u32 version = NAP->VERSION;
  chSysUnlock();
  return version;
}

u8 nap_conf_rd_version_string(char version_string[]) {
  u8 i = 0;
  u32 reg = 0;

  chSysLock();
  u32 ctrl = (NAP->CONTROL & ~((u32)NAP_CONTROL_VERSION_ADDR_Msk));

  do {
    NAP->CONTROL = SET_NAP_CONTROL_VERSION_ADDR(
        ctrl, (i + NAP_VERSION_STRING_OFFSET) / sizeof(reg));
    reg = NAP->VERSION;
    memcpy(&version_string[i], &reg, sizeof(reg));
    i += sizeof(reg);
  } while (reg && i < NAP_VERSION_STRING_LENGTH);
  chSysUnlock();
  version_string[i] = 0;

  return strlen(version_string);
}

u8 nap_conf_rd_date_string(char date_string[]) {
  u32 reg;

  chSysLock();
  u32 ctrl = (NAP->CONTROL & ~((u32)NAP_CONTROL_VERSION_ADDR_Msk));

  NAP->CONTROL = SET_NAP_CONTROL_VERSION_ADDR(ctrl, NAP_BUILD_TIME_OFFSET);
  reg = NAP->VERSION;
  u8 hrs = (reg & 0x00FF0000) >> 16;
  u8 min = (reg & 0x0000FF00) >> 8;
  u8 sec = (reg & 0x000000FF);

  NAP->CONTROL = SET_NAP_CONTROL_VERSION_ADDR(ctrl, NAP_BUILD_DATE_OFFSET);
  reg = NAP->VERSION;
  u16 yrs = (reg & 0xFFFF0000) >> 16;
  u8 mon = (reg & 0x0000FF00) >> 8;
  u8 day = (reg & 0x000000FF);
  chSysUnlock();

  sprintf(date_string,
          "%04X-%02X-%02X %02X:%02X:%02X UTC",
          yrs,
          mon,
          day,
          hrs,
          min,
          sec);

  return strlen(date_string);
}

void nap_rd_dna(u8 dna[]) {
  u32 reg = 0;

  chSysLock();
  u32 ctrl = (NAP->CONTROL & ~((u32)NAP_CONTROL_VERSION_ADDR_Msk));

  for (u8 i = 0; i < NAP_DNA_LENGTH; i += sizeof(reg)) {
    NAP->CONTROL =
        SET_NAP_CONTROL_VERSION_ADDR(ctrl, (i + NAP_DNA_OFFSET) / sizeof(reg));
    reg = NAP->VERSION;
    memcpy(&dna[i], &reg, sizeof(reg));
  }
  chSysUnlock();
}

bool nap_locked(void) {
  return GET_NAP_STATUS_LOCKED(NAP->STATUS) ? true : false;
}

void nap_unlock(const u8 key[]) {
  /* Linux has access to the NAP->AUTHENTICATE register as well.
   * It can potentially access it, while nap_unlock() is writing the key,
   * because both CPUs share 1 AXI bus arbiter, that can interrupt at any
   * 'random' time.
   * In the case Linux was interrupting and working with the register, the
   * RSA core would assert the status-busy flag.
   * This polling would then at least give us information that Linux
   * interfered here.
   */
  chSysLock();
  volatile u16 n = 0;
  while (GET_NAP_STATUS_AUTH_BUSY(NAP->STATUS)) {
    log_error("Linux interrupted writing of NAP->AUTHENTICATE. Count=%i", n);
  }
  for (u32 i = 0; i < NAP_KEY_LENGTH; ++i) {
    NAP->AUTHENTICATION = SET_NAP_AUTHENTICATION_OPERATION(0, 0) |
                          SET_NAP_AUTHENTICATION_ADDR(0, i) |
                          SET_NAP_AUTHENTICATION_BYTE(0, (u32)key[i]);
  }
  chSysUnlock();
}

/** \} */

/** \} */
