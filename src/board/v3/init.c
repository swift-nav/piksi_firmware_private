/*
 * Copyright (C) 2013-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <hal.h>

#include <stdlib.h>
#include <string.h>

#include <libsbp/sbp.h>

#include <libswiftnav/logging.h>

#include "main.h"
#include "board/nap/nap_common.h"
#include "nap/nap_conf.h"
#include "nap/fft.h"
#include "sbp.h"
#include "error.h"
#include "frontend.h"
#include "xadc.h"
#include "nt1065.h"
#include "system_monitor.h"
#include "remoteproc/rpmsg.h"
#include "sbp_fileio.h"
#include "factory_data.h"
#include "clk_dac.h"
#include "peripherals/antenna.h"
#include "manage_led.h"
#include "imu.h"

#define REQUIRED_NAP_VERSION_MASK (0xFFFF0000U)
#define REQUIRED_NAP_VERSION_VAL  (0x03060000U)

#define SLCR_PSS_RST_CTRL (*(volatile u32 *)0xf8000200)
#define SLCR_PSS_RST_CTRL_SOFT_RST 1

#define DEV_CFG_INT_STS (*(volatile u32 *)0xf800700c)
#define DEV_CFG_INT_STS_PCFG_DONE_Msk (1U << 2)

#define FACTORY_DATA_SIZE_MAX 255

#define IMAGE_HARDWARE_INVALID      0xffffffff
#define IMAGE_HARDWARE_UNKNOWN      0x00000000
#define IMAGE_HARDWARE_V3_MICROZED  0x00000001
#define IMAGE_HARDWARE_V3_EVT1      0x00000011
#define IMAGE_HARDWARE_V3_EVT2      0x00000012
#define IMAGE_HARDWARE_V3_PROD      0x00000013

static struct {
  uint32_t hardware;
  uint8_t uuid[16];
  uint8_t nap_key[16];
} factory_params;

static void nap_conf_check(void);
static bool nap_version_ok(u32 version);
static void nap_version_check(void);
static void nap_auth_setup(void);
static void nap_auth_check(void);
static bool factory_params_read(void);

void pre_init(void)
{
  rpmsg_setup();
}

static void random_init(void)
{
  u32 seed = 0;
  u32 sample_data = nap_conf_rd_random();

  for (int i = 0; i < 32; i++)
    seed ^= sample_data >> i;

  srand(seed);
}

void init(void)
{
  fault_handling_setup();
  factory_params_read();

  /* Make sure FPGA is configured - required for EMIO usage */
  nap_conf_check();

  nap_version_check();
  nap_dna_callback_register();
  nap_auth_setup();
  nap_auth_check();
  nap_setup();

  /* Start DAC off at it's midpoint if present */
  set_clk_dac(2048, CLK_DAC_MODE_0);

  frontend_configure();

  /* Initialize rollover counter */
  nap_timing_count();

  random_init();
  xadc_init();
  antenna_init();
  manage_led_setup();
  imu_init();
}

static void nap_conf_check(void)
{
  while (!(DEV_CFG_INT_STS & DEV_CFG_INT_STS_PCFG_DONE_Msk)) {
    log_error("Waiting for NAP");
    chThdSleepSeconds(2);
  }
}

static bool nap_version_ok(u32 version)
{
  return ((version & REQUIRED_NAP_VERSION_MASK) == REQUIRED_NAP_VERSION_VAL);
}

static void nap_version_check(void)
{
  u32 nap_version = nap_conf_rd_version();
  if (!nap_version_ok(nap_version)) {
    while (1) {
      log_error("Unsupported NAP version: 0x%08x", nap_version);
      chThdSleepSeconds(2);
    }
  }
}

static void nap_auth_setup(void)
{
  nap_unlock(factory_params.nap_key);
}

/* Check NAP authentication status. Block and print error message
 * if authentication has failed. This must be done after the NAP,
 * USARTs, and SBP subsystems are set up, so that SBP messages and
 * be sent and received (it can't go in init() or nap_setup()).
 */
static void nap_auth_check(void)
{
  if (nap_locked()) {
    while (1) {
      log_error("NAP Verification Failed");
      chThdSleepSeconds(2);
    }
  }
}

static bool factory_params_read(void)
{
  uint8_t factory_data_buff[FACTORY_DATA_SIZE_MAX];
  factory_data_t *factory_data = (factory_data_t *)factory_data_buff;

  /* read file */
  ssize_t bytes_read = sbp_fileio_read("/factory/mtd", 0, factory_data_buff,
                                       sizeof(factory_data_buff));
  if (bytes_read < (ssize_t)sizeof(factory_data_t)) {
    log_error("error reading factory data");
    return false;
  }

  /* verify header */
  if (factory_data_header_verify(factory_data) != 0) {
    log_error("error verifying factory data header");
    return false;
  }

  uint32_t factory_data_body_size = factory_data_body_size_get(factory_data);
  uint32_t factory_data_size = sizeof(factory_data_t) +
                               factory_data_body_size;

  /* check header + body length */
  if (bytes_read < (ssize_t)factory_data_size) {
    log_error("error reading factory data");
    return false;
  }

  /* verify body */
  if (factory_data_body_verify(factory_data) != 0) {
    log_error("error verifying factory data body");
    return false;
  }

  /* read params */
  if (factory_data_hardware_get(factory_data, &factory_params.hardware) != 0) {
    log_error("error reading hardware parameter from factory data");
    return false;
  }

  if (factory_data_uuid_get(factory_data, factory_params.uuid) != 0) {
    log_error("error reading uuid from factory data");
    return false;
  }

  if (factory_data_nap_key_get(factory_data, factory_params.nap_key) != 0) {
    log_error("error reading NAP key from factory data");
    return false;
  }

  return true;
}

u32 serial_number_get(void)
{
  u32 serial_int = factory_params.uuid[0] +
                   (factory_params.uuid[1] << 8) +
                   (factory_params.uuid[2] << 16) +
                   (factory_params.uuid[3] << 24);
  return serial_int;
}

u8 hw_revision_string_get(char *hw_revision_string)
{
  const char *s = NULL;

  switch(factory_params.hardware) {
  case IMAGE_HARDWARE_UNKNOWN:
    s = "Unknown";
    break;
  case IMAGE_HARDWARE_V3_MICROZED:
    s = "Piksi Multi MicroZed";
    break;
  case IMAGE_HARDWARE_V3_EVT1:
    s = "Piksi Multi EVT1";
    break;
  case IMAGE_HARDWARE_V3_EVT2:
    s = "Piksi Multi EVT2";
    break;
  case IMAGE_HARDWARE_V3_PROD:
    s = "Piksi Multi";
    break;
  default:
    s = "Invalid";
    break;
  }

  strcpy(hw_revision_string, s);
  return strlen(hw_revision_string);
}

u8 nap_version_string_get(char *nap_version_string)
{
  return nap_conf_rd_version_string(nap_version_string);
}
