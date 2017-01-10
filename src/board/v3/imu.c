/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "imu.h"
#include <settings.h>
#include <math.h>
#include <ch.h>
#include <hal.h>
#include <sbp.h>
#include <timing.h>
#include <libswiftnav/time.h>
#include <libswiftnav/logging.h>
#include <libsbp/imu.h>
#include <board/nap/nap_common.h>
#include <board/v3/peripherals/bmi160.h>

#define IMU_THREAD_PRIO        (HIGHPRIO - 1)
#define IMU_THREAD_STACK       2000
#define IMU_AUX_THREAD_PRIO    (LOWPRIO + 10)
#define IMU_AUX_THREAD_STACK   2000

/** Working area for the IMU data processing thread. */
static THD_WORKING_AREA(wa_imu_thread, IMU_THREAD_STACK);
/** Working area for the IMU auxiliary data processing thread. */
static THD_WORKING_AREA(wa_imu_aux_thread, IMU_AUX_THREAD_STACK);

/** IMU interrupt request semaphore. Signals to the IMU data processing thread
 * that an IMU interrupt has occured. */
static BSEMAPHORE_DECL(imu_irq_sem, TRUE);

/** Lower 32 bits of the timing count, stored by the ISR when an IMU interrupt
 * occurs so the processing thread can recover the precise time of the
 * interrupt. */
static u32 nap_tc;

/* Settings */

static imu_rate_t imu_rate = IMU_RATE_50HZ;
static bool raw_imu_output = false;
static u8 acc_range = 0;
static bmi160_gyr_range_t gyr_range = BMI160_GYR_1000DGS;

/** Interrupt service routine for the IMU_INT1 interrupt.
 * Records the time and then flags the IMU data processing thread to wake up. */
static void imu_isr(void *context)
{
  (void)context;
  chSysLockFromISR();

  /* Record the NAP timing count in the interrupt to minimize latency.
   * Note that this is just the 32 LSBs, we will recover the full 64 bit value
   * in the processing thread. */
  nap_tc = NAP->TIMING_COUNT;

  /* Wake up processing thread */
  chBSemSignalI(&imu_irq_sem);

  chSysUnlockFromISR();
}

/** IMU auxiliary data processing thread. */
static void imu_aux_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("IMU aux");

  msg_imu_aux_t imu_aux;
  (void)imu_aux;

  while (TRUE) {
    if (raw_imu_output) {
      imu_aux.imu_type = 0; /* Bosch BMI160 */
      imu_aux.temp = bmi160_read_temp();
      imu_aux.imu_conf = (gyr_range << 4) | acc_range;

      /* Send out IMU_AUX SBP message. */
      sbp_send_msg(SBP_MSG_IMU_AUX, sizeof(imu_aux), (u8*)&imu_aux);
    }

    chThdSleepMilliseconds(1000);
  }
}

/** IMU data processing thread. */
static void imu_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("IMU");

  s16 acc[3];
  s16 gyro[3];
  s16 mag[3];
  u32 sensor_time;
  msg_imu_raw_t imu_raw;

  while (TRUE) {

    /* Wait until an IMU interrupt occurs. */
    chBSemWait(&imu_irq_sem);

    bool new_acc, new_gyro, new_mag;
    bmi160_new_data_available(&new_acc, &new_gyro, &new_mag);

    bmi160_get_data(acc, gyro, mag, &sensor_time);

    if (new_mag) {
      /* TODO: Magnetometer */
    }

    if (new_acc != new_gyro) {
      log_debug("Accelerometer and Gyro not both ready %u %u\n",
                new_acc, new_gyro);
      continue;
    }

    if (new_acc && new_gyro) {

      /* Read out the IMU data and fill out the SBP message. */
      imu_raw.acc_x = acc[0];
      imu_raw.acc_y = acc[1];
      imu_raw.acc_z = acc[2];
      imu_raw.gyr_x = gyro[0];
      imu_raw.gyr_y = gyro[1];
      imu_raw.gyr_z = gyro[2];

      /* Recover the full 64 bit timing count from the 32 LSBs
       * captured in the ISR. */
      u64 tc = nap_sample_time_to_count(nap_tc);

      /* Calculate the GPS time of the observation, if possible. */
      if (time_quality >= TIME_FINE) {
        /* We know the GPS time to high accuracy, this allows us to convert a
         * timing count value into a GPS time. */
        gps_time_t t = napcount2gpstime(tc);

        /* Format the time of week as a fixed point value for the SBP message. */
        double tow_ms = t.tow * 1000;
        imu_raw.tow = (u32)tow_ms;
        imu_raw.tow_f = (u8)round((tow_ms - imu_raw.tow)*255);
      } else {
        /* Time is unknown, make it as invalid in the SBP message. */
        imu_raw.tow = (1 << 31);
        imu_raw.tow_f = 0;
      }

      /* Send out IMU_RAW SBP message. */
      sbp_send_msg(SBP_MSG_IMU_RAW, sizeof(imu_raw), (u8*)&imu_raw);
    }
  }
}

static bool imu_rate_changed(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    bmi160_set_imu_rate(imu_rate);
    return true;
  }
  return false;
}

static bool raw_imu_output_changed(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    bmi160_imu_set_enabled(raw_imu_output);
    return true;
  }
  return false;
}

static bool acc_range_changed(struct setting *s, const char *val)
{
  if (!s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return false;
  }

  /* Convert between the setting value, which is an integer corresponding to
   * the index of the selected setting in the list of strings, and the relevant
   * enum values */
  switch (acc_range) {
  case 0: /* 2g */
    bmi160_set_acc_range(BMI160_ACC_2G);
    break;
  case 1: /* 4g */
    bmi160_set_acc_range(BMI160_ACC_4G);
    break;
  case 2: /* 8g */
    bmi160_set_acc_range(BMI160_ACC_8G);
    break;
  case 3: /* 16g */
    bmi160_set_acc_range(BMI160_ACC_16G);
    break;
  default:
    log_error("Unexpected accelerometer range setting: %u", acc_range);
    break;
  }

  return true;
}

static bool gyr_range_changed(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    /* The settings values and the enum values correspond numerically so no
     * need to convert between them. */
    bmi160_set_gyr_range(gyr_range);
    return true;
  }
  return false;
}

void imu_init(void)
{
  bmi160_init();

  SETTING_NOTIFY("imu", "imu_raw_output", raw_imu_output, TYPE_BOOL,
                 raw_imu_output_changed);

  static const char const *rate_enum[] =
    /* TODO: 400 Hz mode disabled for now as at that speed there is a timing
     * issue resulting in messages with duplicate timestamps. */
    {"25", "50", "100", "200", /* "400",*/ NULL};
  static struct setting_type rate_setting;
  int TYPE_RATE = settings_type_register_enum(rate_enum, &rate_setting);
  SETTING_NOTIFY("imu", "imu_rate", imu_rate, TYPE_RATE, imu_rate_changed);

  static const char const *acc_range_enum[] =
    {"2g", "4g", "8g", "16g", NULL};
  static struct setting_type acc_range_setting;
  int TYPE_ACC_RANGE = settings_type_register_enum(acc_range_enum,
                                                   &acc_range_setting);
  SETTING_NOTIFY("imu", "acc_range", acc_range, TYPE_ACC_RANGE,
                 acc_range_changed);

  static const char const *gyr_range_enum[] =
    {"2000", "1000", "500", "250", "125", NULL};
  static struct setting_type gyr_range_setting;
  int TYPE_GYR_RANGE = settings_type_register_enum(gyr_range_enum,
                                                   &gyr_range_setting);
  SETTING_NOTIFY("imu", "gyro_range", gyr_range, TYPE_GYR_RANGE,
                 gyr_range_changed);

  chThdCreateStatic(wa_imu_thread, sizeof(wa_imu_thread),
                    IMU_THREAD_PRIO, imu_thread, NULL);
  chThdCreateStatic(wa_imu_aux_thread, sizeof(wa_imu_aux_thread),
                    IMU_AUX_THREAD_PRIO, imu_aux_thread, NULL);

  /* Enable IMU INT1 interrupt */
  gic_handler_register(IRQ_ID_IMU_INT1, imu_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_IMU_INT1, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_IMU_INT1, IMU_INT1_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_IMU_INT1);
}

