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
#include "settings/settings.h"
#include "timing/timing.h"

#include <board/nap/nap_common.h>
#include <board/v3/peripherals/bmi160.h>
#include <ch.h>
#include <hal.h>
#include <libsbp/imu.h>
#include <libsbp/mag.h>
#include <libswiftnav/gnss_time.h>
#include <libswiftnav/logging.h>
#include <math.h>
#include <sbp.h>

#define IMU_THREAD_PRIO (HIGHPRIO - 1)
#define IMU_THREAD_STACK (2 * 1024)
#define IMU_AUX_THREAD_PRIO (LOWPRIO + 10)
#define IMU_AUX_THREAD_STACK (2 * 1024)

#define IMU_RATE_MAX 200.0f
#define MAG_RATE_MAX 25.0f

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

/* rate = 25 * 2 ^ (imu_rate - 6) valid for 12 > imu_rate > 0 */
static bool rate_enum_to_hz(const u8 rate_e, float *rate_hz) {
  if ((rate_e == 0) || (rate_e >= 12)) return false;
  (*rate_hz) = 25.0f * powf(2.0f, rate_e - 6.0f);
  return true;
}
static bool rate_hz_to_enum(const float rate_hz, u8 *rate_e) {
  if (rate_hz < 0) return false;
  u8 rate_tmp = rintf(log2f(rate_hz / 25.0)) + 6;
  if ((rate_tmp == 0) || (rate_tmp >= 12)) return false;
  (*rate_e) = rate_tmp;
  return true;
}

/* Settings */
static float imu_rate_hz = 50.0f;
static bool raw_imu_output = false;
static u8 acc_range = 2;
static bmi160_gyr_range_t gyr_range = BMI160_GYR_1000DGS;
static float mag_rate_hz = 12.5f;
static bool raw_mag_output = false;

/* rate_change_in_progress indicates that imu rate has recently been changed.
 * Set by settings callback. Unset the first time imu_thread is awoken after
 * change. Used to suppress warnings when IMU has just been reconfigured.
 */

static bool rate_change_in_progress = false;

/** Interrupt service routine for the IMU_INT1 interrupt.
 * Records the time and then flags the IMU data processing thread to wake up. */
static void imu_isr(void *context) {
  (void)context;
  chSysLockFromISR();

  /* Record the NAP timing count in the interrupt to minimize latency.
   * Note that this is just the 32 LSBs, we will recover the full 64 bit value
   * in the processing thread.
   */
  nap_tc = NAP->TIMING_COUNT;

  /* Wake up processing thread */
  if (imu_irq_sem.bs_sem.s_cnt == 1) {
    /* If cnt == 1, interrupt was received again before imu_thread was awaken.
     * Use semaphore reset to communicate to imu_thread that it missed an IRQ.
     */
    chBSemResetI(&imu_irq_sem, 1);
  } else {
    chBSemSignalI(&imu_irq_sem);
  }
  chSysUnlockFromISR();
}

static void imu_aux_send(void) {
  msg_imu_aux_t imu_aux;
  imu_aux.imu_type = 0; /* Bosch BMI160 */
  imu_aux.temp = bmi160_read_temp();
  imu_aux.imu_conf = (gyr_range << 4) | acc_range;

  /* Send out IMU_AUX SBP message. */
  sbp_send_msg(SBP_MSG_IMU_AUX, sizeof(imu_aux), (u8 *)&imu_aux);
}

/** IMU auxiliary data processing thread. */
static void imu_aux_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("IMU aux");

  while (TRUE) {
    if (raw_imu_output) {
      imu_aux_send();
    }

    chThdSleepMilliseconds(1000);
  }
}

/** IMU data processing thread. */
static void imu_thread(void *arg) {
  (void)arg;
  chRegSetThreadName("IMU");

  s16 acc[3];
  s16 gyro[3];
  s16 mag[3];
  u32 sensor_time;
  u32 p_sensor_time = 0;
  double p_tow_ms = 0;
  msg_imu_raw_t imu_raw;
  msg_mag_raw_t mag_raw;

  while (TRUE) {
    /* Wait until an IMU interrupt occurs. */
    systime_t timeout = TIME_INFINITE;
    if (raw_imu_output || raw_mag_output) {
      /* If output is enabled and we're actually expecting interrupts, we
       * set a timeout a little after the expected interrupt.
       *
       * This works out because of the numeric values of imu_rate_t:
       * imu_rate = BMI160_RATE_6_25HZ = 4 -> 160ms, so timeout is set to 200ms
       * imu_rate = BMI160_RATE_400HZ = 10 -> 2.5ms,so timeout is set to
       * 3.125ms
       */
      float max_rate_hz;
      if (!raw_imu_output) {
        max_rate_hz = mag_rate_hz;
      } else {
        max_rate_hz = imu_rate_hz;
      }
      timeout = MS2ST((1200.0f / max_rate_hz));
    }
    s32 ret = chBSemWaitTimeout(&imu_irq_sem, timeout);
    if (ret == MSG_TIMEOUT) {
      if (!rate_change_in_progress) {
        log_info("IMU IRQ not received before timeout.");
      }
      rate_change_in_progress = false;
      continue;
    } else if (ret == MSG_RESET) {
      log_warn("IMU frame overrun. IMU not serviced before next interrupt.");
      /* In the case of a frame overrun, we still need to read from IMU to tell
       * it to take next sample. We expect that the nap_timing_count from ISR
       * represents that of the latest sample (the 2nd interrupt received)
       */
    }

    bool new_acc, new_gyro, new_mag;
    float expected_dt, dt, dt_err_pcent; /* Timing instrumentation */
    expected_dt = 1.0f / imu_rate_hz;
    bmi160_new_data_available(&new_acc, &new_gyro, &new_mag);

    if (new_acc != new_gyro && raw_imu_output) {
      log_warn("IMU interrupt without both accel and gyro ready: (%u, %u)",
               new_acc,
               new_gyro);
      continue;
    }

    if (!new_acc && !new_gyro && !new_mag) {
      log_warn("IMU interrupt serviced with no data ready.");
      continue;
    }

    /* read data */

    s16 *mag_ptr = (new_mag) ? mag : NULL;
    bmi160_get_data(acc, gyro, mag_ptr, &sensor_time);

    /* Warn if dt error exceeds threshold according to IMU. Ignore
       large errors since there is an undiagnosed discontinuity problem where
       the sensor_time register reads bogus information. The datasheet seems
       to imply that this register is shadowed.
    */

    if (raw_imu_output) {
      dt = (sensor_time - p_sensor_time) * BMI160_SENSOR_TIME_TO_SECONDS;
      dt_err_pcent = (fabsf(dt - expected_dt) / expected_dt * 100.0f);

      if (!rate_change_in_progress &&
          dt_err_pcent > BMI160_DT_ERR_THRESH_PERCENT &&
          dt_err_pcent < BMI160_DT_ERR_MAX_PERCENT && p_sensor_time > 0) {
        log_warn("Reported IMU sampling period of %.0f ms (expected %.0f) ",
                 dt * 1000.0f,
                 expected_dt * 1000.0f);
        log_warn("IMU Error register: %u. IMU status register: %u",
                 bmi160_read_error(),
                 bmi160_read_status());
      }
      p_sensor_time = sensor_time;
      if (rate_change_in_progress) {
        p_sensor_time = 0;
      }
    }

    u32 tow;
    u8 tow_f;
    /* Recover the full 64 bit timing count from the 32 LSBs
     * captured in the ISR. */
    u64 tc = nap_sample_time_to_count(nap_tc);

    /* Calculate the GPS time of the observation, if possible. */
    if (get_time_quality() >= TIME_PROPAGATED) {
      /* We know the GPS time to high accuracy, this allows us to convert a
       * timing count value into a GPS time. */
      gps_time_t t = napcount2gpstime(tc);

      /* Format the time of week as a fixed point value for the SBP message.
       */
      double tow_ms = t.tow * 1000;
      tow = (u32)tow_ms;
      tow_f = (u8)round((tow_ms - tow) * 255);

      /* Warn if imu dt error exceeds threshold according to our ME */

      if (raw_imu_output) {
        dt = (tow_ms - p_tow_ms) / 1000.0;
        dt_err_pcent = fabsf(dt - expected_dt) / expected_dt * 100.0f;

        if (!rate_change_in_progress && p_tow_ms != 0 &&
            dt_err_pcent > BMI160_DT_ERR_THRESH_PERCENT) {
          log_warn("Measured IMU sampling period of %.0f ms (expected %.0f ms)",
                   dt * 1000.0f,
                   expected_dt * 1000.0f);
          log_warn("IMU Error register: %u. IMU status register: %u",
                   bmi160_read_error(),
                   bmi160_read_status());
        }

        p_tow_ms = tow_ms;
        if (rate_change_in_progress) {
          p_tow_ms = 0;
        }

        /* Warn if sensor read delay after ISR exceeds threshold */

        u64 tc_now = nap_sample_time_to_count(NAP->TIMING_COUNT);
        gps_time_t t_now = napcount2gpstime(tc_now);
        dt = gpsdifftime(&t_now, &t);
        dt_err_pcent =
            dt / expected_dt * 100.0f; /* Delay's proportion of period */
        if (dt_err_pcent > BMI160_READ_DELAY_THRESH_PERCENT) {
          log_warn("IMU read delay of %.0f ms (%.0f %% of period)",
                   dt * 1000.0,
                   dt_err_pcent);
          log_warn("IMU Error register: %u. IMU status register: %u",
                   bmi160_read_error(),
                   bmi160_read_status());
        }
      }
    } else {
      /* Time is unknown, make it as invalid in the SBP message. */
      tow = (1 << 31);
      tow_f = 0;
    }

    if (new_acc && new_gyro && raw_imu_output) {
      /* Read out the IMU data and fill out the SBP message. */
      imu_raw.acc_x = acc[1];
      imu_raw.acc_y = acc[0];
      imu_raw.acc_z = -acc[2];
      imu_raw.gyr_x = gyro[1];
      imu_raw.gyr_y = gyro[0];
      imu_raw.gyr_z = -gyro[2];
      imu_raw.tow = tow;
      imu_raw.tow_f = tow_f;

      /* Send out IMU_RAW SBP message. */
      sbp_send_msg(SBP_MSG_IMU_RAW, sizeof(imu_raw), (u8 *)&imu_raw);
    }
    if (new_mag && raw_mag_output) {
      /* Read out the magnetometer data and fill out the SBP message. */
      mag_raw.mag_x = mag[1];
      mag_raw.mag_y = -mag[0];
      mag_raw.mag_z = mag[2];
      mag_raw.tow = tow;
      mag_raw.tow_f = tow_f;

      /* Send out MAG_RAW SBP message. */
      sbp_send_msg(SBP_MSG_MAG_RAW, sizeof(mag_raw), (u8 *)&mag_raw);
    }
    rate_change_in_progress = false;
  }
}

static bool imu_rate_changed(struct setting *s, const char *val) {
  if (!s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return false;
  }
  if (imu_rate_hz > IMU_RATE_MAX) return false;
  u8 imu_rate_e;
  if (!rate_hz_to_enum(imu_rate_hz, &imu_rate_e)) {
    log_error("Unexpected IMU rate setting: %.3f Hz", imu_rate_hz);
    return false;
  }
  float imu_rate_tmp = 0.0f;
  rate_enum_to_hz(imu_rate_e, &imu_rate_tmp);
  log_info("validate IMU rate change to %.3f", imu_rate_tmp);
  bmi160_set_imu_rate(imu_rate_e);
  return true;
}

static bool raw_imu_output_changed(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    bmi160_imu_set_enabled(raw_imu_output);
    return true;
  }
  return false;
}

static bool mag_rate_changed(struct setting *s, const char *val) {
  if (!s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return false;
  }
  if (mag_rate_hz > MAG_RATE_MAX) return false;
  u8 mag_rate_e;
  if (!rate_hz_to_enum(mag_rate_hz, &mag_rate_e)) {
    log_error("Unexpected magnetometer rate setting: %.3f", mag_rate_hz);
    return false;
  }
  float mag_rate_tmp = 0.0f;
  rate_enum_to_hz(mag_rate_e, &mag_rate_tmp);
  log_info("validate IMU rate change to %.3f", mag_rate_tmp);
  bmi160_set_mag_rate(mag_rate_e);
  return true;
}

static bool raw_mag_output_changed(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    bmi160_mag_set_enabled(raw_mag_output);
    return true;
  }
  return false;
}

static bool acc_range_changed(struct setting *s, const char *val) {
  if (!s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    return false;
  }

  u8 output = raw_imu_output;
  raw_imu_output = false;

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

  if (output) {
    imu_aux_send();
    raw_imu_output = true;
  }

  return true;
}

static bool gyr_range_changed(struct setting *s, const char *val) {
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    u8 output = raw_imu_output;
    raw_imu_output = false;

    /* The settings values and the enum values correspond numerically so no
     * need to convert between them. */
    bmi160_set_gyr_range(gyr_range);

    if (output) {
      imu_aux_send();
      raw_imu_output = true;
    }

    return true;
  }
  return false;
}

void imu_init(void) {
  bmi160_init();

  SETTING_NOTIFY("imu",
                 "imu_raw_output",
                 raw_imu_output,
                 TYPE_BOOL,
                 raw_imu_output_changed);

  SETTING_NOTIFY("imu", "imu_rate", imu_rate_hz, TYPE_FLOAT, imu_rate_changed);

  static const char *const acc_range_enum[] = {"2g", "4g", "8g", "16g", NULL};
  static struct setting_type acc_range_setting;
  int TYPE_ACC_RANGE =
      settings_type_register_enum(acc_range_enum, &acc_range_setting);
  SETTING_NOTIFY(
      "imu", "acc_range", acc_range, TYPE_ACC_RANGE, acc_range_changed);

  static const char *const gyr_range_enum[] = {
      "2000", "1000", "500", "250", "125", NULL};
  static struct setting_type gyr_range_setting;
  int TYPE_GYR_RANGE =
      settings_type_register_enum(gyr_range_enum, &gyr_range_setting);
  SETTING_NOTIFY(
      "imu", "gyro_range", gyr_range, TYPE_GYR_RANGE, gyr_range_changed);

  SETTING_NOTIFY("imu",
                 "mag_raw_output",
                 raw_mag_output,
                 TYPE_BOOL,
                 raw_mag_output_changed);

  SETTING_NOTIFY("imu", "mag_rate", mag_rate_hz, TYPE_FLOAT, mag_rate_changed);

  chThdCreateStatic(
      wa_imu_thread, sizeof(wa_imu_thread), IMU_THREAD_PRIO, imu_thread, NULL);
  chThdCreateStatic(wa_imu_aux_thread,
                    sizeof(wa_imu_aux_thread),
                    IMU_AUX_THREAD_PRIO,
                    imu_aux_thread,
                    NULL);

  /* Enable IMU INT1 interrupt */
  gic_handler_register(IRQ_ID_IMU_INT1, imu_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_IMU_INT1, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_IMU_INT1, IMU_INT1_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_IMU_INT1);
}
