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

#include <libsbp/imu.h>
#include <libsbp/mag.h>
#include <libsbp/system.h>
#include <math.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/logging.h>

#include "board/nap/nap_common.h"
#include "board/v3/peripherals/bmi160.h"
#include "calc/starling_integration.h"
#include "ch.h"
#include "hal.h"
#include "hal/is_duro.h"
#include "sbp/sbp.h"
#include "settings/settings_client.h"
#include "timing/timing.h"

#define IMU_THREAD_PRIO (HIGHPRIO - 1)
#define IMU_THREAD_STACK (2 * 1024)
#define IMU_AUX_THREAD_PRIO (LOWPRIO + 10)
#define IMU_AUX_THREAD_STACK (2 * 1024)

#define TIME_STATUS_SYSTEM_STARTUP (0x40000000)
#define MS_US (1000)
#define GNSS_OFFSET_SEND_INTERVAL_MS (1000)

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

/* this lookuptable is used to translate enum values of imu_rate setting
 * to the period in seconds expected for this setting value */

static const double BMI160_DT_LOOKUP[] = {0.04,   /* 25 Hz */
                                          0.02,   /* 50 Hz */
                                          0.01,   /* 100 Hz */
                                          0.005}; /* 200 Hz */

/* Settings */
static u8 imu_rate = 2; /* 100 Hz */
static bool raw_imu_output = false;
static u8 acc_range = 2;
static bmi160_gyr_range_t gyr_range = BMI160_GYR_125DGS;
static u8 mag_rate = 1;
static bool raw_mag_output = false;

/* Now that the imu rate is linked to the ins freq rate, need to
 * monitor the ins rate here as well. Ideally INS fusion engine
 * should not be linked with the imu.
 */
static double ins_solution_frequency = 0.;

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
  nap_tc = nap_timing_count_low();

  /* Wake up processing thread */
  if (imu_irq_sem.bs_sem.s_cnt == 1) {
    /* If cnt == 1, interrupt was received again before imu_thread was awoken.
     * Use semaphore reset to communicate to imu_thread that it misssed an IRQ.
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
  sbp_send_imu_msg(SBP_MSG_IMU_AUX, sizeof(imu_aux), (u8 *)&imu_aux);
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
  double gnss_p_tow_ms = 0;
  double local_p_tow_ms = 0;
  double gnss_offset_sent_p_ms = 0;
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
      u32 max_rate;
      /* Only drive timeout from mag if IMU is disabled
       * since max mag rate == min imu rate
       */
      if (!raw_imu_output) {
        /* Setting enum offset by 4 from rate enum for mag */
        max_rate = mag_rate + 4;
      } else {
        /* Setting enum offset by 6 from rate enum for imu */
        max_rate = imu_rate + 6;
      }
      timeout = MS2ST(200) / (1 << (max_rate - 4));
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
    double expected_dt, dt, dt_err_pcent; /* Timing instrumentation */
    expected_dt = BMI160_DT_LOOKUP[imu_rate];
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

    /* Warn if dt error exceeds threshhold according to IMU. Ignore
       large errors since there is an undiagnosed discontinuity problem where
       the sensor_time register reads bogus information. The datasheet seems
       to imply that this register is shadowed.
    */

    if (raw_imu_output) {
      dt = (sensor_time - p_sensor_time) * BMI160_SENSOR_TIME_TO_SECONDS;
      dt_err_pcent = (fabs(dt - expected_dt) / expected_dt * 100.0);

      if (!rate_change_in_progress &&
          dt_err_pcent > BMI160_DT_ERR_THRESH_PERCENT &&
          dt_err_pcent < BMI160_DT_ERR_MAX_PERCENT && p_sensor_time > 0) {
        log_warn("Reported IMU sampling period of %.0f ms (expected %.0f) ",
                 dt * 1000.0,
                 expected_dt * 1000.0);
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
    gps_time_t sample_time = GPS_TIME_UNKNOWN;
    /* Recover the full 64 bit timing count from the 32 LSBs
     * captured in the ISR. */
    u64 tc = nap_sample_time_to_count(nap_tc);
    double time_from_start = tc * (1 - get_clock_drift()) * RX_DT_NOMINAL;
    // Convert local time to ms and wrap local timestamp around 1 week or
    // 604800000 milliseconds
    double time_from_start_ms = fmod((time_from_start * SECS_MS), WEEK_MS);
    // Set time status to time of system startup
    tow = (u32)time_from_start_ms | TIME_STATUS_SYSTEM_STARTUP;
    // Set fractional part according to specification.
    tow_f = (u8)lrint((time_from_start_ms - (u32)time_from_start_ms) * 256);

    // Log local time wrap around and adjust gnss_offset_sent_p_ms;
    if (time_from_start_ms < local_p_tow_ms) {
      gnss_offset_sent_p_ms = gnss_offset_sent_p_ms - WEEK_MS;
      log_warn("Local time wrap around, current: %fms, previous: %fms",
               time_from_start_ms,
               local_p_tow_ms);
    }

    local_p_tow_ms = time_from_start_ms;

    /* Calculate the GPS time of the observation, if possible. */
    if (get_time_quality() >= TIME_PROPAGATED) {
      /* We know the GPS time to high accuracy, this allows us to convert a
       * timing count value into a GPS time. */
      sample_time = napcount2gpstime(tc);
      double gnss_tow_ms = sample_time.tow * SECS_MS;

      /* Warn if imu dt error exceeds threshhold according to our ME */
      if (raw_imu_output) {
        dt = (gnss_tow_ms - gnss_p_tow_ms) / 1000.0;
        dt_err_pcent = fabs(dt - expected_dt) / expected_dt * 100.0;

        if (!rate_change_in_progress && gnss_p_tow_ms != 0 &&
            dt_err_pcent > BMI160_DT_ERR_THRESH_PERCENT) {
          log_warn("Measured IMU sampling period of %.0f ms (expected %.0f ms)",
                   dt * 1000.0,
                   expected_dt * 1000.0);
          log_warn("IMU Error register: %u. IMU status register: %u",
                   bmi160_read_error(),
                   bmi160_read_status());
        }
        gnss_p_tow_ms = gnss_tow_ms;
        if (rate_change_in_progress) {
          gnss_p_tow_ms = 0;
        }

        /* Warn if sensor read delay after ISR exceeds threshhold */

        u64 tc_now = nap_timing_count();
        gps_time_t t_now = napcount2gpstime(tc_now);
        dt = gpsdifftime(&t_now, &sample_time);
        dt_err_pcent =
            dt / expected_dt * 100.0; /* Delay's proportion of period */
        if (dt_err_pcent > BMI160_READ_DELAY_THRESH_PERCENT) {
          log_warn("IMU read delay of %.0f ms (%.0f %% of period)",
                   dt * 1000.0,
                   dt_err_pcent);
          log_warn("IMU Error register: %u. IMU status register: %u",
                   bmi160_read_error(),
                   bmi160_read_status());
        }

        // calculate and send gnss time offset message.
        if ((time_from_start_ms - gnss_offset_sent_p_ms) >
            GNSS_OFFSET_SEND_INTERVAL_MS) {
          gnss_offset_sent_p_ms = time_from_start_ms;
          // convert local and gnss from milliseconds to microseconds.
          u64 local_tow_us = (u64)(time_from_start_ms * MS_US);
          u64 gnss_tow_us = (u64)(sample_time.tow * SECS_US) +
                            ((u64)sample_time.wn * WEEK_MS * MS_US);
          u64 gnss_time_offset = gnss_tow_us - local_tow_us;

          // Pack and send offset message.
          msg_gnss_time_offset_t msg;
          msg.microseconds = gnss_time_offset % MS_US;
          gnss_time_offset /= MS_US;
          msg.milliseconds = gnss_time_offset % WEEK_MS;
          gnss_time_offset /= WEEK_MS;
          msg.weeks = gnss_time_offset;
          msg.flags = 0;
          sbp_send_msg(SBP_MSG_GNSS_TIME_OFFSET, sizeof(msg), (u8 *)&msg);
        }
      }
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
      sbp_send_imu_msg(SBP_MSG_IMU_RAW, sizeof(imu_raw), (u8 *)&imu_raw);
    }
    if (new_mag && raw_mag_output) {
      /* Read out the magnetometer data and fill out the SBP message. */
      mag_raw.mag_x = mag[1];
      mag_raw.mag_y = -mag[0];
      mag_raw.mag_z = mag[2];
      mag_raw.tow = tow;
      mag_raw.tow_f = tow_f;

      /* Send out MAG_RAW SBP message. */
      sbp_send_imu_msg(SBP_MSG_MAG_RAW, sizeof(mag_raw), (u8 *)&mag_raw);
    }
    rate_change_in_progress = false;
  }
}

static bool check_valid_imu_rate(void *ctx) {
  (void)ctx;
  double imu_rates[] = {25, 50, 100, 250};

  if (imu_rate >= 4) {
    log_error("Unexpected imu rate in settings: %u", imu_rate);
    return false;
  }

  if (ins_solution_frequency > 0 &&
      imu_rates[imu_rate] < 2 * ins_solution_frequency) {
    log_error("IMU rate [%.2f] needs to be twice the ins rate [%.2f]",
              imu_rates[imu_rate],
              ins_solution_frequency);
    /* sbp_log(LOG_ERROR, "IMU rate [%.2f] needs to be twice the ins rate
     * [%.2f]", */
    /*     imu_rates[imu_rate], ins_solution_frequency); */
    return false;
  }

  return true;
}

static int imu_rate_changed(void *ctx) {
  /* Convert between the setting value, which is an integer corresponding to
   * the index of the selected setting in the list of strings, and the relevant
   * enum values */
  rate_change_in_progress = true;

  if (!check_valid_imu_rate(ctx)) {
    return SETTINGS_WR_VALUE_REJECTED;
  }

  switch (imu_rate) {
    case 0: /* 25Hz */
      bmi160_set_imu_rate(BMI160_RATE_25HZ);
      break;
    case 1: /* 50Hz */
      bmi160_set_imu_rate(BMI160_RATE_50HZ);
      break;
    case 2: /* 100Hz */
      bmi160_set_imu_rate(BMI160_RATE_100HZ);
      break;
    case 3: /* 200Hz */
      bmi160_set_imu_rate(BMI160_RATE_200HZ);
      break;
    default:
      log_error("Unexpected imu rate setting: %u", imu_rate);
      return SETTINGS_WR_VALUE_REJECTED;
  }

  return SETTINGS_WR_OK;
}

static int ins_rate_changed(void *ctx) {
  (void)ctx;

  log_info("Ins freq changed to %.2f", ins_solution_frequency);

  return SETTINGS_WR_OK;
}

static int raw_imu_output_changed(void *ctx) {
  (void)ctx;

  bmi160_imu_set_enabled(raw_imu_output);

  return SETTINGS_WR_OK;
}

static int mag_rate_changed(void *ctx) {
  (void)ctx;

  /* Convert between the setting value, which is an integer corresponding to
   * the index of the selected setting in the list of strings, and the relevant
   * enum values */
  switch (mag_rate) {
    case 0: /* 6.25Hz */
      bmi160_set_mag_rate(BMI160_RATE_6_25HZ);
      break;
    case 1: /* 12.5Hz */
      bmi160_set_mag_rate(BMI160_RATE_12_5HZ);
      break;
    case 2: /* 25Hz */
      bmi160_set_mag_rate(BMI160_RATE_25HZ);
      break;
    default:
      log_error("Unexpected magnetometer rate setting: %u", mag_rate);
      return SETTINGS_WR_VALUE_REJECTED;
  }

  return SETTINGS_WR_OK;
}

static int raw_mag_output_changed(void *ctx) {
  (void)ctx;

  bmi160_mag_set_enabled(raw_mag_output);

  return SETTINGS_WR_OK;
}

static int acc_range_changed(void *ctx) {
  (void)ctx;

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

  return SETTINGS_WR_OK;
}

static int gyr_range_changed(void *ctx) {
  (void)ctx;
  u8 output = raw_imu_output;
  raw_imu_output = false;

  /* The settings values and the enum values correspond numerically so no
   * need to convert between them. */
  bmi160_set_gyr_range(gyr_range);

  if (output) {
    imu_aux_send();
    raw_imu_output = true;
  }

  return SETTINGS_WR_OK;
}

void imu_init(void) {
  bmi160_init();

  /* try to grab runtime mode from Linux, and turn on if ins */
  raw_imu_output = (device_is_duro() && ins_is_active());

  SETTING_NOTIFY("imu",
                 "imu_raw_output",
                 raw_imu_output,
                 SETTINGS_TYPE_BOOL,
                 raw_imu_output_changed);

  static const char *const imu_rate_enum[] =
      /* TODO: 400 Hz mode disabled for now as at that speed there is a timing
       * issue resulting in messages with duplicate timestamps. */
      {"25", "50", "100", "200", /* "400",*/ NULL};
  settings_type_t imu_rate_setting;
  settings_api_register_enum(imu_rate_enum, &imu_rate_setting);
  SETTING_NOTIFY(
      "imu", "imu_rate", imu_rate, imu_rate_setting, imu_rate_changed);

  SETTING_WATCH("ins",
                "fused_soln_freq",
                ins_solution_frequency,
                SETTINGS_TYPE_FLOAT,
                ins_rate_changed);

  static const char *const acc_range_enum[] = {"2g", "4g", "8g", "16g", NULL};
  settings_type_t acc_range_setting;
  settings_api_register_enum(acc_range_enum, &acc_range_setting);
  SETTING_NOTIFY(
      "imu", "acc_range", acc_range, acc_range_setting, acc_range_changed);

  static const char *const gyr_range_enum[] = {
      "2000", "1000", "500", "250", "125", NULL};
  settings_type_t gyr_range_setting;
  settings_api_register_enum(gyr_range_enum, &gyr_range_setting);
  SETTING_NOTIFY(
      "imu", "gyro_range", gyr_range, gyr_range_setting, gyr_range_changed);

  SETTING_NOTIFY("imu",
                 "mag_raw_output",
                 raw_mag_output,
                 SETTINGS_TYPE_BOOL,
                 raw_mag_output_changed);

  static const char *const mag_rate_enum[] =
      /* Rates up to 100Hz possible, but not recomended */
      {"6.25", "12.5", "25", NULL};
  settings_type_t mag_rate_setting;
  settings_api_register_enum(mag_rate_enum, &mag_rate_setting);
  SETTING_NOTIFY(
      "imu", "mag_rate", mag_rate, mag_rate_setting, mag_rate_changed);

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
