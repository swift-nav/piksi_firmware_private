/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Jacob McNamee <jacob@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtc_m41t62.h"
#include "rtc_m41t62_regs.h"

#include <ch.h>
#include <hal.h>
#include <libswiftnav/logging.h>

#include <string.h>

#define RTC_I2C_ADDR      0x68
#define RTC_I2C_TIMEOUT   MS2ST(100)
#define RTC_IRQ_TIMEOUT   MS2ST(1500)
#define RTC_RS_VALUE      RTC_M41T62_RS_32k

#define ONES(value)     (((value) /   1) % 10)
#define TENS(value)     (((value) /  10) % 10)
#define HUNDREDS(value) (((value) / 100) % 10)

static const I2CConfig rtc_i2c_config = RTC_I2C_CONFIG;

static BSEMAPHORE_DECL(rtc_irq_bsem, 0);

/** Lock and start the I2C driver.
 */
static void i2c_open(void)
{
  i2cAcquireBus(&RTC_I2C);
  i2cStart(&RTC_I2C, &rtc_i2c_config);
}

/** Unlock and stop the I2C driver.
 */
static void i2c_close(void)
{
  i2cStop(&RTC_I2C);
  i2cReleaseBus(&RTC_I2C);
}

/** Perform an I2C read operation.
 *
 * \param addr          Register address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
static msg_t i2c_read(u8 addr, u8 *data, size_t length)
{
  return i2cMasterTransmitTimeout(&RTC_I2C, RTC_I2C_ADDR,
                                  &addr, 1, data, length,
                                  RTC_I2C_TIMEOUT);
}

/** Perform an I2C write operation.
 *
 * \param addr          Register address.
 * \param data          Input data buffer.
 * \param length        Number of bytes to write.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
static msg_t i2c_write(u8 addr, const u8 *data, size_t length)
{
 u8 buf[1 + length];
 buf[0] = addr;
 memcpy(&buf[1], data, length);
 return i2cMasterTransmitTimeout(&RTC_I2C, RTC_I2C_ADDR,
                                 buf, sizeof(buf), NULL, 0,
                                 RTC_I2C_TIMEOUT);
}

/** Perform an I2C read transaction.
 *
 * \note This function handles starting and stopping the I2C driver internally.
 *
 * \param addr          Register address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
static msg_t i2c_read_txn(u8 addr, u8 *data, size_t length)
{
  msg_t msg;
  i2c_open();
  {
    msg = i2c_read(addr, data, length);
  }
  i2c_close();
  return msg;
}

/** Perform an I2C write transaction.
 *
 * \note This function handles starting and stopping the I2C driver internally.
 *
 * \param addr          Register address.
 * \param data          Input data buffer.
 * \param length        Number of bytes to write.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
static msg_t i2c_write_txn(u8 addr, const u8 *data, size_t length)
{
  msg_t msg;
  i2c_open();
  {
    msg = i2c_write(addr, data, length);
  }
  i2c_close();
  return msg;
}

/** Write the RTC alarm registers.
 *
 * \param square_wave_enable    Square wave output enable.
 *
 * \return true if the operation succeeded, false otherwise.
 */
static bool alarm_regs_write(bool square_wave_enable)
{
  rtc_m41t62_alarm_regs_t alarm_regs;
  memset(&alarm_regs, 0, sizeof(alarm_regs));

  alarm_regs.SQWE = square_wave_enable ? 1 : 0;
  alarm_regs.AFE = 1;
  alarm_regs.RPT5 = RTC_M41T62_RPT5(RTC_M41T62_RPT_SECOND);
  alarm_regs.RPT4 = RTC_M41T62_RPT4(RTC_M41T62_RPT_SECOND);
  alarm_regs.RPT3 = RTC_M41T62_RPT3(RTC_M41T62_RPT_SECOND);
  alarm_regs.RPT2 = RTC_M41T62_RPT2(RTC_M41T62_RPT_SECOND);
  alarm_regs.RPT1 = RTC_M41T62_RPT1(RTC_M41T62_RPT_SECOND);

  return (i2c_write_txn(RTC_M41T62_REG_ALARM_START,
                       (const u8 *)&alarm_regs, sizeof(alarm_regs)) == MSG_OK);
}

/** Prepare and wait for an alarm IRQ aligned to a one second boundary.
 *
 * \note This function enables the square wave output.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
static bool second_wait(void)
{
  /* Enable square wave output */
  if (!alarm_regs_write(true)) {
    return false;
  }

  /* Ensure that only one interrupt can occur after calling irq_wait_setup()
   * so that the square wave output may be counted in a robust manner. This
   * requires clearing the IRQ flags first, which makes it possible to miss
   * the interrupt. Try a few times to mitigate this condition.
   */
  u8 tries = 3;
  do {
    /* Make sure semaphore is in the TAKEN state */
    chBSemWaitTimeout(&rtc_irq_bsem, TIME_IMMEDIATE);

    /* Read flags to move address pointer and release IRQ line */
    rtc_m41t62_flags_reg_t flags_reg;
    if (i2c_read_txn(RTC_M41T62_REG_FLAGS,
                     (u8 *)&flags_reg, sizeof(flags_reg)) != MSG_OK) {
      return false;
    }

    /* Make sure semaphore is still in the TAKEN state. Otherwise an IRQ
     * has already occurred and the sequence must be retried.
     */
    if (chBSemWaitTimeout(&rtc_irq_bsem, TIME_IMMEDIATE) == MSG_TIMEOUT) {
      break;
    }

  } while(--tries > 0);

  if (tries == 0) {
    return false;
  }

  /* Wait for IRQ semaphore */
 return (chBSemWaitTimeout(&rtc_irq_bsem, RTC_IRQ_TIMEOUT) == MSG_OK);
}

/** Clean up after waiting for a one second boundary.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
static bool second_wait_cleanup(void)
{
  /* Disable square wave output */
  return alarm_regs_write(false);
}

/** Stop and restart the clock oscillator.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
static bool clock_reset(void)
{
  rtc_m41t62_clock_regs_t clock_regs;
  memset(&clock_regs, 0, sizeof(clock_regs));

  /* Write ST = 1 */
  clock_regs.ST = 1;
  if (i2c_write_txn(RTC_M41T62_REG_CLOCK_START,
                    (const u8 *)&clock_regs, sizeof(clock_regs)) != MSG_OK) {
    return false;
  }

  /* Write ST = 0 */
  clock_regs.ST = 0;
  if (i2c_write_txn(RTC_M41T62_REG_CLOCK_START,
                    (const u8 *)&clock_regs, sizeof(clock_regs)) != MSG_OK) {
    return false;
  }

  return true;
}

/** Handle an RTC IRQ.
 */
static void rtc_m41t62_irq_handler(void *context)
{
  (void)context;

  /* Signal IRQ semaphore */
  chSysLockFromISR();
  chBSemSignalI(&rtc_irq_bsem);
  chSysUnlockFromISR();
}

/** Initialize the RTC.
 */
void rtc_m41t62_init(void)
{
  /* Read all registers */
  rtc_m41t62_regs_t regs;
  u8 tries = 3;
  do {
    if (i2c_read_txn(0, (u8 *)&regs, sizeof(regs)) == MSG_OK) {
      break;
    }
    chThdSleepMilliseconds(1);
  } while(--tries > 0);

  if (tries == 0) {
    log_warn("Could not read RTC registers at powerup");
  } else {

    /* Verify clock registers */
    if ((regs.clock_regs.ST != 0) ||
        (regs.clock_regs.OFIE != 0) ||
        (regs.clock_regs.RS != RTC_RS_VALUE)) {
      /* Reset clock, force oscillator fail bit to be set */
      clock_reset();
    }

    /* Verify calibration register */
    if ((regs.calibration_reg.calibration != 0) ||
        (regs.calibration_reg.S != 0) ||
        (regs.calibration_reg.OUT != 1)) {
      /* Set calibration to zero, OUT high */
      rtc_m41t62_calibration_reg_t calibration_reg;
      memset(&calibration_reg, 0, sizeof(calibration_reg));
      calibration_reg.OUT = 1;
      i2c_write_txn(RTC_M41T62_REG_CALIBRATION,
                    (const u8 *)&calibration_reg, sizeof(calibration_reg));
    }

    /* Verify alarm registers */
    if ((regs.alarm_regs.SQWE != 0) ||
        (regs.alarm_regs.AFE != 1) ||
        (regs.alarm_regs.RPT5 != RTC_M41T62_RPT5(RTC_M41T62_RPT_SECOND)) ||
        (regs.alarm_regs.RPT4 != RTC_M41T62_RPT4(RTC_M41T62_RPT_SECOND)) ||
        (regs.alarm_regs.RPT3 != RTC_M41T62_RPT3(RTC_M41T62_RPT_SECOND)) ||
        (regs.alarm_regs.RPT2 != RTC_M41T62_RPT2(RTC_M41T62_RPT_SECOND)) ||
        (regs.alarm_regs.RPT1 != RTC_M41T62_RPT1(RTC_M41T62_RPT_SECOND)) ||
        (regs.alarm_regs.month_ones != 0) ||
        (regs.alarm_regs.month_tens != 0) ||
        (regs.alarm_regs.mday_ones != 0) ||
        (regs.alarm_regs.mday_tens != 0) ||
        (regs.alarm_regs.hour_ones != 0) ||
        (regs.alarm_regs.hour_tens != 0) ||
        (regs.alarm_regs.minute_ones != 0) ||
        (regs.alarm_regs.minute_tens != 0) ||
        (regs.alarm_regs.second_ones != 0) ||
        (regs.alarm_regs.second_tens != 0)) {
      /* Configure 1s alarm with square wave disabled */
      alarm_regs_write(false);
    }
  }

  /* Configure IRQ */
  gic_handler_register(IRQ_ID_RTC, rtc_m41t62_irq_handler, NULL);
  gic_irq_sensitivity_set(IRQ_ID_RTC, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_RTC, RTC_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_RTC);
}

/** Set the RTC time.
 *
 * \note This function also clears the oscillator fail flag.
 * \note The centiseconds field is ignored.
 *
 * \param time          Time to set.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
bool rtc_m41t62_time_set(const rtc_m41t62_time_t *time)
{
  rtc_m41t62_clock_regs_t clock_regs;
  memset(&clock_regs, 0, sizeof(clock_regs));

  /* Note: It is not possible to set the centiseconds value */
  clock_regs.second_ones =    ONES(time->second);
  clock_regs.second_tens =    TENS(time->second);
  clock_regs.minute_ones =    ONES(time->minute);
  clock_regs.minute_tens =    TENS(time->minute);
  clock_regs.hour_ones =      ONES(time->hour);
  clock_regs.hour_tens =      TENS(time->hour);
  clock_regs.wday_ones =      ONES(time->wday);
  clock_regs.mday_ones =      ONES(time->mday);
  clock_regs.mday_tens =      TENS(time->mday);
  clock_regs.month_ones =     ONES(time->month);
  clock_regs.month_tens =     TENS(time->month);
  clock_regs.century_ones =   HUNDREDS(time->year);
  clock_regs.year_ones =      ONES(time->year);
  clock_regs.year_tens =      TENS(time->year);

  clock_regs.ST = 0;
  clock_regs.OFIE = 0;
  clock_regs.RS = RTC_RS_VALUE;

  rtc_m41t62_flags_reg_t flags_reg;
  memset(&flags_reg, 0, sizeof(flags_reg));

  /* Write clock */
  if (i2c_write_txn(RTC_M41T62_REG_CLOCK_START,
                    (const u8 *)&clock_regs, sizeof(clock_regs)) != MSG_OK) {
    return false;
  }

  /* Write flags to make sure oscillator fail is cleared */
  if (i2c_write_txn(RTC_M41T62_REG_FLAGS,
                    (const u8 *)&flags_reg, sizeof(flags_reg)) != MSG_OK) {
    return false;
  }

  return true;
}

/** Get the RTC time.
 *
 * \param time          Output time.
 * \param valid         Validity status of output time.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
bool rtc_m41t62_time_get(rtc_m41t62_time_t *time, bool *valid)
{
  /* Read clock */
  rtc_m41t62_clock_regs_t clock_regs;
  if (i2c_read_txn(RTC_M41T62_REG_CLOCK_START,
                   (u8 *)&clock_regs, sizeof(clock_regs)) != MSG_OK) {
    return false;
  }

  /* Read flags */
  rtc_m41t62_flags_reg_t flags_reg;
  if (i2c_read_txn(RTC_M41T62_REG_FLAGS,
                   (u8 *)&flags_reg, sizeof(flags_reg)) != MSG_OK) {
    return false;
  }

  /* Check oscillator fail flag */
  *valid = flags_reg.OF ? false : true;

  time->centisecond =   10 * clock_regs.centisecond_tens +
                         1 * clock_regs.centisecond_ones;
  time->second =        10 * clock_regs.second_tens +
                         1 * clock_regs.second_ones;
  time->minute =        10 * clock_regs.minute_tens +
                         1 * clock_regs.minute_ones;
  time->hour =          10 * clock_regs.hour_tens +
                         1 * clock_regs.hour_ones;
  time->wday =           1 * clock_regs.wday_ones;
  time->mday =          10 * clock_regs.mday_tens +
                         1 * clock_regs.mday_ones;
  time->month =         10 * clock_regs.month_tens +
                         1 * clock_regs.month_ones;
  time->year =         100 * clock_regs.century_ones +
                        10 * clock_regs.year_tens +
                         1 * clock_regs.year_ones;

  return true;
}

/** Wait for a one second boundary.
 *
 * \note This function enables the square wave output.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
bool rtc_m41t62_second_wait(void)
{
  return second_wait();
}

/** Clean up after waiting for a one second boundary.
 *
 * \note This function disables the square wave output.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
bool rtc_m41t62_second_wait_cleanup(void)
{
  u8 tries = 3;
  do {
    if (second_wait_cleanup()) {
      return true;
    }
    chThdSleepMilliseconds(1);
  } while(--tries > 0);

  return false;
}

/** Restart the clock oscillator.
 *
 * \return true if the operation was completed successfully, false otherwise.
 */
bool rtc_m41t62_oscillator_restart(void)
{
  return clock_reset();
}
