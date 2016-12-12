/*
 * Copyright (C) 2011-2016 Swift Navigation Inc.
 * Contact: Jonathan Diamond <jonathan@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "bmi160.h"
#include <string.h>
#include <hal.h>
#include <ch.h>
#include <libswiftnav/logging.h>

static const SPIConfig spi_config = IMU_SPI_CONFIG;

#define SPI_READ_MASK (1 << 7)

/** Open and lock the SPI but that the BMI160 is on. */
static void bmi160_open_spi(void)
{
  spiAcquireBus(&IMU_SPI);
  spiStart(&IMU_SPI, &spi_config);
  spiSelect(&IMU_SPI);
  /* TODO: seems to be needed for writes */
  chThdSleepMilliseconds(1);
}

/** Close and release the SPI but that the BMI160 is on. */
static void bmi160_close_spi(void)
{
  spiUnselect(&IMU_SPI);
  spiReleaseBus(&IMU_SPI);
}

/** Write a register value to the BMI160. */
static void bmi160_write_reg(u8 reg, u8 data)
{
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  bmi160_close_spi();
}

/** Read a register value from the BMI160. */
static u8 bmi160_read_reg(u8 reg)
{
  const u8 dummy_data = 0x00;
  const u8 send_buf[2] = {reg | SPI_READ_MASK, dummy_data};
  u8 recv_buf[2];
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  bmi160_close_spi();
  return recv_buf[1];
}

/** Wait for a command to complete on the BMI160. */
static void bmi160_wait_cmd_complete(void)
{
  /* TODO: the check for the command to complete doesnt seem to work,
   * adding forced sleep to avoid dropping commands. Is there a better way? */
  chThdSleepMilliseconds(10);
  while(bmi160_read_reg(BMI160_REG_CMD) != 0);
}

void bmi160_init(void)
{
  /* Delay required to prevent IMU initialization conflicting with the
   * front-end configuration, resulting in no signals being acquired or
   * tracked. */
  /* TODO: Investigate why this delay is required. */
  chThdSleepMilliseconds(1);

  /* Pulse SS to trigger BMI160 to use SPI */
  bmi160_open_spi();
  bmi160_close_spi();
  bmi160_open_spi();
  bmi160_close_spi();

  /* Perform a soft reset of the BMI160 */
  bmi160_write_reg(BMI160_REG_CMD, 0xB6);
  bmi160_wait_cmd_complete();

  /* Check if the ID of the BMI160 is as expected. */
  u8 id = bmi160_read_reg(BMI160_REG_CHIP_ID);
  if (id != BMI160_MFDVID) {
    log_error("IMU: BMI160 ID didn't match expected value (%u)", id);
    return;
  }

  /* Configure IMU_INT1, interrupt on data ready */
  bmi160_write_reg(BMI160_REG_INT_OUT_CTRL, 0b00001011);
  bmi160_write_reg(BMI160_REG_INT_MAP_1, 0b10000000);
  bmi160_write_reg(BMI160_REG_INT_EN_1, 0b00010000);
}

/** Set the IMU (Accels and Gyros) data rate.
 * Doesn't affect the operation of the magnetometer. */
void bmi160_set_imu_rate(imu_rate_t rate)
{
  u8 rate_val = BMI160_IMU_RATE_25HZ + (0xF & (u8)rate);
  bmi160_write_reg(BMI160_REG_ACC_CONF, 0b00100000 | rate_val);
  bmi160_write_reg(BMI160_REG_GYR_CONF, 0b00100000 | rate_val);
}

/** Enable or disable the IMU (Accels and Gyros) in the BMI160.
 * Doesn't affect the operation of the magnetometer. */
void bmi160_imu_set_enabled(bool enabled)
{
  /* Set sensor mode to Normal or Suspended depending on the enabled value. */
  u8 mode = enabled ? 1 : 0;

  /* 0b (PMU) 0001 (accel) 00 (mode) 0? */
  bmi160_write_reg(BMI160_REG_CMD, 0b00010000 | mode);
  bmi160_wait_cmd_complete();

  /* 0b (PMU) 0001 (gyro) 01 (mode) 0? */
  bmi160_write_reg(BMI160_REG_CMD, 0b00010100 | mode);
  bmi160_wait_cmd_complete();
}

/** Set the full scale range of the accelerometer. */
void bmi160_set_acc_range(bmi160_acc_range_t range)
{
  bmi160_write_reg(BMI160_REG_ACC_RANGE, 0xF & (u8)range);
}

/** Set the full scale range of the gyroscope. */
void bmi160_set_gyr_range(bmi160_gyr_range_t range)
{
  bmi160_write_reg(BMI160_REG_GYR_RANGE, 0x7 & (u8)range);
}

/** Check if any new data is available from the sensors. */
void bmi160_new_data_available(bool* new_acc, bool* new_gyro, bool* new_mag)
{
  u8 status = bmi160_read_reg(BMI160_REG_STATUS);
  *new_acc = status & BMI160_STATUS_ACC_RDY_Msk;
  *new_gyro = status & BMI160_STATUS_GYRO_RDY_Msk;
  *new_mag = status & BMI160_STATUS_MAG_RDY_Msk;
}

/** Read the sensor data from the BMI160 and BMM150. */
void bmi160_get_data(s16 acc[static 3], s16 gyro[static 3], s16 mag[static 3],
                     u32* sensor_time)
{
  /* First byte is for register address. All sensors are read together. */
  u8 buf[BMI160_DATA_SIZE + 1];
  buf[0] = BMI160_REG_DATA | SPI_READ_MASK;
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(buf), buf, buf);
  bmi160_close_spi();

  /* Extract data from data buffer */
  memcpy(mag, &buf[1 + BMI160_DATA_MAG_OFFSET], 2 * 3);
  memcpy(gyro, &buf[1 + BMI160_DATA_GYRO_OFFSET], 2 * 3);
  memcpy(acc, &buf[1 + BMI160_DATA_ACC_OFFSET], 2 * 3);
  *sensor_time = 0;
  memcpy(sensor_time, &buf[1 + BMI160_DATA_TIME_OFFSET], 3);
}

/* Read the temperature from the BMI160.
 * Note that the gyro must be operational for the temperature sensor to work
 * correctly. */
s16 bmi160_read_temp(void)
{
  return bmi160_read_reg(BMI160_REG_TEMPERATURE_0) |
         (bmi160_read_reg(BMI160_REG_TEMPERATURE_1) << 8);
}

/** Read status register from BMI160. */
u8 bmi160_read_status(void)
{
  return bmi160_read_reg(BMI160_REG_STATUS);
}

/** Read the error register from the BMI160. */
u8 bmi160_read_error(void)
{
  return bmi160_read_reg(BMI160_REG_ERR_REG);
}

