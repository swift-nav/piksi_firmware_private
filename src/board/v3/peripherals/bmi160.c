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

#define BMI160_STATUS_ACC_RDY_Msk 0x80
#define BMI160_STATUS_GYRO_RDY_Msk 0x40
#define BMI160_STATUS_MAG_RDY_Msk 0x20
#define BMI160_STATUS_I2C_OP_Msk 0x04

#define BMI160_DATA_MAG_OFFSET 0
#define BMI160_DATA_GYRO_OFFSET 8
#define BMI160_DATA_ACC_OFFSET 14
#define BMI160_DATA_TIME_OFFSET 20
#define BMI160_DATA_SIZE 23

//ID of device in BMI160_REG_CHIP_ID
#define BMI160_MFDVID 0b11010001

#define BMM150_I2C_SLV_ADDR 0x13
#define BMM150_REG_MFDVID 0x32

#define BMM150_REG_ID 0x40
#define BMM150_REG_MODE1 0x4B
#define BMM150_REG_MODE2 0x4C
#define BMM150_REG_DATA 0x42

static void bmi160_open_spi(void)
{
  spiAcquireBus(&IMU_SPI);
  spiStart(&IMU_SPI, &spi_config);
  spiSelect(&IMU_SPI);
  /* TODO: seems to be needed for writes */
  chThdSleepMilliseconds(1);
}

static void bmi160_close_spi(void)
{
  spiUnselect(&IMU_SPI);
  spiReleaseBus(&IMU_SPI);
}

static void bmi160_write_reg(u8 reg, u8 data)
{
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  bmi160_close_spi();
}

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

static void bmi160_wait_cmd_complete(void)
{
  //TODO: the check for the command to complete doesnt seem to work,
  //adding forced sleep to avoid dropping commands. Is there a better way?
  chThdSleepMilliseconds(10);
  while(bmi160_read_reg(BMI160_REG_CMD) != 0);
}

static void bmm150_wait_cmd_complete(void)
{
  while(bmi160_read_reg(BMI160_REG_STATUS) & BMI160_STATUS_I2C_OP_Msk);
}

static bool bmi160_check_id(void)
{
  return bmi160_read_reg(BMI160_REG_CHIP_ID) == BMI160_MFDVID;
}

static bool bmm150_check_id(void)
{
  //put mag into setup mode
  bmi160_write_reg(BMI160_REG_MAG_IF+1, 0x80);

  //Read chip ID
  bmi160_write_reg(BMI160_REG_MAG_IF+2, BMM150_REG_ID);
  bmm150_wait_cmd_complete();
  u8 id = bmi160_read_reg(BMI160_REG_DATA + BMI160_DATA_MAG_OFFSET);

  //put mag out of setup mode into data mode
  bmi160_write_reg(BMI160_REG_MAG_IF+2, BMM150_REG_DATA);
  bmm150_wait_cmd_complete();
  bmi160_write_reg(BMI160_REG_MAG_IF+1, 3);

  return id == BMM150_REG_MFDVID;
}

void bmi160_init(void)
{
  //TODO: Put this somewhere better?
  #if defined(BOARD_PIKSIV3_EVT2)
  palSetLine(LINE_IMU_EN);
  chThdSleepMilliseconds(1);
  #endif

  //pulse SS to trigger BMI160 to use SPI
  bmi160_open_spi();
  bmi160_close_spi();
  bmi160_open_spi();
  bmi160_close_spi();

  /* Perform a soft reset of the BMI160 */
  bmi160_write_reg(BMI160_REG_CMD, 0xB6);
  bmi160_wait_cmd_complete();

  if (!bmi160_check_id()) {
    log_error("BIST imu: IMU ID didn't match expected value");
    return;
  }

  //set I2C slv addr of mag
  bmi160_write_reg(BMI160_REG_MAG_IF, BMM150_I2C_SLV_ADDR<<1);
  //enable mag IF interface (for BMM150)
  bmi160_write_reg(BMI160_REG_IF_CONF, 0b00100000);

  if (!bmm150_check_id()) {
    log_error("BIST imu: mag ID didn't match expected value");
    //return;
  }

  //lower mag read rate to 25/2 Hz
  //this just sets polling rate, the actual updated rate is configured on the BMM150
  bmi160_write_reg(BMI160_REG_MAG_CONF,0b00000101);

  /*
  //put sensors in normal mode
  //0b (PMU) 0001 (accel) 00 (normal) 01
  bmi160_write_reg(BMI160_REG_CMD, 0b00010001);
  bmi160_wait_cmd_complete();
  //0b (PMU) 0001 (gyro) 01 (normal) 01
  bmi160_write_reg(BMI160_REG_CMD, 0b00010101);
  bmi160_wait_cmd_complete();
  //0b (PMU) 0001 (mag) 10 (normal) 01
  bmi160_write_reg(BMI160_REG_CMD, 0b00011001);
  bmi160_wait_cmd_complete();
  */

  //put into mag into setup mode
  bmi160_write_reg(BMI160_REG_MAG_IF + 1, 0x80);

  //put mag into normal mode
  bmi160_write_reg(BMI160_REG_MAG_IF + 4, 0x01);
  bmi160_write_reg(BMI160_REG_MAG_IF + 3, BMM150_REG_MODE1);
  bmm150_wait_cmd_complete();
  bmi160_write_reg(BMI160_REG_MAG_IF + 4, 0x00);
  bmi160_write_reg(BMI160_REG_MAG_IF + 3, BMM150_REG_MODE2);
  bmm150_wait_cmd_complete();

  //put mag out of setup mode into data mode
  bmi160_write_reg(BMI160_REG_MAG_IF + 2, BMM150_REG_DATA);
  bmm150_wait_cmd_complete();
  bmi160_write_reg(BMI160_REG_MAG_IF + 1, 3);

  bmi160_write_reg(BMI160_REG_INT_OUT_CTRL, 0b00001011);
  bmi160_write_reg(BMI160_REG_INT_MAP_1, 0b10000000);
  bmi160_write_reg(BMI160_REG_INT_EN_1, 0b00010000);

}

void bmi160_set_imu_rate(imu_rate_t rate)
{
  u8 rate_val = BMI160_IMU_RATE_25HZ + (0xF & (u8)rate);
  bmi160_write_reg(BMI160_REG_ACC_CONF, 0b00100000 | rate_val);
  bmi160_write_reg(BMI160_REG_GYR_CONF, 0b00100000 | rate_val);
}

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

void bmi160_set_acc_range(bmi160_acc_range_t range)
{
  bmi160_write_reg(BMI160_REG_ACC_RANGE, 0xF & (u8)range);
}

void bmi160_set_gyr_range(bmi160_gyr_range_t range)
{
  bmi160_write_reg(BMI160_REG_GYR_RANGE, 0x7 & (u8)range);
}

void bmi160_new_data_available(bool* new_acc, bool* new_gyro, bool* new_mag)
{
  u8 status = bmi160_read_reg(BMI160_REG_STATUS);
  *new_acc = status & BMI160_STATUS_ACC_RDY_Msk;
  *new_gyro = status & BMI160_STATUS_GYRO_RDY_Msk;
  *new_mag = status & BMI160_STATUS_MAG_RDY_Msk;
}

void bmi160_get_data(s16 acc[static 3], s16 gyro[static 3], s16 mag[static 3], u32* sensor_time)
{
  //first byte is for register address
  u8 buf[BMI160_DATA_SIZE + 1];
  buf[0] = BMI160_REG_DATA | SPI_READ_MASK;
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(buf), buf, buf);
  bmi160_close_spi();

  memcpy(mag, &buf[1 + BMI160_DATA_MAG_OFFSET], 2 * 3);
  memcpy(gyro, &buf[1 + BMI160_DATA_GYRO_OFFSET], 2 * 3);
  memcpy(acc, &buf[1 + BMI160_DATA_ACC_OFFSET], 2 * 3);
  *sensor_time = 0;
  memcpy(sensor_time, &buf[1 + BMI160_DATA_TIME_OFFSET], 3);
}


u8 bmi160_read_status(void)
{
  return bmi160_read_reg(BMI160_REG_STATUS);
}

u8 bmi160_read_error(void)
{
  return bmi160_read_reg(BMI160_REG_ERR_REG);
}

