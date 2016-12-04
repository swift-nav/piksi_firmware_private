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

#include "imu.h"
#include <math.h>
#include <hal.h>
#include <string.h>
#include <ch.h>
#include <sbp.h>
#include <timing.h>
#include <libswiftnav/time.h>
#include <libswiftnav/logging.h>
#include <libsbp/imu.h>
#include <board/nap/nap_common.h>

static const SPIConfig spi_config = IMU_SPI_CONFIG;

#define SPI_READ_MASK (1 << 7)

#define BMI160_REG_ID 0x00
#define BMI160_REG_ERR_STATUS 0x02
#define BMI160_REG_PMU_STATUS 0x03
#define BMI160_REG_DATA 0x04
#define BMI160_REG_CMD 0x7E
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_STATUS 0x1B
#define BMI160_REG_MAG_CONF 0x44
#define BMI160_REG_MAG_IF 0x4B
#define BMI160_REG_CONF_IF 0x6B
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_MAP0 0x55
#define BMI160_REG_INT_MAP1 0x56
#define BMI160_REG_INT_MAP2 0x57
#define BMI160_REG_INT_EN0 0x50
#define BMI160_REG_INT_EN1 0x51
#define BMI160_REG_INT_EN2 0x52
#define BMI160_REG_INT_STATUS0 0x1C
#define BMI160_REG_INT_STATUS1 0x1D
#define BMI160_REG_INT_STATUS2 0x1E
#define BMI160_REG_INT_STATUS3 0x1F

#define BMI160_STATUS_ACC_RDY_Msk 0x80
#define BMI160_STATUS_GYRO_RDY_Msk 0x40
#define BMI160_STATUS_MAG_RDY_Msk 0x20
#define BMI160_STATUS_I2C_OP_Msk 0x04

#define BMI160_DATA_MAG_OFFSET 0
#define BMI160_DATA_GYRO_OFFSET 8
#define BMI160_DATA_ACC_OFFSET 14
#define BMI160_DATA_TIME_OFFSET 20
#define BMI160_DATA_SIZE 23

//ID of device in BMI160_REG_ID
#define BMI160_MFDVID 0b11010001

#define BMM150_I2C_SLV_ADDR 0x13
#define BMM150_REG_MFDVID 0x32

#define BMM150_REG_ID 0x40
#define BMM150_REG_MODE1 0x4B
#define BMM150_REG_MODE2 0x4C
#define BMM150_REG_DATA 0x42

static void imu_open_spi(void)
{
  spiAcquireBus(&IMU_SPI);
  spiStart(&IMU_SPI, &spi_config);
  spiSelect(&IMU_SPI);
  //TODO: seems to be needed for writes
  chThdSleepMilliseconds(1);
}

static void imu_close_spi(void)
{
  spiUnselect(&IMU_SPI);
  spiReleaseBus(&IMU_SPI);
}

void imu_write_reg(u8 reg, u8 data)
{
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];
  imu_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  imu_close_spi();
}

u8 imu_read_reg(u8 reg)
{
  const u8 dummy_data = 0x00;
  const u8 send_buf[2] = {reg | SPI_READ_MASK, dummy_data};
  u8 recv_buf[2];
  imu_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  imu_close_spi();
  return recv_buf[1];
}

static void imu_wait_cmd_complete()
{
  //TODO: the check for the command to complete doesnt seem to work,
  //adding forced sleep to avoid dropping commands. Is there a better way?
  chThdSleepMilliseconds(100);
  while(imu_read_reg(BMI160_REG_CMD) != 0);
}

static void imu_wait_mag_cmd_complete()
{
  while(imu_read_reg(BMI160_REG_STATUS) & BMI160_STATUS_I2C_OP_Msk);
}

#define IMU_THREAD_PRIO    (HIGHPRIO - 1)
#define IMU_THREAD_STACK   2000
static THD_WORKING_AREA(wa_imu_thread, IMU_THREAD_STACK);
static BSEMAPHORE_DECL(imu_irq_sem, TRUE);

u32 nap_tc;

static void imu_thread(void *arg)
{
  (void)arg;
  chRegSetThreadName("IMU");

  if (!imu_check_id()) {
    log_error("BIST imu: IMU ID didn't match expected value");
    return;
  }
  if (!mag_check_id()) {
    log_error("BIST imu: mag ID didn't match expected value");
    return;
  }

  bool new_acc, new_gyro, new_mag;
	s16 acc[3];
	s16 gyro[3];
  s16 mag[3];
	u32 sensor_time;
  msg_imu_raw_t imu_raw;
  u64 tc;

  while (TRUE) {

    chBSemWait(&imu_irq_sem);
    imu_new_data_available(&new_acc, &new_gyro, &new_mag);
    imu_get_data(acc, gyro, mag, &sensor_time);
    if (time_quality >= TIME_FINE) {
      tc = nap_sample_time_to_count(nap_tc);
      gps_time_t t = rx2gpstime(tc);
      double tow_ms = t.tow * 1000;
      imu_raw.tow = (u32)tow_ms;
      imu_raw.tow_f = (u8)round((tow_ms - imu_raw.tow)*256);
    } else {
      imu_raw.tow = (1 << 31);
      imu_raw.tow_f = 0;
    }
    imu_raw.acc_x = acc[0];
    imu_raw.acc_y = acc[1];
    imu_raw.acc_z = acc[2];
    imu_raw.gyr_x = gyro[0];
    imu_raw.gyr_y = gyro[1];
    imu_raw.gyr_z = gyro[2];
    sbp_send_msg(SBP_MSG_IMU_RAW, sizeof(imu_raw), (u8*)&imu_raw);
  }
}


static void imu_isr(void *context)
{
  (void)context;
  chSysLockFromISR();

  nap_tc = NAP->TIMING_COUNT;

  /* Wake up processing thread */
  chBSemSignalI(&imu_irq_sem);

  chSysUnlockFromISR();
}

void imu_init(void)
{
  //TODO: Put this somewhere better?
  #if defined(BOARD_PIKSIV3_EVT2)
  palSetLine(LINE_IMU_EN);
  chThdSleepMilliseconds(1);
  #endif

  //pulse SS to trigger BMI160 to use SPI
  imu_open_spi();
  imu_close_spi();
  imu_open_spi();
  imu_close_spi();

  //set I2C slv addr of mag
  imu_write_reg(BMI160_REG_MAG_IF, BMM150_I2C_SLV_ADDR<<1);
  //enable mag IF interface (for BMM150)
  imu_write_reg(BMI160_REG_CONF_IF, 0b00100000);

  //lower mag read rate to 25/2 Hz
  //this just sets polling rate, the actual updated rate is configured on the BMM150
  imu_write_reg(BMI160_REG_MAG_CONF,0b00000101);

  //put sensors in normal mode
  //0b (PMU) 0001 (accel) 00 (normal) 01
  imu_write_reg(BMI160_REG_CMD, 0b00010001);
  imu_wait_cmd_complete();
  //0b (PMU) 0001 (gyro) 01 (normal) 01
  imu_write_reg(BMI160_REG_CMD, 0b00010101);
  imu_wait_cmd_complete();
  //0b (PMU) 0001 (mag) 10 (normal) 01
  imu_write_reg(BMI160_REG_CMD, 0b00011001);
  imu_wait_cmd_complete();

  //put into mag into setup mode
  imu_write_reg(BMI160_REG_MAG_IF + 1, 0x80);

  //put mag into normal mode
  imu_write_reg(BMI160_REG_MAG_IF + 4, 0x01);
  imu_write_reg(BMI160_REG_MAG_IF + 3, BMM150_REG_MODE1);
  imu_wait_mag_cmd_complete();
  imu_write_reg(BMI160_REG_MAG_IF + 4, 0x00);
  imu_write_reg(BMI160_REG_MAG_IF + 3, BMM150_REG_MODE2);
  imu_wait_mag_cmd_complete();

  //put mag out of setup mode into data mode
  imu_write_reg(BMI160_REG_MAG_IF + 2, BMM150_REG_DATA);
  imu_wait_mag_cmd_complete();
  imu_write_reg(BMI160_REG_MAG_IF + 1, 3);

  imu_write_reg(BMI160_REG_INT_OUT_CTRL, 0b00001011);
  imu_write_reg(BMI160_REG_INT_MAP1, 0b10000000);
  imu_write_reg(BMI160_REG_INT_EN1, 0b00010000);

  chThdCreateStatic(wa_imu_thread, sizeof(wa_imu_thread),
                    IMU_THREAD_PRIO, imu_thread, NULL);

  /* Enable AOK interrupt */
  gic_handler_register(IRQ_ID_IMU_INT1, imu_isr, NULL);
  gic_irq_sensitivity_set(IRQ_ID_IMU_INT1, IRQ_SENSITIVITY_EDGE);
  gic_irq_priority_set(IRQ_ID_IMU_INT1, IMU_INT1_IRQ_PRIORITY);
  gic_irq_enable(IRQ_ID_IMU_INT1);
}

void imu_new_data_available(bool* new_acc, bool* new_gyro, bool* new_mag)
{
  u8 status = imu_read_reg(BMI160_REG_STATUS);
  *new_acc = status & BMI160_STATUS_ACC_RDY_Msk;
  *new_gyro = status & BMI160_STATUS_GYRO_RDY_Msk;
  *new_mag = status & BMI160_STATUS_MAG_RDY_Msk;
}

void imu_get_data(s16 acc[static 3], s16 gyro[static 3], s16 mag[static 3], u32* sensor_time)
{
  //first byte is for register address
  u8 buf[BMI160_DATA_SIZE + 1];
  buf[0] = BMI160_REG_DATA | SPI_READ_MASK;
  imu_open_spi();
  spiExchange(&IMU_SPI, sizeof(buf), buf, buf);
  imu_close_spi();

  memcpy(mag, &buf[1 + BMI160_DATA_MAG_OFFSET], 2 * 3);
  memcpy(gyro, &buf[1 + BMI160_DATA_GYRO_OFFSET], 2 * 3);
  memcpy(acc, &buf[1 + BMI160_DATA_ACC_OFFSET], 2 * 3);
  *sensor_time = 0;
  memcpy(sensor_time, &buf[1 + BMI160_DATA_TIME_OFFSET], 3);
}

bool imu_check_id(void)
{
  return imu_read_reg(BMI160_REG_ID) == BMI160_MFDVID;
}

bool mag_check_id(void)
{
  //put mag into setup mode
  imu_write_reg(BMI160_REG_MAG_IF+1, 0x80);

  //Read chip ID
  imu_write_reg(BMI160_REG_MAG_IF+2, BMM150_REG_ID);
  imu_wait_mag_cmd_complete();
  u8 id = imu_read_reg(BMI160_REG_DATA + BMI160_DATA_MAG_OFFSET);

  //put mag out of setup mode into data mode
  imu_write_reg(BMI160_REG_MAG_IF+2, BMM150_REG_DATA);
  imu_wait_mag_cmd_complete();
  imu_write_reg(BMI160_REG_MAG_IF+1, 3);

  return id == BMM150_REG_MFDVID;
}

u8 imu_read_status(void)
{
  return imu_read_reg(BMI160_REG_STATUS);
}

u8 imu_read_error(void)
{
  return imu_read_reg(BMI160_REG_ERR_STATUS);
}

u8 imu_read_pmu_status(void)
{
  return imu_read_reg(BMI160_REG_PMU_STATUS);
}
