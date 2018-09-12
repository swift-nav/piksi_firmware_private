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

#include "bmi160.h"
#include <ch.h>
#include <hal.h>
#include <string.h>
#include <swiftnav/logging.h>

static const SPIConfig spi_config = IMU_SPI_CONFIG;

static bmm150_trim_t bmm150_trim_param;

#define SPI_READ_MASK (1 << 7)

/** Open and lock the SPI but that the BMI160 is on. */
static void bmi160_open_spi(void) {
  spiAcquireBus(&IMU_SPI);
  spiStart(&IMU_SPI, &spi_config);
  spiSelect(&IMU_SPI);
  /* TODO: seems to be needed for writes */
  chThdSleepMilliseconds(1);
}

/** Close and release the SPI but that the BMI160 is on. */
static void bmi160_close_spi(void) {
  spiUnselect(&IMU_SPI);
  spiReleaseBus(&IMU_SPI);
}

/** Write a register value to the BMI160. */
static void bmi160_write_reg(u8 reg, u8 data) {
  const u8 send_buf[2] = {reg, data};
  u8 recv_buf[2];
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  bmi160_close_spi();
}

/** Read a register value from the BMI160. */
static u8 bmi160_read_reg(u8 reg) {
  const u8 dummy_data = 0x00;
  const u8 send_buf[2] = {reg | SPI_READ_MASK, dummy_data};
  u8 recv_buf[2];
  bmi160_open_spi();
  spiExchange(&IMU_SPI, sizeof(send_buf), send_buf, recv_buf);
  bmi160_close_spi();
  return recv_buf[1];
}

/** Wait for a command to complete on the BMI160. */
static void bmi160_wait_cmd_complete(void) {
  /* TODO: the check for the command to complete doesnt seem to work,
   * adding forced sleep to avoid dropping commands. Is there a better way? */
  chThdSleepMilliseconds(10);
  while (bmi160_read_reg(BMI160_REG_CMD) != 0)
    ;
}

/** Wait for a I2C magnetometer command to complete on the BMI160. */
static void bmm150_wait_cmd_complete(void) {
  /* TODO: the check for the command to complete doesnt seem to work,
   * adding forced sleep to avoid dropping commands. Is there a better way? */
  chThdSleepMilliseconds(10);
  while (bmi160_read_reg(BMI160_REG_STATUS) & BMI160_STATUS_I2C_OP_Msk)
    ;
}

static void bmi160_set_bmm150_config_mode(bool config) {
  if (config) {
    /* Put into mag into setup mode */
    bmi160_write_reg(BMI160_REG_MAG_IF + 1, 0x80);
  } else {
    /* Put mag out of setup mode into data mode */
    bmi160_write_reg(BMI160_REG_MAG_IF + 2, BMM150_REG_DATA);
    bmm150_wait_cmd_complete();
    bmi160_write_reg(BMI160_REG_MAG_IF + 1, 3);
  }
}

static void bmi160_write_bmm150_reg(u8 reg, u8 data) {
  bmi160_set_bmm150_config_mode(true);
  bmi160_write_reg(BMI160_REG_MAG_IF + 4, data);
  bmi160_write_reg(BMI160_REG_MAG_IF + 3, reg);
  bmm150_wait_cmd_complete();
  bmi160_set_bmm150_config_mode(false);
}

static u8 bmi160_read_bmm150_reg(u8 reg) {
  bmi160_set_bmm150_config_mode(true);
  bmi160_write_reg(BMI160_REG_MAG_IF + 2, reg);
  bmm150_wait_cmd_complete();
  u8 data = bmi160_read_reg(BMI160_REG_DATA + BMI160_DATA_MAG_OFFSET);
  bmi160_set_bmm150_config_mode(false);
  return data;
}

static u16 read_bmm150_trim_u16(u8 bmm150_lsb) {
  u16 ret = bmi160_read_bmm150_reg(bmm150_lsb);
  ret += bmi160_read_bmm150_reg(bmm150_lsb + 1) << 8;
  return ret;
}

/** Read the magnetometer trim settings. These are
 * set in the factory and only need to be read once */
static void read_mag_trim(bmm150_trim_t* trim_param) {
  trim_param->dig_x1 = bmi160_read_bmm150_reg(BMM150_DIG_X1);
  trim_param->dig_y1 = bmi160_read_bmm150_reg(BMM150_DIG_Y1);
  trim_param->dig_x2 = bmi160_read_bmm150_reg(BMM150_DIG_X2);
  trim_param->dig_y2 = bmi160_read_bmm150_reg(BMM150_DIG_Y2);
  trim_param->dig_z1 = read_bmm150_trim_u16(BMM150_DIG_Z1_LSB);
  trim_param->dig_z2 = read_bmm150_trim_u16(BMM150_DIG_Z2_LSB);
  trim_param->dig_z3 = read_bmm150_trim_u16(BMM150_DIG_Z3_LSB);
  trim_param->dig_z4 = read_bmm150_trim_u16(BMM150_DIG_Z4_LSB);
  trim_param->dig_xy1 = bmi160_read_bmm150_reg(BMM150_DIG_XY1);
  trim_param->dig_xy2 = bmi160_read_bmm150_reg(BMM150_DIG_XY2);
  /*xyz1 MSB has has other info in upper bits according to Bosch driver code.*/
  u16 temp_msb = ((u16)(bmi160_read_bmm150_reg(BMM150_DIG_XYZ1_LSB + 1) & 0x7F))
                 << 8;
  trim_param->dig_xyz1 =
      (u16)(temp_msb) | (u16)(bmi160_read_bmm150_reg(BMM150_DIG_XYZ1_LSB));
}

void bmi160_init(void) {
  /* Delay required to prevent IMU initialization conflicting with the
   * front-end configuration, resulting in no signals being acquired or
   * tracked. */
  /* TODO: Investigate why this delay is required. */
  chThdSleepMilliseconds(1);

  if (!bmm150_unit_test()) {
    log_error("MAG: Compensation Unit Test Failed");
  }

  /* Pulse SS to trigger BMI160 to use SPI */
  bmi160_open_spi();
  bmi160_close_spi();
  bmi160_open_spi();
  bmi160_close_spi();

  /* Perform a soft reset of the BMI160 */
  bmi160_write_reg(BMI160_REG_CMD, 0xB6);
  bmi160_wait_cmd_complete();
  /* Check if the ID of the BMI160 is as expected. */
  u8 imu_id = bmi160_read_reg(BMI160_REG_CHIP_ID);
  if (imu_id != BMI160_MFDVID) {
    log_error("IMU: BMI160 ID didn't match expected value (%u)", imu_id);
    return;
  }

  /* set I2C slv addr of mag */
  bmi160_write_reg(BMI160_REG_MAG_IF, BMM150_I2C_SLV_ADDR << 1);
  /* configure secondary interface for mag I2C */
  bmi160_write_reg(BMI160_REG_IF_CONF, 0b00100000);
  /* Need to enable interface to reset */
  bmi160_mag_set_enabled(true);
  /* Perform a soft reset of the BMM150 */
  bmi160_write_bmm150_reg(BMM150_REG_MODE1, 0b10000011);
  /* Set the mag averaging to regular preset (7 samples xy, 15 z) */
  bmi160_write_bmm150_reg(BMM150_REG_NXY, 4);
  bmi160_write_bmm150_reg(BMM150_REG_NZ, 7);
  /* Put the BMM150 in "forced" mode */
  bmi160_write_bmm150_reg(BMM150_REG_MODE2, 0b00000010);
  /* Check if the ID of the BMI160 is as expected. */
  u8 mag_id = bmi160_read_bmm150_reg(BMM150_REG_ID);
  if (mag_id != BMM150_REG_MFDVID) {
    log_error("Mag: BMM150 ID didn't match expected value (%u)", mag_id);
    return;
  }
  read_mag_trim(&bmm150_trim_param);
  bmi160_mag_set_enabled(false);

  /* Configure IMU_INT1, interrupt on data ready */
  bmi160_write_reg(BMI160_REG_INT_OUT_CTRL, 0b00001011);
  bmi160_write_reg(BMI160_REG_INT_MAP_1, 0b10000000);
  bmi160_write_reg(BMI160_REG_INT_EN_1, 0b00010000);
}

/** Set the IMU (Accels and Gyros) data rate.
 * Doesn't affect the operation of the magnetometer. */
void bmi160_set_imu_rate(bmi160_rate_t rate) {
  u8 rate_val = (0xF & (u8)rate);
  bmi160_write_reg(BMI160_REG_ACC_CONF, 0b00100000 | rate_val);
  bmi160_write_reg(BMI160_REG_GYR_CONF, 0b00100000 | rate_val);
}

/** Set the magnetometer data rate.
 * Doesn't affect the operation of the IMU. */
void bmi160_set_mag_rate(bmi160_rate_t rate) {
  u8 rate_val = (0xF & (u8)rate);
  bmi160_write_reg(BMI160_REG_MAG_CONF, 0b00100000 | rate_val);
}

/** Enable or disable the IMU (Accels and Gyros) in the BMI160.
 * Doesn't affect the operation of the magnetometer. */
void bmi160_imu_set_enabled(bool enabled) {
  /* Set sensor mode to Normal or Suspended depending on the enabled value. */
  u8 mode = enabled ? 1 : 0;

  /* 0b (PMU) 0001 (accel) 00 (mode) 0? */
  bmi160_write_reg(BMI160_REG_CMD, 0b00010000 | mode);
  bmi160_wait_cmd_complete();

  /* 0b (PMU) 0001 (gyro) 01 (mode) 0? */
  bmi160_write_reg(BMI160_REG_CMD, 0b00010100 | mode);
  bmi160_wait_cmd_complete();
}

/** Enable or disable the magnetometer in the BMI160 and BMM150.
 * Doesn't affect the operation of the IMU. */
void bmi160_mag_set_enabled(bool enabled) {
  /* Set sensor mode to Normal or Suspended depending on the enabled value. */
  u8 mode = enabled ? 1 : 0;
  /* 0b (PMU) 0001 (mag) 10 (mode) 0? */
  bmi160_write_reg(BMI160_REG_CMD, 0b00011000 | mode);
  bmi160_wait_cmd_complete();
}

/** Set the full scale range of the accelerometer. */
void bmi160_set_acc_range(bmi160_acc_range_t range) {
  bmi160_write_reg(BMI160_REG_ACC_RANGE, 0xF & (u8)range);
}

/** Set the full scale range of the gyroscope. */
void bmi160_set_gyr_range(bmi160_gyr_range_t range) {
  bmi160_write_reg(BMI160_REG_GYR_RANGE, 0x7 & (u8)range);
}

/** Check if any new data is available from the sensors. */
void bmi160_new_data_available(bool* new_acc, bool* new_gyro, bool* new_mag) {
  u8 status = bmi160_read_reg(BMI160_REG_STATUS);
  *new_acc = status & BMI160_STATUS_ACC_RDY_Msk;
  *new_gyro = status & BMI160_STATUS_GYRO_RDY_Msk;
  *new_mag = status & BMI160_STATUS_MAG_RDY_Msk;
}

/** The following compensation functions are verbatim from Bosch BMM150 Driver
   available here:
    https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c
   with sha c7774c906ce98c6a5558db3a752d5cdd0abe260f

    The only modifications from BOSCH reference are:
      - function names
      - signatures
      - chibios types
    Note:
      - we used to scale our integers so the lsb was 1/16 of a microtesla,
        but this was removed in v1.6 release to conform to Bosch driver
        and to avoid any chance of overflow, however slight.
    **/

static s16 bmm150_compensate_X(const bmm150_trim_t* trim_param,
                               s16 mag_data_x,
                               u16 data_rhall) {
  s16 retval;
  u16 process_comp_x0 = 0;
  s32 process_comp_x1;
  u16 process_comp_x2;
  s32 process_comp_x3;
  s32 process_comp_x4;
  s32 process_comp_x5;
  s32 process_comp_x6;
  s32 process_comp_x7;
  s32 process_comp_x8;
  s32 process_comp_x9;
  s32 process_comp_x10;

  /* Overflow condition check */
  if (mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
    if (data_rhall != 0) {
      /* Availability of valid data*/
      process_comp_x0 = data_rhall;
    } else if (trim_param->dig_xyz1 != 0) {
      process_comp_x0 = trim_param->dig_xyz1;
    } else {
      process_comp_x0 = 0;
    }
    if (process_comp_x0 != 0) {
      /* Processing compensation equations*/
      process_comp_x1 = ((s32)trim_param->dig_xyz1) * 16384;
      process_comp_x2 =
          ((u16)(process_comp_x1 / process_comp_x0)) - ((u16)0x4000);
      retval = ((s16)process_comp_x2);
      process_comp_x3 = (((s32)retval) * ((s32)retval));
      process_comp_x4 = (((s32)trim_param->dig_xy2) * (process_comp_x3 / 128));
      process_comp_x5 = (s32)(((s16)trim_param->dig_xy1) * 128);
      process_comp_x6 = ((s32)retval) * process_comp_x5;
      process_comp_x7 =
          (((process_comp_x4 + process_comp_x6) / 512) + ((s32)0x100000));
      process_comp_x8 = ((s32)(((s16)trim_param->dig_x2) + ((s16)0xA0)));
      process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
      process_comp_x10 = ((s32)mag_data_x) * process_comp_x9;
      retval = ((s16)(process_comp_x10 / 8192));
      retval = (retval + (((s16)trim_param->dig_x1) * 8)) / 16;
    } else {
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  } else {
    /* Overflow condition */
    retval = BMM150_OVERFLOW_OUTPUT;
  }

  return retval;
}

static s16 bmm150_compensate_Y(const bmm150_trim_t* trim_param,
                               s16 mag_data_y,
                               u16 data_rhall) {
  s16 retval;
  u16 process_comp_y0 = 0;
  s32 process_comp_y1;
  u16 process_comp_y2;
  s32 process_comp_y3;
  s32 process_comp_y4;
  s32 process_comp_y5;
  s32 process_comp_y6;
  s32 process_comp_y7;
  s32 process_comp_y8;
  s32 process_comp_y9;

  /* Overflow condition check */
  if (mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
    if (data_rhall != 0) {
      /* Availability of valid data*/
      process_comp_y0 = data_rhall;
    } else if (trim_param->dig_xyz1 != 0) {
      process_comp_y0 = trim_param->dig_xyz1;
    } else {
      process_comp_y0 = 0;
    }
    if (process_comp_y0 != 0) {
      /*Processing compensation equations*/
      process_comp_y1 = (((s32)trim_param->dig_xyz1) * 16384) / process_comp_y0;
      process_comp_y2 = ((u16)process_comp_y1) - ((u16)0x4000);
      retval = ((s16)process_comp_y2);
      process_comp_y3 = ((s32)retval) * ((s32)retval);
      process_comp_y4 = ((s32)trim_param->dig_xy2) * (process_comp_y3 / 128);
      process_comp_y5 = ((s32)(((s16)trim_param->dig_xy1) * 128));
      process_comp_y6 =
          ((process_comp_y4 + (((s32)retval) * process_comp_y5)) / 512);
      process_comp_y7 = ((s32)(((s16)trim_param->dig_y2) + ((s16)0xA0)));
      process_comp_y8 =
          (((process_comp_y6 + ((s32)0x100000)) * process_comp_y7) / 4096);
      process_comp_y9 = (((s32)mag_data_y) * process_comp_y8);
      retval = (s16)(process_comp_y9 / 8192);
      retval = (retval + (((s16)trim_param->dig_y1) * 8)) / 16;
    } else {
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  } else {
    /* Overflow condition*/
    retval = BMM150_OVERFLOW_OUTPUT;
  }

  return retval;
}

/*!
 * @brief This internal API is used to obtain the compensated
 * magnetometer Z axis data(micro-tesla) in s16.
 */
static s16 bmm150_compensate_Z(const bmm150_trim_t* trim_param,
                               s16 mag_data_z,
                               u16 data_rhall) {
  s32 retval;
  s16 process_comp_z0;
  s32 process_comp_z1;
  s32 process_comp_z2;
  s32 process_comp_z3;
  s16 process_comp_z4;

  if (mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) {
    if ((trim_param->dig_z2 != 0) && (trim_param->dig_z1 != 0) &&
        (data_rhall != 0) && (trim_param->dig_xyz1 != 0)) {
      /*Processing compensation equations*/
      process_comp_z0 = ((s16)data_rhall) - ((s16)trim_param->dig_xyz1);
      process_comp_z1 =
          (((s32)trim_param->dig_z3) * ((s32)(process_comp_z0))) / 4;
      process_comp_z2 = (((s32)(mag_data_z - trim_param->dig_z4)) * 32768);
      process_comp_z3 = ((s32)trim_param->dig_z1) * (((s16)data_rhall) * 2);
      process_comp_z4 = (s16)((process_comp_z3 + (32768)) / 65536);
      retval = ((process_comp_z2 - process_comp_z1) /
                (trim_param->dig_z2 + process_comp_z4));

      /* saturate result to +/- 2 micro-tesla */
      if (retval > BMM150_POSITIVE_SATURATION_Z) {
        retval = BMM150_POSITIVE_SATURATION_Z;
      } else {
        if (retval < BMM150_NEGATIVE_SATURATION_Z)
          retval = BMM150_NEGATIVE_SATURATION_Z;
      }
      /* Original Bosch code has Conversion of LSB to micro-tesla per below*/
      retval = retval / 16;

    } else {
      retval = BMM150_OVERFLOW_OUTPUT;
    }
  } else {
    /* Overflow condition*/
    retval = BMM150_OVERFLOW_OUTPUT;
  }

  return (s16)retval;
}
/** Read the sensor data from the BMI160 and BMM150.
 * If parameter mag==NULL skip the mag data (reading mag when not ready causes
 * errors)
*/
void bmi160_get_data(s16 acc[static 3],
                     s16 gyro[static 3],
                     s16* mag,
                     u32* sensor_time) {
  /* First byte is for register address. All sensors are read together. */
  u8 buf[BMI160_DATA_SIZE + 1];
  u8 start_reg = BMI160_REG_DATA;
  u8 read_size = BMI160_DATA_SIZE;
  u8* buf_ptr = buf;
  /* Skip the mag portion of IMU data if not ready */
  if (mag == NULL) {
    start_reg += BMI160_DATA_GYRO_OFFSET;
    read_size -= BMI160_DATA_GYRO_OFFSET;
    buf_ptr += BMI160_DATA_GYRO_OFFSET;
  }
  buf_ptr[0] = start_reg | SPI_READ_MASK;
  bmi160_open_spi();
  spiExchange(&IMU_SPI, read_size, buf_ptr, buf_ptr);
  bmi160_close_spi();

  /* Extract data from data buffer */
  if (mag != NULL) {
    s16 mag_raw[3];
    u16 rhall = 0;
    s16 msb = 0;
    u8 lsb = 0;
    /* Mag x & y values are encoded in a 13 bit signed int. Lower 5 bits in
     * first reg Upper 8 bits are in next.*/
    lsb = (buf[1 + BMI160_DATA_MAG_OFFSET] & (u8)BMM150_DATA_X_MSK) >>
          BMM150_DATA_X_POS;
    /* Here we shift msb up by 5  (mulitiply by 32) */
    msb = ((s16)((s8)buf[2 + BMI160_DATA_MAG_OFFSET])) * 32;
    mag_raw[0] = (s16)(msb | lsb);
    /* Mag y*/
    lsb = (buf[3 + BMI160_DATA_MAG_OFFSET] & (u8)BMM150_DATA_Y_MSK) >>
          BMM150_DATA_Y_POS;
    msb = ((s16)((s8)buf[4 + BMI160_DATA_MAG_OFFSET])) * 32;
    mag_raw[1] = (s16)(msb | lsb);
    /* Mag z value is encoded in a 15 bit signed int. */
    lsb = (buf[5 + BMI160_DATA_MAG_OFFSET] & (u8)BMM150_DATA_Z_MSK) >>
          BMM150_DATA_Z_POS;
    /*  Here we shift msb up by 7 (mulitiply by 128)*/
    msb = ((s16)((s8)buf[6 + BMI160_DATA_MAG_OFFSET])) * 128;
    mag_raw[2] = (s16)(msb | lsb);
    /* RHALL */
    lsb = (buf[7 + BMI160_DATA_MAG_OFFSET] & (u8)BMM150_DATA_RHALL_MSK) >>
          BMM150_DATA_RHALL_POS;
    rhall = (u16)((((u16)buf[8 + BMI160_DATA_MAG_OFFSET]) * 64) | lsb);

    /* Report temperature/bias compensated measurements
     * 12:4 fixed point
     */
    // memcpy(mag_raw, &buf[1 + BMI160_DATA_MAG_OFFSET], 2 * 3);
    /* Report temperature/bias compensated measurements
     * 12:4 fixed point
     */
    mag[0] = bmm150_compensate_X(&bmm150_trim_param, mag_raw[0], rhall);
    mag[1] = bmm150_compensate_Y(&bmm150_trim_param, mag_raw[1], rhall);
    mag[2] = bmm150_compensate_Z(&bmm150_trim_param, mag_raw[2], rhall);
  }
  memcpy(gyro, &buf[1 + BMI160_DATA_GYRO_OFFSET], 2 * 3);
  memcpy(acc, &buf[1 + BMI160_DATA_ACC_OFFSET], 2 * 3);
  *sensor_time = 0;
  memcpy(sensor_time, &buf[1 + BMI160_DATA_TIME_OFFSET], 3);
}

/* Read the temperature from the BMI160.
 * Note that the gyro must be operational for the temperature sensor to work
 * correctly. */
s16 bmi160_read_temp(void) {
  return bmi160_read_reg(BMI160_REG_TEMPERATURE_0) |
         (bmi160_read_reg(BMI160_REG_TEMPERATURE_1) << 8);
}

/** Read status register from BMI160. */
u8 bmi160_read_status(void) { return bmi160_read_reg(BMI160_REG_STATUS); }

/** Read the error register from the BMI160. */
u8 bmi160_read_error(void) { return bmi160_read_reg(BMI160_REG_ERR_REG); }

bool bmm150_unit_test(void) {
  /* Returns true if magnetometer compensation calculations match expectation
   * Float: x: -21.812500 y: 19.937500 z: 75.687500
   * Fixed: x: -349 y: 319 z: 1211
   * Based on:
   * https://drive.google.com/open?id=0B8rYj-Dy3tcDM3I1bmhWS05wblhpcHZPUkwtRkhvWmg2Wjdn
   */
  const s16 raw_data_x = -60;
  const s16 raw_data_y = 55;
  const s16 raw_data_z = 202;
  const s16 raw_data_r = 6828;
  const bmm150_trim_t dummy_bmm050_trim = {
      0, 0, 26, 26, 22752, 737, -1035, 0, 29, -3, 6753};
  bool passed = true;
  s16 comp_x = bmm150_compensate_X(&dummy_bmm050_trim, raw_data_x, raw_data_r);
  s16 comp_y = bmm150_compensate_Y(&dummy_bmm050_trim, raw_data_y, raw_data_r);
  s16 comp_z = bmm150_compensate_Z(&dummy_bmm050_trim, raw_data_z, raw_data_r);
  passed &= (comp_x == (s16)(-21.75)); /* microtesla */
  passed &= (comp_y == (s16)(19.96));  /* microtesla */
  passed &= (comp_z == (s16)(75.74));  /* microtesla */
  return passed;
}
