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

#ifndef _SWIFTNAV_BMI160_H_
#define _SWIFTNAV_BMI160_H_

#include <libswiftnav/common.h>

/* BMI160 Register Addresses */

#define BMI160_REG_CHIP_ID 0x00
#define BMI160_REG_ERR_REG 0x02
#define BMI160_REG_PMU_STATUS 0x03
#define BMI160_REG_DATA 0x04
#define BMI160_REG_SENSORTIME 0x18
#define BMI160_REG_STATUS 0x1B
#define BMI160_REG_INT_STATUS_0 0x1C
#define BMI160_REG_INT_STATUS_1 0x1D
#define BMI160_REG_INT_STATUS_2 0x1E
#define BMI160_REG_INT_STATUS_3 0x1F
#define BMI160_REG_TEMPERATURE_0 0x20
#define BMI160_REG_TEMPERATURE_1 0x21
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_MAG_CONF 0x44
#define BMI160_REG_MAG_IF 0x4B
#define BMI160_REG_INT_EN_0 0x50
#define BMI160_REG_INT_EN_1 0x51
#define BMI160_REG_INT_EN_2 0x52
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_MAP_0 0x55
#define BMI160_REG_INT_MAP_1 0x56
#define BMI160_REG_INT_MAP_2 0x57
#define BMI160_REG_INT_DATA_0 0x58
#define BMI160_REG_INT_DATA_1 0x59
#define BMI160_REG_IF_CONF 0x6B
#define BMI160_REG_CMD 0x7E

typedef enum {
  BMI160_ACC_2G = 0b0011,
  BMI160_ACC_4G = 0b0101,
  BMI160_ACC_8G = 0b1000,
  BMI160_ACC_16G = 0b1100
} bmi160_acc_range_t;

typedef enum {
  BMI160_GYR_2000DGS = 0b000,
  BMI160_GYR_1000DGS = 0b001,
  BMI160_GYR_500DGS = 0b010,
  BMI160_GYR_250DGS = 0b011,
  BMI160_GYR_125DGS = 0b100
} bmi160_gyr_range_t;

/* rate = 25 * 2 ^ (imu_rate - 6) valid for 12 > imu_rate > 0 */
typedef enum {
  BMI160_RATE_6_25HZ = 0b0100,
  BMI160_RATE_12_5HZ = 0b0101,
  BMI160_RATE_25HZ = 0b0110,
  BMI160_RATE_50HZ = 0b0111,
  BMI160_RATE_100HZ = 0b1000,
  BMI160_RATE_200HZ = 0b1001,
  BMI160_RATE_400HZ = 0b1010
} bmi160_rate_t;

#define BMI160_STATUS_ACC_RDY_Msk 0x80
#define BMI160_STATUS_GYRO_RDY_Msk 0x40
#define BMI160_STATUS_MAG_RDY_Msk 0x20
#define BMI160_STATUS_I2C_OP_Msk 0x04

#define BMI160_DATA_MAG_OFFSET 0
#define BMI160_DATA_GYRO_OFFSET 8
#define BMI160_DATA_ACC_OFFSET 14
#define BMI160_DATA_TIME_OFFSET 20
#define BMI160_DATA_SIZE 23

/* ID of device in BMI160_REG_CHIP_ID */
#define BMI160_MFDVID 0b11010001

/* BMI150 Register Addresses */
#define BMM150_REG_ID 0x40
#define BMM150_REG_MODE1 0x4B
#define BMM150_REG_MODE2 0x4C
#define BMM150_REG_DATA 0x42
#define BMM150_REG_NXY 0x51
#define BMM150_REG_NZ 0x52

/* BMI150 Trim Extended Registers */
#define BMM150_DIG_X1 (0x5D)
#define BMM150_DIG_Y1 (0x5E)
#define BMM150_DIG_Z4_LSB (0x62)
#define BMM150_DIG_Z4_MSB (0x63)
#define BMM150_DIG_X2 (0x64)
#define BMM150_DIG_Y2 (0x65)
#define BMM150_DIG_Z2_LSB (0x68)
#define BMM150_DIG_Z2_MSB (0x69)
#define BMM150_DIG_Z1_LSB (0x6A)
#define BMM150_DIG_Z1_MSB (0x6B)
#define BMM150_DIG_XYZ1_LSB (0x6C)
#define BMM150_DIG_XYZ1_MSB (0x6D)
#define BMM150_DIG_Z3_LSB (0x6E)
#define BMM150_DIG_Z3_MSB (0x6F)
#define BMM150_DIG_XY2 (0x70)
#define BMM150_DIG_XY1 (0x71)

/* BMI150 bit arthimetic macros */
#define BMM150_DATA_X_MSK		(0xF8)
#define BMM150_DATA_X_POS		3

#define BMM150_DATA_Y_MSK		(0xF8)
#define BMM150_DATA_Y_POS		3

#define BMM150_DATA_Z_MSK		(0xFE)
#define BMM150_DATA_Z_POS		1

#define BMM150_DATA_RHALL_MSK		(0xFC)
#define BMM150_DATA_RHALL_POS		2

/**\name OVERFLOW DEFINITIONS  */
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL (-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL (-16384)
#define BMM150_INIT_VALUE (0)
#define BMM150_OVERFLOW_OUTPUT (-32768)
#define BMM150_OVERFLOW_OUTPUT_S32 ((s32)(-2147483647 - 1))
#define BMM150_OVERFLOW_OUTPUT_FLOAT (0.0f)
#define BMM150_NEGATIVE_SATURATION_Z (-32767)
#define BMM150_POSITIVE_SATURATION_Z (32767)

/* Structure containing mag trim parameters */
typedef struct bmm050_t {
  s8 dig_x1; /**< trim x1 data */
  s8 dig_y1; /**< trim y1 data */

  s8 dig_x2; /**< trim x2 data */
  s8 dig_y2; /**< trim y2 data */

  u16 dig_z1; /**< trim z1 data */
  s16 dig_z2; /**< trim z2 data */
  s16 dig_z3; /**< trim z3 data */
  s16 dig_z4; /**< trim z4 data */

  u8 dig_xy1; /**< trim xy1 data */
  s8 dig_xy2; /**< trim xy2 data */

  u16 dig_xyz1; /**< trim xyz1 data */
} bmm150_trim_t;

/* I2C Slave Address of BMM150 */
#define BMM150_I2C_SLV_ADDR 0x13

/* ID of device in BMM150_REG_ID */
#define BMM150_REG_MFDVID 0x32

void bmi160_init(void);
void bmi160_new_data_available(bool* new_acc, bool* new_gyro, bool* new_mag);
void bmi160_get_data(s16 acc[static 3],
                     s16 gyro[static 3],
                     s16 mag[static 3],
                     u32* sensor_time);
void bmi160_set_imu_rate(bmi160_rate_t rate);
void bmi160_imu_set_enabled(bool enabled);
void bmi160_set_acc_range(bmi160_acc_range_t range);
void bmi160_set_gyr_range(bmi160_gyr_range_t range);
void bmi160_set_mag_rate(bmi160_rate_t rate);
void bmi160_mag_set_enabled(bool enabled);
s16 bmi160_read_temp(void);
u8 bmi160_read_status(void);
u8 bmi160_read_error(void);
bool bmm150_unit_test(void);

#endif
