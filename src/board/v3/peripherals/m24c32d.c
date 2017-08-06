/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "m24c32d.h"

#include <ch.h>
#include <hal.h>
#include <libswiftnav/logging.h>

#include <string.h>

#define EEPROM_I2C_ADDR 0x50
#define EEPROM_ID_I2C_ADDR 0x58
#define EEPROM_I2C_TIMEOUT MS2ST(100)

static const I2CConfig eeprom_i2c_config = EEPROM_I2C_CONFIG;

/** Perform an I2C read operation.
 *
 * \param dev           Device address.
 * \param addr          Register address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
static msg_t i2c_read(u8 dev, u16 addr, u8 *data, size_t length) {
  msg_t ret;
  i2cAcquireBus(&EEPROM_I2C);
  i2cStart(&EEPROM_I2C, &eeprom_i2c_config);
  u8 buf[2] = {addr >> 8, addr & 0xFF};
  ret = i2cMasterTransmitTimeout(
      &EEPROM_I2C, dev, buf, sizeof(buf), data, length, EEPROM_I2C_TIMEOUT);

  i2cStop(&EEPROM_I2C);
  i2cReleaseBus(&EEPROM_I2C);
  return ret;
}

/** Perform an I2C write operation.
 *
 * \param addr          Register address.
 * \param data          Input data buffer.
 * \param length        Number of bytes to write.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
static msg_t i2c_write(u8 dev, u16 addr, const u8 *data, size_t length) {
  msg_t ret;
  i2cAcquireBus(&EEPROM_I2C);
  i2cStart(&EEPROM_I2C, &eeprom_i2c_config);
  u8 buf[2 + length];
  buf[0] = addr >> 8;
  buf[1] = addr & 0xFF;
  memcpy(&buf[2], data, length);
  ret = i2cMasterTransmitTimeout(
      &EEPROM_I2C, dev, buf, sizeof(buf), NULL, 0, EEPROM_I2C_TIMEOUT);
  i2cStop(&EEPROM_I2C);
  i2cReleaseBus(&EEPROM_I2C);
  return ret;
}

/** Perform an random-access read of the memory array.
 *
 * \param addr          Start address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
bool m24c32d_read(u16 addr, u8 *data, size_t length) {
  return i2c_read(EEPROM_I2C_ADDR, addr, data, length);
}

/** Perform an page write of the memory array.
 *
 * \param addr          Start address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
bool m24c32d_write(u16 addr, const u8 *data, size_t length) {
  return i2c_write(EEPROM_I2C_ADDR, addr, data, length);
}

/** Perform a read of the ID page.
 *
 * \param addr          Start address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
bool m24c32d_id_read(u8 addr, u8 *data, size_t length) {
  return i2c_read(EEPROM_ID_I2C_ADDR, addr, data, length);
}

/** Perform a write of the ID page.
 *
 * \param addr          Start address.
 * \param data          Output data buffer.
 * \param length        Number of bytes to read.
 *
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
bool m24c32d_id_write(u8 addr, const u8 *data, size_t length) {
  return i2c_write(EEPROM_ID_I2C_ADDR, addr, data, length);
}

/** Lock the ID page.
 * \return MSG_OK if the operation succeeded, error message otherwise.
 */
bool m24c32d_id_lock(void) {
  u8 data = 2;
  return i2c_write(EEPROM_ID_I2C_ADDR, (1 << 10), &data, sizeof(data));
}
