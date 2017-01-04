/*
 * Copyright (C) 2012-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libswiftnav/logging.h>

#include <hal.h>
#include <ch.h>

#include "../settings.h"
#include "usart.h"
#include "io_support.h"
#include "version.h"

/** \addtogroup io
 * \{ */

static const char const * portmode_enum[] = {"SBP", "NMEA", NULL};
static struct setting_type portmode;

usart_settings_t ftdi_usart = {
  .mode               = SBP,
  .baud_rate          = USART_DEFAULT_BAUD_FTDI,
  .sbp_message_mask   = 0xFFFF,
  .sbp_fwd = 1,
};

usart_settings_t uarta_usart = {
  .mode               = SBP,
  .baud_rate          = USART_DEFAULT_BAUD_TTL,
  .sbp_message_mask   = 0xC0,
  .sbp_fwd = 0,
};

usart_settings_t uartb_usart = {
  .mode             = SBP,
  .baud_rate        = USART_DEFAULT_BAUD_TTL,
  .sbp_message_mask = 0xFF00,
  .sbp_fwd = 0,
};

/** \} */

/** \addtogroup peripherals
 * \{ */

/** \defgroup usart USART
 * Functions to setup and use STM32F4 USART peripherals with DMA.
 * \{ */

usart_state ftdi_state = {.sd = SD_SBP,
                          .claimed = _BSEMAPHORE_DATA(ftdi_state.claimed, FALSE)};
usart_state uarta_state = {.sd = SD_UARTA,
                          .claimed = _BSEMAPHORE_DATA(uarta_state.claimed, FALSE)};
usart_state uartb_state = {.sd = SD_UARTB,
                          .claimed = _BSEMAPHORE_DATA(uartb_state.claimed, FALSE)};

static bool baudrate_change_notify(struct setting *s, const char *val);

/** Set up the USART peripherals, hook them into the settings subsystem
*
*/
void usarts_setup(void)
{
  int TYPE_PORTMODE = settings_type_register_enum(portmode_enum, &portmode);

  SETTING("uart_ftdi", "mode", ftdi_usart.mode, TYPE_PORTMODE);
  SETTING("uart_ftdi", "sbp_message_mask", ftdi_usart.sbp_message_mask, TYPE_INT);
  SETTING("uart_ftdi", "fwd_msg", ftdi_usart.sbp_fwd, TYPE_INT);
  SETTING_NOTIFY("uart_ftdi", "baudrate", ftdi_usart.baud_rate, TYPE_INT,
                 baudrate_change_notify);

  SETTING("uart_uarta", "mode", uarta_usart.mode, TYPE_PORTMODE);
  SETTING("uart_uarta", "sbp_message_mask", uarta_usart.sbp_message_mask, TYPE_INT);
  SETTING("uart_uarta", "fwd_msg", uarta_usart.sbp_fwd, TYPE_INT);
  SETTING_NOTIFY("uart_uarta", "baudrate", uarta_usart.baud_rate, TYPE_INT,
          baudrate_change_notify);

  SETTING("uart_uartb", "mode", uartb_usart.mode, TYPE_PORTMODE);
  SETTING("uart_uartb", "sbp_message_mask", uartb_usart.sbp_message_mask, TYPE_INT);
  SETTING("uart_uartb", "fwd_msg", uartb_usart.sbp_fwd, TYPE_INT);
  SETTING_NOTIFY("uart_uartb", "baudrate", uartb_usart.baud_rate, TYPE_INT,
          baudrate_change_notify);

  usarts_enable(ftdi_usart.baud_rate, uarta_usart.baud_rate, uartb_usart.baud_rate);

}

/** Callback for settings subsystem changing the baudrate of a UART.
*
*/
static bool baudrate_change_notify(struct setting *s, const char *val)
{
  if (s->type->from_string(s->type->priv, s->addr, s->len, val)) {
    usarts_disable();
    usarts_enable(ftdi_usart.baud_rate, uarta_usart.baud_rate, uartb_usart.baud_rate);
    return true;
  }
  return false;
}


/** Enable the USART peripherals.
 * USART 6, 1 and 3 peripherals are configured
 * (connected to the FTDI, UARTA and UARTB ports on the Piksi respectively).
 */
void usarts_enable(u32 ftdi_baud, u32 uarta_baud, u32 uartb_baud)
{
  io_support_init();

  io_support_set_parameters(SD_SBP, ftdi_baud);
  io_support_set_parameters(SD_UARTA, uarta_baud);
  io_support_set_parameters(SD_UARTB, uartb_baud);
}

/** Disable all USARTs. */
void usarts_disable()
{
  /* Disable DMA channels. */
  /* Disable all USARTs. */
  io_support_disable(SD_SBP);
  io_support_disable(SD_UARTA);
  io_support_disable(SD_UARTB);
}

/** Claim this USART for exclusive use by the calling module.
 * This prevents the USART from being used by other modules, and inhibits
 * the standard protocols.  This allows modem (or other) drivers to claim
 * the USART and prevent SBP or other protocol driver from interfering with
 * communications.
 *
 * The same module may nest claims to the port.  The port must be released
 * as many times as it was claimed before it will be available for
 * for another module.
 *
 * \see ::usart_release
 * \param s The USART DMA state structure.
 * \param module A pointer to identify the calling module.  This is compared
 *               by value of the pointer.  The pointer target is unused.
 */
bool usart_claim(usart_state* s, const void *module)
{
  chSysLock();
  if (chBSemWaitTimeoutS(&s->claimed, 0) == MSG_OK) {
    s->claimed_by = module;
    s->claim_nest = 0;
    chSysUnlock();
    return true;
  } else if (s->claimed_by == module) {
    s->claim_nest++;
    chSysUnlock();
    return true;
  }
  chSysUnlock();
  return false;
}

/** Release claimed USART.
 * \see ::usart_claim
 * \param s The USART DMA state structure.
 */
void usart_release(usart_state* s)
{
  if (s->claim_nest)
    s->claim_nest--;
  else
    chBSemSignal(&s->claimed);
}

/** Returns a lower bound on the number of bytes in the receive buffer.
 * \param s The USART state structure.
 */
u32 usart_n_read(usart_state* s)
{
  if (s->sd == NULL)
    return 0;

  return io_support_n_read(s->sd);
}

/** Calculate the space left in the USART DMA transmit buffer.
 * \param s The USART state structure.
 * \return The number of bytes that may be safely written to the buffer.
 */
u32 usart_tx_n_free(usart_state* s)
{
  if (s->sd == NULL)
    return 0;

  return io_support_tx_n_free(s->sd);
}

/** Read bytes from the USART RX buffer.
 *
 * \param s The USART state structure.
 * \param data Pointer to a buffer where the received data will be stored.
 * \param len The number of bytes to attempt to read.
 * \param timeout Return if this time passes with no reception.
 * \return The number of bytes successfully read from the receive buffer.
 */
u32 usart_read_timeout(usart_state* s, u8 data[], u32 len, u32 timeout)
{
  if ((s->sd == NULL) || (len == 0))
    return 0;

  u32 n = io_support_read_timeout(s->sd, data, len, timeout);
  s->rx.byte_counter += n;
  return n;
}

/** Read bytes from the USART RX buffer.
 *
 * \param s The USART state structure.
 * \param data Pointer to a buffer where the received data will be stored.
 * \param len The number of bytes to attempt to read.
 * \return The number of bytes successfully read from the receive buffer.
 */
u32 usart_read(usart_state* s, u8 data[], u32 len)
{
  return usart_read_timeout(s, data, len, TIME_IMMEDIATE);
}

/**
 * Returns the total bytes divided by the total elapsed seconds since the
 * previous call of this function.
 *
 * \param s The USART stats structure
 */
float usart_throughput(struct usart_stats* s)
{
  float elapsed = ((float)chVTTimeElapsedSinceX(s->last_byte_ticks) /
    (double)CH_CFG_ST_FREQUENCY*1000.0);
  float kbps = s->byte_counter / elapsed;

  s->byte_counter = 0;
  s->last_byte_ticks = chVTGetSystemTime();

  return kbps;
}

/** Write out data over the USART.
 *
 * \param s The USART state structure.
 * \param data A pointer to the data to write out.
 * \param len  The number of bytes to write.
 * \return The number of bytes that will be written, may be less than len.
 */
u32 usart_write(usart_state* s, const u8 data[], u32 len)
{
  if (s->sd == NULL)
    return len;
  u32 n = io_support_write(s->sd, data, len);
  s->tx.byte_counter += n;
  return n;
}

/** \} */

/** \} */
